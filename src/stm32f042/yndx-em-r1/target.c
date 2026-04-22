/*
 * Copyright (c) 2016, Devan Lai
 * Copyright (c) 2019, Unwired Devices LLC <info@unwds.com>
 * Copyright (c) 2026, Yandex LLC <olartam@yandex-team.ru>
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice
 * appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/dma.h>

#include <stdio.h>
#include <string.h>

#include "tick.h"
#include "target.h"
#include "config.h"
#include "console.h"
#include "battery_model.h"
#include "DAP/CMSIS_DAP_config.h"
#include "DAP/CMSIS_DAP.h"
#include "DFU/DFU.h"
#include "USB/vcdc.h"

#define ENABLE_DEBUG    (0)

/* last flash page (1KB on STM32F042) is for settings */
/* so max firmware size is 32768 - 1024 = 31744 bytes */
#define FLASH_CONFIG_PAGE   31 /* last of 32 pages */
#define FLASH_CONFIG_ADDR   (FLASH_BASE + 1024*FLASH_CONFIG_PAGE)
#define FLASH_CONFIG_MAGIC  0xDEADF00D

#define TIM2_PRESCALER_3US      75
#define TIM2_PRESCALER_10US     225
#define TIM2_PRESCALER_50US     1250

#define TIM2_PRESCALER          TIM2_PRESCALER_3US

#if TIM2_PRESCALER == TIM2_PRESCALER_3US
#define ADC_SAMPLE_PERIOD_US    3
#elif TIM2_PRESCALER == TIM2_PRESCALER_10US
#define ADC_SAMPLE_PERIOD_US    10
#elif TIM2_PRESCALER == TIM2_PRESCALER_50US
#define ADC_SAMPLE_PERIOD_US    50
#else
#error "Unsupported TIM2 prescaler"
#endif

#define ADC_SAMPLE_TIME         ADC_SMPTIME_007DOT5

#define DMA_DATA_SIZE           400
static uint16_t dma_data[DMA_DATA_SIZE];

/*
 * Divide positive or negative dividend by positive divisor and round
 * to closest integer. Result is undefined for negative divisors and
 * for negative dividends if the divisor variable type is unsigned.
 */
#define DIV_ROUND_CLOSEST(x, divisor)(			\
{							\
	typeof(x) __x = x;				\
	typeof(divisor) __d = divisor;			\
	(((typeof(x))-1) > 0 ||				\
	 ((typeof(divisor))-1) > 0 || (__x) > 0) ?	\
		(((__x) + ((__d) / 2)) / (__d)) :	\
		(((__x) - ((__d) / 2)) / (__d));	\
}							\
)

/* Reconfigure processor settings */
void cpu_setup(void) {

}

/* Set STM32 to 48 MHz. */
void clock_setup(void) {
    rcc_clock_setup_in_hsi48_out_48mhz();

    /* Trim from USB sync frame */
    crs_autotrim_usb_enable();
    rcc_set_usbclk_source(RCC_HSI48);
}

/* 1000 Hz frequency */
static const uint16_t frequency = 1;

/* 1000 ms blink period */
static const uint32_t blink_period_boot = 1000;

static uint32_t button1_counter = 0;
static uint32_t button2_counter = 0;
static uint32_t button3_counter = 0;
static uint16_t target_release_reset = 0;
static uint16_t target_release_boot = 0;
static uint16_t target_start_measurements = 0;
static bool target_power_state = false;
static bool target_boot_state = false;
static uint32_t blink_counter = 0;
static uint16_t target_power_failure = 0;

static volatile uint32_t adc_voltage_raw = 0;
static volatile uint32_t seconds_passed = 0;
static bool display_counter = false;
static uint32_t display_mode = 0;
static uint32_t vdda = 0;
static uint32_t current_report_counter = 0;
static bool is_interface_connected = false;
static uint64_t energy_accumultated_uwh = 0;
static uint64_t energy_accumultated_uah = 0;
static uint32_t energy_ahr = 0;
static uint32_t energy_whr = 0;
static uint32_t current_max_ua = 0;
static bool update_display = false;

static bool banner_displayed = false;
static uint32_t do_calibrate = 0;
static volatile uint32_t cal_voltage = 0;
static volatile int current_power_range = 0;
static volatile uint8_t cmd_int = 0;
static volatile bool dap_connected = false;

static volatile bool board_v2 = false;

#define USB_COMMAND_SIZE    96

static volatile struct {
    uint32_t current[3];
    uint32_t raw_current[3];
    uint32_t count[3];
    uint32_t voltage;
    uint32_t vcount;
} adc_data;

static volatile struct {
    uint64_t c_acc_01ua_us;    /* charge accumulator in 0.1uA*us */
    fixed_t current_c;         /* current used capacity, Ah */
    fixed_t voltage_slow_part; /* voltage term that depends on C */
    fixed_t voltage_fast_part; /* voltage term that depends on I */
} battery_state;

typedef enum {
    SHOW_SECONDS        = 1 << 0,
    SHOW_VOLTAGE        = 1 << 1,
    SHOW_CURRENT        = 1 << 2,
    SHOW_AMPEREHOURS    = 1 << 3,
    SHOW_WATTHOURS      = 1 << 4,
} show_values_t;

typedef enum {
    CMD_INT_CALIBRATE   = 1 << 0,
    CMD_INT_CONSOLEOUT  = 1 << 1,
    CMD_INT_LCDOUT      = 1 << 2,
    CMD_INT_BATTERY_SLOW = 1 << 3,
} internal_commands_t;

/* emb_settings size must be multiple of 4 */
static volatile struct {
    uint32_t magic;
    uint32_t voltage_coeff;
    uint32_t period;
    uint32_t show;
    uint32_t baudrate;
    uint32_t dap_active;

    /* Battery parameters */
    fixed_t start_c;   /* initial used capacity, Ah */
    fixed_t q;         /* nominal full capacity, Ah */
    fixed_t r;         /* internal resistance, Ohm */

    /* Empirical coefficients */
    fixed_t e0;        /* base voltage, V */
    fixed_t k1;        /* coefficient for ln(1 - C/Q), V */
    fixed_t k2;        /* coefficient for ln(C/Q), V */
    fixed_t a;         /* exponential term amplitude, V */
    fixed_t b;         /* exponential term coefficient, 1/Ah */
} emb_settings;

/* last flash page (1KB on STM32F042) is for settings */
/* so max firmware size is 32768 - 1024 = 31744 bytes */
static void save_settings(void) {
    flash_unlock();
    flash_erase_page(FLASH_CONFIG_ADDR);
    flash_program_word(FLASH_CONFIG_ADDR, FLASH_CONFIG_MAGIC);

    uint32_t *settings = (void *)&emb_settings;
    for (unsigned i = 1; i < sizeof(emb_settings)/4; i++) {
        flash_program_word(FLASH_CONFIG_ADDR + i*4, settings[i]);
    }
    flash_lock();
    vcdc_println("[INF] Settings saved");
}

/* stop measurements on DAP connect */
void DAP_On_Connect(void) {
    vcdc_println("[INF] DAP connect");
    if (!emb_settings.dap_active) {
        dap_connected = true;
        timer_disable_counter(TIM2);
    }
}

/* restart measurements on DAP connect */
void DAP_On_Disconnect(void) {
    vcdc_println("[INF] DAP disconnect");
    if (!emb_settings.dap_active) {
        dap_connected = false;
        timer_enable_counter(TIM2);
    }
}

static void battery_update_fast(uint32_t range, uint32_t raw_sum, uint32_t samples) {
    if (samples == 0) {
        return;
    }

    /*
     * Keep the same conversion path as the existing firmware:
     * raw ADC average -> shunt voltage in 0.1 mV -> current in 0.1 uA.
     */
    uint32_t raw_avg = DIV_ROUND_CLOSEST(raw_sum, samples);
    uint32_t sense_01mv = (vdda * (10 * raw_avg)) / 4095;
    uint32_t current_01ua = 0;

    switch (range) {
        case 0:
            current_01ua = DIV_ROUND_CLOSEST(10 * sense_01mv, 102);
            break;
        case 1:
            current_01ua = 100 * DIV_ROUND_CLOSEST(10 * sense_01mv, 102);
            break;
        case 2:
            current_01ua = 100 * 100 * DIV_ROUND_CLOSEST(10 * sense_01mv, 102);
            break;
        default:
            return;
    }

    /* Charge accumulator in 0.1 uA * us. */
    battery_state.c_acc_01ua_us +=
        (uint64_t)current_01ua * samples * ADC_SAMPLE_PERIOD_US;

    /* Fast voltage part: -R * I, where I is converted to A in Q16.16. */
    fixed_t current_a = (fixed_t)DIV_ROUND_CLOSEST(
        (uint64_t)current_01ua * FIXED_ONE,
        10000000ULL
    );
    battery_state.voltage_fast_part = -fixed_mul(emb_settings.r, current_a);
}

static void battery_update_slow(void) {
    fixed_t c_delta = (fixed_t)DIV_ROUND_CLOSEST(
        battery_state.c_acc_01ua_us,
        549316406250ULL
    );

    battery_state.current_c = fixed_add(battery_state.current_c, c_delta);
    battery_state.c_acc_01ua_us = 0;

    if (emb_settings.q <= 0) {
        return;
    }

    fixed_t ratio1 = fixed_div(battery_state.current_c, emb_settings.q);
    fixed_t arg1 = fixed_sub(FIXED_ONE, ratio1);
    fixed_t ln1 = fixed_ln(arg1);

    fixed_t ln2 = fixed_ln(ratio1);

    fixed_t BC = fixed_mul(emb_settings.b, battery_state.current_c);
    fixed_t negBC = -BC;
    fixed_t exp_term = fixed_mul(emb_settings.a, fixed_exp(negBC));

    fixed_t term1 = fixed_mul(emb_settings.k1, ln1);
    fixed_t term2 = fixed_mul(emb_settings.k2, ln2);
    fixed_t sum = fixed_add(emb_settings.e0, term1);
    sum = fixed_add(sum, term2);
    sum = fixed_add(sum, exp_term);

    battery_state.voltage_slow_part = sum;
}

static fixed_t fixed_from_x1000(long value) {
    return (fixed_t)DIV_ROUND_CLOSEST((int64_t)value * FIXED_ONE, 1000);
}

static long fixed_to_x1000(fixed_t value) {
    return (long)DIV_ROUND_CLOSEST((int64_t)value * 1000, FIXED_ONE);
}

static void print_battery_params(void) {
    char str[96];
    snprintf(str, sizeof(str),
             "[PAR] params %ld %ld %ld %ld %ld %ld %ld %ld",
             fixed_to_x1000(emb_settings.start_c),
             fixed_to_x1000(emb_settings.q),
             fixed_to_x1000(emb_settings.r),
             fixed_to_x1000(emb_settings.e0),
             fixed_to_x1000(emb_settings.k1),
             fixed_to_x1000(emb_settings.k2),
             fixed_to_x1000(emb_settings.a),
             fixed_to_x1000(emb_settings.b));
    vcdc_println(str);
}

static void disable_power(void) {
    /* Stop TIM2 */
    timer_disable_counter(TIM2);
    
    /* disable DMA channel */
    dma_disable_channel(DMA1, DMA_CHANNEL1);
    adc_disable_dma(ADC1);
    
    /* Stop ADC */
    adc_power_off(ADC1);
    
    /* Disable output power */
    gpio_clear(PWR_EXTX1_EN_PORT, PWR_EXTX1_EN_PIN);
	gpio_clear(PWR_DCDC_EN_PORT,  PWR_DCDC_EN_PIN);
	gpio_clear(PWR_USBC_EN_PORT,  PWR_USBC_EN_PIN);

    target_power_state = false;
    target_boot_state = false;
    
    gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
    gpio_clear(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
    gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
	gpio_clear(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
    
    vcdc_println("[INF] power disabled");
}

static void adc_setup_common(void) {
    rcc_periph_clock_enable(RCC_ADC1);
    
    gpio_mode_setup(CURRENT_SENSE_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, CURRENT_SENSE_PIN);
    
    adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
    adc_disable_discontinuous_mode(ADC1);
    adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_TIM2_TRGO, ADC_CFGR1_EXTEN_RISING_EDGE);
    adc_set_right_aligned(ADC1);
    adc_disable_temperature_sensor();
    adc_disable_dma(ADC1);
    
    adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
    
    /* set 1 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SAMPLE_TIME);
    
    /* Disable end of conversion IRQ */
    adc_disable_eoc_interrupt(ADC1);
    
    /* Enable VREF */
    adc_enable_vrefint();
    
    /* Enable analog watchdog on PB0 */
    adc_enable_analog_watchdog_on_selected_channel(ADC1, 8);
    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_HT) | ADC_TR1_HT_VAL(CURRENT_HIGHER_THRESHOLD);
    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_LT) | ADC_TR1_LT_VAL(CURRENT_LOWER_THRESHOLD);
    
    /* Analog watchdog interrupt highest priority */
    nvic_set_priority(NVIC_ADC_COMP_IRQ, 0);
    nvic_enable_irq(NVIC_ADC_COMP_IRQ);
    
    /* DMA interrupt with medium priority */
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 64);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
    
    /* SysTick interrupt with low priority */
    nvic_set_priority(NVIC_SYSTICK_IRQ, 128);

    rcc_periph_clock_enable(RCC_DMA1);
    
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    
    /* Medium priority. */
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_MEDIUM);

    /* ADC is 16 bit */
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);

    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_disable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
    
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_half_transfer_interrupt(DMA1, DMA_CHANNEL1);
    
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)(&ADC1_DR));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)(dma_data));
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, DMA_DATA_SIZE);
    
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    
    dma_enable_channel(DMA1, DMA_CHANNEL1);
    
    /* overwrite data in case of overrun */
    ADC_CFGR1(ADC1) |= ADC_CFGR1_OVRMOD;
    
    /* enable DMA circular mode */
    ADC_CFGR1(ADC1) |= ADC_CFGR1_DMACFG;
}

static void adc_measure_current(void) {
    /* stop any ongoing conversion */
    ADC_CR(ADC1) = ADC_CR_ADSTP;
    while (ADC_CR(ADC1) & ADC_CR_ADSTP) { }

    /* Measurements to be triggered by TIM2 */
    adc_enable_external_trigger_regular(ADC1, ADC_CFGR1_EXTSEL_TIM2_TRGO, ADC_CFGR1_EXTEN_RISING_EDGE);
    
    /* PB0 channel */
    uint8_t adc_channels = 8;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    /* Enable watchdog interrupt */
    adc_enable_watchdog_interrupt(ADC1);

    adc_enable_dma(ADC1);
    
    /* start ADC measurements */
    adc_start_conversion_regular(ADC1);
    
    /* start TIM2 */
    timer_enable_counter(TIM2);
}

static void adc_measure_vdda(void) {
    /* stop any ongoing conversion */
    ADC_CR(ADC1) = ADC_CR_ADSTP;
    while (ADC_CR(ADC1) & ADC_CR_ADSTP) { }
    
    adc_disable_dma(ADC1);
    
    /* disable ADC */
    adc_power_off(ADC1);
    
    /* calibration and clock source selection must be done with ADC disabled */   
    adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
    adc_calibrate(ADC1);
    
    /* enable ADC */
    adc_power_on(ADC1);
    
    /* ~5 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
    
    /* ADC will be run once */
    adc_disable_external_trigger_regular(ADC1);
    
    /* disable analog watchdog interrupt */
    adc_disable_watchdog_interrupt(ADC1);
    
    /* VREF channel only */
    uint8_t adc_channels = 17;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    /* discard old data if present */
    adc_read_regular(ADC1);
    
    /* start ADC and wait for it to finish */
    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));

    /* VREF in ADC counts */
    vdda = adc_read_regular(ADC1);
    
    uint16_t cal_vref = ST_VREFINT_CAL;
    
    /* VDDA in millivolts */
    vdda = (3300 * cal_vref) / vdda;
    
    char vdda_str[20];
    snprintf(vdda_str, 20, "[VDD] %lu", vdda);
    vcdc_println(vdda_str);
    
    /* set 1 us sampling time */
    adc_set_sample_time_on_all_channels(ADC1, ADC_SAMPLE_TIME);
}

static uint32_t adc_measure_voltage(void) {
    /* stop any ongoing conversion */
    ADC_CR(ADC1) = ADC_CR_ADSTP;
    while (ADC_CR(ADC1) & ADC_CR_ADSTP) { }
    
    adc_disable_dma(ADC1);
    
    /* ADC will be run once */
    adc_disable_external_trigger_regular(ADC1);
    
    /* disable analog watchdog interrupt */
    adc_disable_watchdog_interrupt(ADC1);
    
    /* VREF channel only */
    uint8_t adc_channels = 9;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    /* discard old data if present */
    adc_read_regular(ADC1);

    /* start ADC and wait for it to finish */
    adc_start_conversion_regular(ADC1);
    while (!(adc_eoc(ADC1)));
    uint32_t adc_read = adc_read_regular(ADC1);

    uint32_t voltage = (emb_settings.voltage_coeff * vdda * adc_read) / (4095*10);
    
    return voltage;
}

static void calibrate_voltage(uint32_t cal_value) {
    adc_measure_vdda();
    
    /* PB1 channel only */
    uint8_t adc_channels = 9;
    adc_set_regular_sequence(ADC1, 1, &adc_channels);
    
    uint32_t voltage_mv = 0;
    
    for (int i = 0; i < 10; i++) {
        /* start ADC and wait for it to finish */
        adc_start_conversion_regular(ADC1);
        while (!(adc_eoc(ADC1)));

        voltage_mv += adc_read_regular(ADC1);
        
        /* 100 us delay between measurements */
        volatile uint32_t k = 4800;
        do {
            k--;
        } while (k);
    }
    voltage_mv = (vdda * voltage_mv) / 4095;
    
    /* coeff is x10 for better accuracy */
    emb_settings.voltage_coeff = (100 * cal_value)/voltage_mv;
    
    char coeff_str[15];
    snprintf(coeff_str, 15, "[CAL] %lu", emb_settings.voltage_coeff);
    vcdc_println(coeff_str);
    
    adc_power_off(ADC1);
    
    save_settings();
}

void dma1_channel1_isr(void) {
    /* using local variables for data processing */
    uint32_t data = 0;
    uint32_t range = current_power_range;
    
    /* half transfer event */
    if ((DMA1_ISR & DMA_ISR_HTIF1) != 0) {
        for (int i = 0; i < DMA_DATA_SIZE/2; i++) {
            data += dma_data[i];
        }
        battery_update_fast(range, data, DMA_DATA_SIZE/2);
        data = DIV_ROUND_CLOSEST(data, DMA_DATA_SIZE/2);
        adc_data.raw_current[range] += data;
        adc_data.count[range] += 1;
    }
    
    /* transfer completed event */
    if ((DMA1_ISR & DMA_ISR_TCIF1) != 0) {
        for (int i = DMA_DATA_SIZE/2; i < DMA_DATA_SIZE; i++) {
            data += dma_data[i];
        }
        battery_update_fast(range, data, DMA_DATA_SIZE/2);
        data = DIV_ROUND_CLOSEST(data, DMA_DATA_SIZE/2);
        adc_data.raw_current[range] += data;
        adc_data.count[range] += 1;
    }
    
    /* clear DMA interrupt flags */
    DMA1_IFCR |= DMA_IFCR_CGIF1;
}

/* analog watchdog interrupt */
void adc_comp_isr(void)
{
    /* stop ADC timer */
    TIM_CR1(TIM2) &= ~TIM_CR1_CEN;

    uint16_t adc_sample = ADC_DR(ADC1);
    
    /* do not switch ranges if it was a short glitch or other ISR (should not happen) */
    if (!(ADC_ISR(ADC1) & ADC_ISR_AWD1) || 
       ((adc_sample > CURRENT_LOWER_THRESHOLD) && (adc_sample < CURRENT_HIGHER_THRESHOLD))) {
        /* reset ADC watchdog flag */
        ADC_ISR(ADC1) |= ADC_ISR_AWD1;
        /* restart timer */
        TIM_CR1(TIM2) |= TIM_CR1_CEN;
        return;
    }
    
    /* disable DMA channel */
    DMA_CCR(DMA1, DMA_CHANNEL1) &= ~DMA_CCR_EN;

    /* using local variables for data processing */
    int range = current_power_range;
    int delta = 0;
    
    while (ADC_ISR(ADC1) & ADC_ISR_AWD1)
    {
        /* reset ADC watchdog flag */
        ADC_ISR(ADC1) |= ADC_ISR_AWD1;
        
        if (adc_sample < CURRENT_LOWER_THRESHOLD) {
            if ((range + delta) != 0) {
                if ((range + delta) == 2) {
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                    GPIO_BRR(CURRENT_RANGE2_PORT) = CURRENT_RANGE2_PIN;
                    
                    /* reenable watchdog high threshold */
                    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_HT) | ADC_TR1_HT_VAL(CURRENT_HIGHER_THRESHOLD);
                }
                else { /* range 1 */
                    /* disable watchdog low threshold by setting it to 0 */
                    ADC_TR1(ADC1) = (ADC_TR1(ADC1) & ~ADC_TR1_LT);
                    
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE0_PORT) = CURRENT_RANGE0_PIN;
                    GPIO_BRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                }
                delta--;
            } else { /* lowest range already */
                
            }
        } else {        /* adc_sample > CURRENT_HIGHER_THRESHOLD */
            if ((range + delta) != 2) {
                if ((range + delta) == 0) {
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                    GPIO_BRR(CURRENT_RANGE0_PORT) = CURRENT_RANGE0_PIN;

                    /* reenable watchdog low threshold */
                    ADC_TR1(ADC1) |= ADC_TR1_LT_VAL(CURRENT_LOWER_THRESHOLD);
                } else {  /* range 2 */
                    /* make-before-break! */
                    GPIO_BSRR(CURRENT_RANGE2_PORT) = CURRENT_RANGE2_PIN;
                    GPIO_BRR(CURRENT_RANGE1_PORT) = CURRENT_RANGE1_PIN;
                    
                    /* disable watchdog high threshold by setting it to maximum */
                    ADC_TR1(ADC1) |= ADC_TR1_HT;
                }
                delta++;
            }
        }
        if ((range + delta) == 1) {
            /* restart data acquisition with small delay */
            ADC_ISR(ADC1) |= ADC_ISR_EOC;
            TIM_CNT(TIM2) = 0;
            TIM_CR1(TIM2) |= TIM_CR1_CEN;
            
            /* sample ADC to check if another correction is needed */
            while (!(ADC_ISR(ADC1) & ADC_ISR_EOC));
            TIM_CR1(TIM2) &= ~TIM_CR1_CEN;
            ADC_ISR(ADC1) |= ADC_ISR_EOC;
        } else {
            break;
        }
    }

    current_power_range += delta;

    uint32_t data = 0;
    uint32_t size = DMA_DATA_SIZE - DMA_CNDTR(DMA1, DMA_CHANNEL1);
    
    /* copy data to temporary array if needed */
    uint16_t data_tmp[DMA_DATA_SIZE/2];
    uint16_t *data_ptr;

    if (size) {
        if (size > DMA_DATA_SIZE/2) {
            size -= DMA_DATA_SIZE/2;
            data_ptr = &dma_data[DMA_DATA_SIZE/2];
        } else {
            data_ptr = data_tmp;
            memcpy((void *)data_ptr, (void *)dma_data, size * sizeof(uint16_t));
        }
    }
    
    /* reset DMA channel data counter and interrupt flags */
    DMA_CNDTR(DMA1, DMA_CHANNEL1) = DMA_DATA_SIZE;
    DMA1_IFCR |= DMA_IFCR_CGIF1;
    /* reenable DMA channel */
    DMA_CCR(DMA1, DMA_CHANNEL1) |= DMA_CCR_EN;

    /* restart acquisition timer */
    TIM_CNT(TIM2) = 0;
    TIM_CR1(TIM2) |= TIM_CR1_CEN;

    /* clear possible pending watchdog interrupt */
    NVIC_ICPR(NVIC_ADC_COMP_IRQ / 32) = (1 << (NVIC_ADC_COMP_IRQ % 32));

    /* process data */
    if (size) {
        for (uint32_t i = 0; i < size; i++) {
            data += data_ptr[i];
        }

        battery_update_fast(range, data, size);
        data = DIV_ROUND_CLOSEST(data, size);
        adc_data.raw_current[range] += data;
        adc_data.count[range] += 1;
    }
}

static void console_command_parser(uint8_t *usb_command) {
    const char *help_period = "period <ms> - set period in milliseconds, 10 to 1000";
    const char *help_iface = "iface <on|off> - enable/disable UART and SWD interfaces";
    const char *help_power = "power <on|off> - enable/disable onboard DC/DC";
    const char *help_maxreset = "maxreset - reset maximum current";
    const char *help_display = "display <N> - set display mode by number";
    const char *help_calibrate = "calibrate <mV> - calibrate voltage divider";
    const char *help_show = "show <SEC|VOL|CUR|AHR|WHR|all> - report values";
    const char *help_hide = "hide <SEC|VOL|CUR|AHR|WHR|all> - don't report values";
    const char *help_baudrate = "baudrate <bps> - set target UART baudrate";
    const char *help_reset = "reset - reset target";
    const char *help_boot = "boot - switch target to bootloader mode";
    const char *help_dap = "dap <on|off> - stop current measurements when DAP is active";
    const char *help_params = "params [<C> <Q> <R> <E0> <k1> <k2> <a> <b>] - get/set battery model parameters, x1000";

    int cmdlen;

    if (memcmp((char *)usb_command, "help", strlen("help")) == 0) {
        vcdc_println(help_period);
        vcdc_println(help_reset);
        vcdc_println(help_boot);
        vcdc_println(help_iface);
        vcdc_println(help_power);
        vcdc_println(help_display);
        vcdc_println(help_maxreset);
        vcdc_println(help_calibrate);
        vcdc_println(help_show);
        vcdc_println(help_hide);
        vcdc_println(help_baudrate);
        vcdc_println(help_dap);
        vcdc_println(help_params);
    }
    else
    if (memcmp((char *)usb_command, "maxreset", strlen("maxreset")) == 0) {
        current_max_ua = 0;
    }
    else
    if (memcmp((char *)usb_command, "params", cmdlen = strlen("params")) == 0) {
        if (((char *)usb_command)[cmdlen] == 0) {
            print_battery_params();
            return;
        }

        if (((char *)usb_command)[cmdlen] != ' ') {
            vcdc_println(help_params);
            return;
        }

        char *ptr = (char *)&usb_command[cmdlen + 1];
        char *endptr;
        long start_c;
        long q;
        long r;
        long e0;
        long k1;
        long k2;
        long a;
        long b;

        start_c = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        q = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        r = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        e0 = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        k1 = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        k2 = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        a = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }
        ptr = endptr;

        b = strtol(ptr, &endptr, 10);
        if (endptr == ptr) {
            vcdc_println(help_params);
            return;
        }

        if ((start_c < 0) || (q <= 0) || (r < 0) || (start_c > q)) {
            vcdc_println(help_params);
            return;
        }

        emb_settings.start_c = fixed_from_x1000(start_c);
        emb_settings.q = fixed_from_x1000(q);
        emb_settings.r = fixed_from_x1000(r);
        emb_settings.e0 = fixed_from_x1000(e0);
        emb_settings.k1 = fixed_from_x1000(k1);
        emb_settings.k2 = fixed_from_x1000(k2);
        emb_settings.a = fixed_from_x1000(a);
        emb_settings.b = fixed_from_x1000(b);

        battery_state.c_acc_01ua_us = 0;
        battery_state.current_c = emb_settings.start_c;
        battery_state.voltage_fast_part = 0;
        battery_state.voltage_slow_part = 0;
        battery_update_slow();

        save_settings();
        vcdc_println("[INF] Battery parameters saved");
        print_battery_params();
    }
    else
    if (memcmp((char *)usb_command, "period ", cmdlen = strlen("period ")) == 0) {
        int period = strtol((char *)&usb_command[cmdlen], NULL, 10);

        if ((period >= 10) && (period <= 1000)) {
            vcdc_print("[INF] Period is ");
            char str[10];
            snprintf(str, 10, "%d", period);
            vcdc_print(str);
            vcdc_println(" ms");
            
            emb_settings.period = period;
            save_settings();
        }
        else {
            vcdc_println(help_period);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "baudrate ", cmdlen = strlen("baudrate ")) == 0) {
        int baudrate = strtol((char *)&usb_command[cmdlen], NULL, 10);

        if (baudrate != 0) {
            vcdc_print("[INF] Baudrate is ");
            char str[10];
            snprintf(str, 10, "%d", baudrate);
            vcdc_print(str);
            vcdc_println(" bps");
            
            console_reconfigure(baudrate, 8, USART_STOPBITS_1, USART_PARITY_NONE);
            
            emb_settings.baudrate = baudrate;
            save_settings();
        }
        else {
            vcdc_println(help_baudrate);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "iface ", cmdlen = strlen("iface ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "on", 2) == 0) {
            is_interface_connected = false;
            button2_counter = 100;
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "off", 3) == 0) {
            is_interface_connected = true;
            button2_counter = 100;
        }
        else {
            vcdc_println(help_iface);
            vcdc_send_buffer_space();
        }
    }
    if (memcmp((char *)usb_command, "dap ", cmdlen = strlen("dap ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "on", 2) == 0) {
            emb_settings.dap_active = 1;
            save_settings();
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "off", 3) == 0) {
            emb_settings.dap_active = 0;
            save_settings();
        }
        else {
            vcdc_println(help_iface);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "power ", cmdlen = strlen("power ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "on", 2) == 0) {
            target_power_state = false;
            button1_counter = 100;
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "off", 3) == 0) {
            target_power_state = true;
            button1_counter = 100;
        }
        else
        {
            vcdc_println(help_power);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "display ", cmdlen = strlen("display ")) == 0) {
        display_mode = strtol((char *)&usb_command[cmdlen], NULL, 10);
    }
    else
    if (memcmp((char *)usb_command, "reset ", cmdlen = strlen("reset ")) == 0) {
        #if nRESET_GPIO_INVERT
            gpio_set(TARGET_RESET_PORT, TARGET_RESET_PIN);
        #else
            gpio_clear(TARGET_RESET_PORT, TARGET_RESET_PIN);
        #endif
        
        target_release_reset = 100; /* 100 ms */
    }
    else
    if (memcmp((char *)usb_command, "boot ", cmdlen = strlen("boot ")) == 0) {
        #if nRESET_GPIO_INVERT
            gpio_set(TARGET_RESET_PORT, TARGET_RESET_PIN);
        #else
            gpio_clear(TARGET_RESET_PORT, TARGET_RESET_PIN);
        #endif
        
        gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
        
        target_release_boot = 750; /* 750 ms */
        target_release_reset = 100; /* 100 ms */
    }
    else
    if (memcmp((char *)usb_command, "calibrate ", cmdlen = strlen("calibrate ")) == 0) {
        cal_voltage = strtol((char *)&usb_command[cmdlen], NULL, 10);
        if ((cal_voltage > 0) && (cal_voltage < 20000)) {
            /* delay before calibration 1500 ms */
            do_calibrate = 1500;
            
            char str[20] = { };
            snprintf(str, 20, "CAL %lu", cal_voltage);
            
            // tic33m_display_string(&tic33m_dev, str, strlen(str));
            current_report_counter = 1;
        } else {
            vcdc_println(help_calibrate);
        }
    }
    else
    if (memcmp((char *)usb_command, "show ", cmdlen = strlen("show ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "all", 3) == 0) {
            emb_settings.show = 0xFF;
            save_settings();
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "SEC", 3) == 0) {
            if (!(emb_settings.show & SHOW_SECONDS)) {
                emb_settings.show |= SHOW_SECONDS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "VOL", 3) == 0) {
            if (!(emb_settings.show & SHOW_VOLTAGE)) {
                emb_settings.show |= SHOW_VOLTAGE;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "CUR", 3) == 0) {
            if (!(emb_settings.show & SHOW_CURRENT)) {
                emb_settings.show |= SHOW_CURRENT;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "AHR", 3) == 0) {
            if (!(emb_settings.show & SHOW_AMPEREHOURS)) {
                emb_settings.show |= SHOW_AMPEREHOURS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "WHR", 3) == 0) {
            if (!(emb_settings.show & SHOW_WATTHOURS)) {
                emb_settings.show |= SHOW_WATTHOURS;
                save_settings();
            }
        }
        else
        {
            vcdc_println(help_show);
            vcdc_send_buffer_space();
        }
    }
    else
    if (memcmp((char *)usb_command, "hide ", cmdlen = strlen("hide ")) == 0) {
        if (memcmp((char *)&usb_command[cmdlen], "all", 3) == 0) {
            emb_settings.show = 0;
            save_settings();
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "SEC", 3) == 0) {
            if (emb_settings.show & SHOW_SECONDS) {
                emb_settings.show &= ~SHOW_SECONDS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "VOL", 3) == 0) {
            if (emb_settings.show & SHOW_VOLTAGE) {
                emb_settings.show &= ~SHOW_VOLTAGE;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "CUR", 3) == 0) {
            if (emb_settings.show & SHOW_CURRENT) {
                emb_settings.show &= ~SHOW_CURRENT;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "AHR", 3) == 0) {
            if (emb_settings.show & SHOW_AMPEREHOURS) {
                emb_settings.show &= ~SHOW_AMPEREHOURS;
                save_settings();
            }
        }
        else
        if (memcmp((char *)&usb_command[cmdlen], "WHR", 3) == 0) {
            if (emb_settings.show & SHOW_WATTHOURS) {
                emb_settings.show &= ~SHOW_WATTHOURS;
                save_settings();
            }
        }
        else
        {
            vcdc_println(help_hide);
            vcdc_send_buffer_space();
        }
    }
}

/* ticks every 1 ms */
void systick_activity(void)
{
    current_report_counter++;
    
    /* every 10 ms */
    if (current_report_counter && (current_report_counter % 10 == 0)) {
        // tic33m_lclk(&tic33m_dev);
    }
    
    /* every 100 ms */
    if (current_report_counter && (current_report_counter % 100 == 0)) {
        /* console command parser */
        static uint8_t usb_command[USB_COMMAND_SIZE + 1];
        size_t cmd_size = vcdc_recv_buffered(usb_command, USB_COMMAND_SIZE);
        if (cmd_size != 0) {
            usb_command[cmd_size] = 0;
            console_command_parser(usb_command);
        }
    }
    
    if (do_calibrate) {
        do_calibrate--;
        
        if (!do_calibrate) {
            /* calibration at 3.0V */
            cmd_int |= CMD_INT_CALIBRATE;
        }
    }
    
    /* only once 500 ms after start */
    if ((!banner_displayed) && (current_report_counter == 500)) {
        #if defined(BANNER_STR1)
            vcdc_println(BANNER_STR1);
        #endif
        #if defined(BANNER_STR2)
            vcdc_println(BANNER_STR2);
        #endif
        #if defined(BANNER_STR3)
            vcdc_println(BANNER_STR3);
        #endif
        banner_displayed = true;
        
        if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
            vcdc_println("[ERR] Configuration NOT loaded");
        } else {
            vcdc_println("[INF] Configuration loaded");
        }
        
        char str[30];
        snprintf(str, 30, "[INF] Period is %lu ms", emb_settings.period);
        vcdc_println(str);
    }
    
    /* every 'emb_settings.period' ms */
    if (current_report_counter &&
       (current_report_counter % emb_settings.period == 0) &&
        !dap_connected) {
        /* calculate average ADC value */
        if (target_power_state && !target_start_measurements) {
            /* disable timer and DMA channel */
            timer_disable_counter(TIM2);
            adc_disable_dma(ADC1);
            /* measure voltage */
            adc_data.voltage = adc_measure_voltage();

            /* convert to millivolts x10 */
            for (int i = 0; i < 3; i++) {
                if (adc_data.count[i]) {
                    adc_data.current[i] = (vdda * (DIV_ROUND_CLOSEST(10*adc_data.raw_current[i], adc_data.count[i]))) / 4095;
#if ENABLE_DEBUG
                    char str[30];
                    snprintf(str, 30, "RAW ADC: %d: ", i);
                    vcdc_print(str);
                    
                    snprintf(str, 30, "%lu / ", adc_data.raw_current[i]);
                    vcdc_print(str);

                    snprintf(str, 30, "%lu = ", adc_data.count[i]);
                    vcdc_print(str);

                    snprintf(str, 30, "%lu", DIV_ROUND_CLOSEST(adc_data.raw_current[i], adc_data.count[i]));
                    vcdc_println(str);
#endif
                } else {
                    adc_data.current[i] = 0;
                }
                adc_data.raw_current[i] = 0;
                adc_data.count[i] = 0;
            }
            
            adc_measure_current();
            
            cmd_int |= CMD_INT_CONSOLEOUT;
        }
    }
    
    if (target_release_reset) {
        if (--target_release_reset == 0) {
            /* Release reset */
#if nRESET_GPIO_INVERT
            gpio_clear(TARGET_RESET_PORT, TARGET_RESET_PIN);
#else
            gpio_set(TARGET_RESET_PORT, TARGET_RESET_PIN);
#endif
            vcdc_println("[INF] target reset");
        }
    }
    
    if (target_start_measurements) {
        if (--target_start_measurements == 0) {
            for (int i = 0; i < 3; i++) {
                adc_data.raw_current[i] = 0;
                adc_data.count[i] = 0;
            }
            
            adc_setup_common();
            adc_measure_vdda();
            adc_measure_current();
        }
    }

    /* Blink a LED if target bootloader mode active */
    if (target_boot_state) {
        if (--blink_counter == 0) {
            blink_counter = blink_period_boot;
			// N/A on YNDX version
            // gpio_toggle(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
        }
    }

    if (target_release_boot) {
        if (--target_release_boot == 0) {
            /* Release boot */
            gpio_set(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
            vcdc_println("[INF] target bootloader activated");
        }
    }
    
    /* Check if INTERFACE DISCONNECT button is pressed */
	// N/A in YNDX version
	/*
    if (gpio_get(TARGET_IFACE_BTN_PORT, TARGET_IFACE_BTN_PIN) == 0) {
        button2_counter++;
    } else { // button released
        if (button2_counter >= 100) {
            if (is_interface_connected) {
                gpio_clear(TARGET_IFACE_EN_PORT, TARGET_IFACE_EN_PIN);
                is_interface_connected = false;
            } else {
                gpio_set(TARGET_IFACE_EN_PORT, TARGET_IFACE_EN_PIN);
                is_interface_connected = true;
            }
        }
        button2_counter = 0;
    }
	*/
    
    /* Check if Button 1 is pressed */
    if (gpio_get(BTN1_GPIO_PORT, BTN1_GPIO_PIN) == 0) {
        if (++button3_counter >= 5000) {
            /* delay before calibration 1500 ms */
            do_calibrate = 1500;
            
            /* calibrate @ 3000 mV */
            cal_voltage = 3000;
            // tic33m_display_string(&tic33m_dev, "CAL 3000", strlen("CAL 3000"));
            current_report_counter = 1;
        }
    } else {
        /* button released */
        if ((button3_counter >= 100) && (button3_counter < 5000)) {
            if (++display_mode == 3) {
                display_mode = 0;
            }
            update_display = true;
        }
        button3_counter = 0;
    } 

    /* Check if Button 2 is pressed */
    if (gpio_get(nBOOT0_GPIO_PORT, nBOOT0_GPIO_PIN) != 0) {
        if ((++button1_counter == 1000) && target_power_state) {
            /* 1000 ms long press */
            target_boot_state = !target_boot_state;
            /* Reset target */
#if nRESET_GPIO_INVERT
            gpio_set(TARGET_RESET_PORT, TARGET_RESET_PIN);
#else
            gpio_clear(TARGET_RESET_PORT, TARGET_RESET_PIN);
#endif
        
            if (target_boot_state) {            
                /* Boot from system memory */
                gpio_clear(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
                target_release_boot = 500;
            } else {
                /* Enable green LED */
                if (!board_v2) {
                    gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
                }
            }            
            target_release_reset = 1;
        }
    } else {
        /* button released */
        if ((button1_counter >= 100) && (button1_counter < 1000)) {
            /* 100 ms short press */    
            target_power_state = !target_power_state;

            /* Toggle power */
            if (target_power_state) {
                /* Reset target */
#if nRESET_GPIO_INVERT
                gpio_set(TARGET_RESET_PORT, TARGET_RESET_PIN);
#else
                gpio_clear(TARGET_RESET_PORT, TARGET_RESET_PIN);
#endif
                /* Set boot from flash */
                gpio_set(TARGET_BOOT_PORT, TARGET_BOOT_PIN); /* inverted */
                /* Reset accumulated energy */
                energy_accumultated_uah = 0;
                energy_accumultated_uwh = 0;
                seconds_passed = 0;
                /* Enable current shunt (range 2 by default) */
                gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
                gpio_set(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
                gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
				gpio_clear(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
                current_power_range = 1;
                current_max_ua = 0;

                target_boot_state = false;
                target_power_failure = 0;
                
                target_release_reset = 100;
                
                /* start current monitoring in 100 ms */
                target_start_measurements = 100;
                
                vcdc_println("[INF] power enabled");
            } else {
                disable_power();
            }
        }
        button1_counter = 0;
    }
    
    /* every second */
    if (current_report_counter && (current_report_counter % 1000 == 0)) {
        if (target_power_state) {
            seconds_passed += 1;
            cmd_int |= CMD_INT_BATTERY_SLOW;
        }
    }
    
    /* every 2 seconds */
    if (current_report_counter && (current_report_counter % 2000 == 0)) {
        cmd_int |= CMD_INT_LCDOUT;
    }
    
    /* every hour - ADC recalibration and VDDA measurement */
    if (current_report_counter == 3600000) {
        timer_disable_counter(TIM2);
        adc_measure_vdda();
        adc_measure_current();
        current_report_counter = 0;
    }
    
    if (update_display) {
        cmd_int |= CMD_INT_LCDOUT;
    }

    if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM3, TIM_SR_CC1IF);

        /* Calculate and set the next compare value. */
        uint16_t new_time = timer_get_counter(TIM3) + frequency;

        timer_set_oc_value(TIM3, TIM_OC1, new_time);
    }
}

void user_activity(void) {
    if (cmd_int & CMD_INT_CALIBRATE) {
        cmd_int &= ~CMD_INT_CALIBRATE;
        calibrate_voltage(cal_voltage);
        disable_power();
    }

    if (cmd_int & CMD_INT_BATTERY_SLOW) {
        cmd_int &= ~CMD_INT_BATTERY_SLOW;
        battery_update_slow();
    }
    
    char cur_str[30] = { 0 };
    
    if (cmd_int & CMD_INT_CONSOLEOUT) {
#if defined(STACK_CANARY_WORD) && ENABLE_DEBUG
        snprintf(cur_str, 30, "[DEV] MSP %u bytes", check_stack_size());
        vcdc_println(cur_str);
#endif

        cmd_int &= ~CMD_INT_CONSOLEOUT;

        uint32_t current = 0;
        uint32_t n = 0;
        if (adc_data.current[0]) {
            n++;
        }
        if (adc_data.current[1]) {
            n++;
        }
        if (adc_data.current[2]) {
            n++;
        }
        
        current = (DIV_ROUND_CLOSEST(10 * adc_data.current[0], 102)) +
                  (100 * DIV_ROUND_CLOSEST(10 * adc_data.current[1], 102)) + 
                  (100 * 100 * DIV_ROUND_CLOSEST(10 * adc_data.current[2], 102));
        
        if (n > 1) {
            current /= n;
        }
        
        if (current > current_max_ua) {
            current_max_ua = current;
        }
        
        /* energy */
        energy_accumultated_uah += current;
        energy_accumultated_uwh += DIV_ROUND_CLOSEST(current * adc_data.voltage, 1000);
        
        /* convert to microampere-hours */
        energy_ahr = energy_accumultated_uah/(3600*10*(1000/emb_settings.period));
        /* convert to microwatt-hours */
        energy_whr = energy_accumultated_uwh/(3600*10*(1000/emb_settings.period));

        if (emb_settings.show & SHOW_SECONDS) {
            snprintf(cur_str, 30, "[SEC] %lu", seconds_passed);
            vcdc_println(cur_str);
        }
        
        if (emb_settings.show & SHOW_VOLTAGE) {
            snprintf(cur_str, 30, "[VOL] %lu", adc_data.voltage);
            vcdc_println(cur_str);
        }

        if (emb_settings.show & SHOW_CURRENT) {
            snprintf(cur_str, 30, "[CUR] %lu.%lu", DIV_ROUND_CLOSEST(current, 10), current % 10);
#if ENABLE_DEBUG
            vcdc_print(cur_str);

            snprintf(cur_str, 30, " (%lu - %lu - %lu)", adc_data.current[0], adc_data.current[1], adc_data.current[2]);
            vcdc_println(cur_str);
#else
            vcdc_println(cur_str);
#endif
        }
        
        if (emb_settings.show & SHOW_AMPEREHOURS) {
            snprintf(cur_str, 30, "[AHR] %lu", energy_ahr);
            vcdc_println(cur_str);
        }
        
        if (emb_settings.show & SHOW_WATTHOURS) {
            snprintf(cur_str, 30, "[WHR] %lu", energy_whr);
            vcdc_println(cur_str);
        }
    }

    if (cmd_int & CMD_INT_LCDOUT) {
        cmd_int &= ~CMD_INT_LCDOUT;
        if (display_counter || update_display) {
            char lcdtext[11] = { };
            memset(lcdtext, ' ', 10);
            
            int precision;
            uint32_t value;
            
            switch (display_mode) {
                case 1:
                    /* maximum current  */
                    value = DIV_ROUND_CLOSEST(current_max_ua, 10);
                    lcdtext[0] = 'I';
                    precision = 3;
                    break;
                case 2:
                    /* microwatt-hours */
                    value = energy_whr;
                    lcdtext[0] = 'P';
                    precision = 3;
                    break;
                default:
                    /* microampere-hours */
                    value = energy_ahr;
                    lcdtext[0] = 'C';
                    precision = 3;
                    break;
            }
            
            snprintf(cur_str, 10, "%lu", value);
            int len = strlen(cur_str);

            if ((len > 8) || ((len > 7) && (precision > 0))) {
                strcpy(&lcdtext[1], "       OL");
            } else {
                /* add forwarding zeros if needed */
                if (len <= precision) {
                    memmove(&cur_str[1 + precision - len], cur_str, len + 1);
                    memset(cur_str, '0', 1 + precision - len);
                    len = strlen(cur_str);
                }
                memcpy(&lcdtext[9 - len], cur_str, len - precision + 1);

                /* add decimal point if needed */
                if (precision) {
                    lcdtext[9 - precision] = '.';
                    memcpy(&lcdtext[9 - precision + 1], &cur_str[len - precision], precision + 1);
                }
            }

            // tic33m_display_string(&tic33m_dev, lcdtext, strlen(lcdtext));
            
            if (!update_display) {
                display_counter = false;
            }
        } else {
            /* display time */
            // tic33m_display_time(&tic33m_dev, seconds_passed);
            display_counter = true;
        }
        update_display = false;
    }
}

/* starts ADC conversion every ~3 us */
static void tim2_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM2);

    /* Generates ~3 us clock */
    rcc_periph_reset_pulse(RST_TIM2);
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(TIM2, 1);
    timer_set_prescaler(TIM2, TIM2_PRESCALER);
    timer_set_clock_division(TIM2, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(TIM2, TIM_CR2_MMS_UPDATE);
}

static void button_setup(void) {
    gpio_mode_setup(BTN1_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BTN1_GPIO_PIN);
    /* Set BOOT0 pin to an input */
    gpio_mode_setup(nBOOT0_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, nBOOT0_GPIO_PIN);
}

void gpio_setup(void) {
    /* Enable GPIO clocks. */
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOF);

    button_setup();

    /* Reset target pin */
    gpio_set_output_options(TARGET_RESET_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TARGET_RESET_PIN);
    gpio_mode_setup(TARGET_RESET_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_RESET_PIN);

    /* Boot from flash target pin */
    gpio_set_output_options(TARGET_BOOT_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TARGET_BOOT_PIN);
    gpio_mode_setup(TARGET_BOOT_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_BOOT_PIN);

    /* Target interface enable pin */
    gpio_set_output_options(TARGET_IFACE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, TARGET_IFACE_PIN);
    gpio_mode_setup(TARGET_IFACE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, TARGET_IFACE_PIN);
    /* Enable interface by default */
    gpio_set(TARGET_IFACE_PORT, TARGET_IFACE_PIN);
    is_interface_connected = true;

    /* DUT power relays control */
    gpio_set_output_options(PWR_EXTX1_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PWR_EXTX1_EN_PIN);
    gpio_mode_setup(PWR_EXTX1_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_EXTX1_EN_PIN);
	gpio_set_output_options(PWR_DCDC_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PWR_DCDC_EN_PIN);
    gpio_mode_setup(PWR_DCDC_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_DCDC_EN_PIN);
	gpio_set_output_options(PWR_USBC_EN_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, PWR_USBC_EN_PIN);
    gpio_mode_setup(PWR_USBC_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PWR_USBC_EN_PIN);
    
    /* Disable output power */
    gpio_clear(PWR_EXTX1_EN_PORT, PWR_EXTX1_EN_PIN);
	gpio_clear(PWR_DCDC_EN_PORT,  PWR_DCDC_EN_PIN);
	gpio_clear(PWR_USBC_EN_PORT,  PWR_USBC_EN_PIN);
    
    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_SYSCFGCOMPEN);
    
    /* Current ranges select */
    gpio_set_output_options(CURRENT_RANGE0_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE0_PIN);
    gpio_mode_setup(CURRENT_RANGE0_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE0_PIN);
    gpio_set_output_options(CURRENT_RANGE1_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE1_PIN);
    gpio_mode_setup(CURRENT_RANGE1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE1_PIN);
    gpio_set_output_options(CURRENT_RANGE2_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE2_PIN);
    gpio_mode_setup(CURRENT_RANGE2_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE2_PIN);
	gpio_set_output_options(CURRENT_RANGE3_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, CURRENT_RANGE3_PIN);
    gpio_mode_setup(CURRENT_RANGE3_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CURRENT_RANGE3_PIN);
    
    /* Disable all shunts */
    gpio_clear(CURRENT_RANGE0_PORT, CURRENT_RANGE0_PIN);
    gpio_clear(CURRENT_RANGE1_PORT, CURRENT_RANGE1_PIN);
    gpio_clear(CURRENT_RANGE2_PORT, CURRENT_RANGE2_PIN);
	gpio_clear(CURRENT_RANGE3_PORT, CURRENT_RANGE3_PIN);
    current_power_range = 0;

    /* Power on in 100 ms by TIM3 */
    /* button1_counter = 100; */

    /* Setup timers */
    tim2_setup();
    
    memcpy((void *)&emb_settings, (void *)FLASH_CONFIG_ADDR, sizeof(emb_settings));
    if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
        emb_settings.voltage_coeff = 0;
        emb_settings.period = 100;
        emb_settings.baudrate = DEFAULT_BAUDRATE;
        emb_settings.dap_active = 1;
        emb_settings.start_c = 0;
        emb_settings.q = 0;
        emb_settings.r = 0;
        emb_settings.e0 = 0;
        emb_settings.k1 = 0;
        emb_settings.k2 = 0;
        emb_settings.a = 0;
        emb_settings.b = 0;
    }
    
    if ((emb_settings.period < 10) || (emb_settings.period >= 1000)) {
        emb_settings.period = 100;
    }
    
    if ((emb_settings.baudrate == 0) || (emb_settings.baudrate > 1000000)) {
        emb_settings.baudrate = DEFAULT_BAUDRATE;
    }

    battery_state.c_acc_01ua_us = 0;
    battery_state.current_c = emb_settings.start_c;
    battery_state.voltage_slow_part = 0;
    battery_state.voltage_fast_part = 0;
    
    console_reconfigure(emb_settings.baudrate, 8, USART_STOPBITS_1, USART_PARITY_NONE);
    
    int len = strlen(FW_VERSION);
    if (emb_settings.magic != FLASH_CONFIG_MAGIC) {
        // tic33m_display_string(&tic33m_dev, "ERR " FW_VERSION, 5 + len);
    } else {
        // tic33m_display_string(&tic33m_dev, "CAL " FW_VERSION, 5 + len);
    }

    /* enable green LED */
    if (!board_v2) {
        gpio_clear(LED_CON_GPIO_PORT, LED_CON_GPIO_PIN);
    } else {
        gpio_clear(LED_RANGE0_GPIO_PORT, LED_RANGE0_GPIO_PIN);
    }
}

void target_console_init(void) {
    /* Enable UART clock */
    rcc_periph_clock_enable(CONSOLE_USART_CLOCK);

    /* Setup GPIO pins for UART2 */
    gpio_mode_setup(CONSOLE_USART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, CONSOLE_USART_GPIO_PINS);
    gpio_set_af(CONSOLE_USART_GPIO_PORT, CONSOLE_USART_GPIO_AF, CONSOLE_USART_GPIO_PINS);
}

/* No LEDs on this board */
void led_bit(uint8_t position, bool state) {
	(void) position;
	(void) state;
}

void led_num(uint8_t value) {
	(void) value;
}
