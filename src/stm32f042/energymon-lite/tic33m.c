/*
 * Copyright (c) 2019, Unwired Devices LLC  <info@unwds.com>
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


#include <libopencm3/stm32/gpio.h>
#include <string.h>
#include "tic33m.h"

typedef struct {
    char symbol;
    uint8_t code;
} ascii_symbols_t;

/* TIC33M symbol
 *    128
 *  64    2
 *     4
 *  32    8
 *     16    1
*/

static const ascii_symbols_t tic33m_digits[]  = {
    { ' ', 0 },
    { '-', 4 },
    { '_', 2 },
    { '`', 128 },
    { '0', 2+8+16+32+64+128 },
    { '1', 2+8 },
    { '2', 128+2+4+32+16 },
    { '3', 2+8+128+4+16 },
    { '4', 64+4+2+8 },
    { '5', 128+64+4+8+16 },
    { '6', 128+64+4+8+16+32 },
    { '7', 128+2+8 },
    { '8', 2+4+8+16+32+64+128 },
    { '9', 2+4+8+16+64+128 },
    { 'A', 2+4+8+32+64+128 },
    { 'b', 4+8+16+32+64 },
    { 'C', 16+32+64+128 },
    { 'c', 4+16+32 },
    { 'd', 2+4+8+16+32 },
    { 'E', 4+16+32+64+128 },
    { 'F', 4+32+64+128 },
    { 'G', 8+16+32+64+128 },
    { 'h', 4+8+32+64 },
    { 'I', 32+64 },
    { 'i', 8 },
    { 'J', 2+8+16 },
    /* K is impossible */
    { 'L', 16+32+64 },
    /* M is impossible */
    { 'n', 4+8+32 },
    { 'o', 4+8+16+32 },
    { 'P', 2+4+32+64+128 },
    /* Q is impossible */
    { 'r', 4+32 },
    { 'S', 4+8+16+64+128 },
    { 't', 4+16+32+64 },
    { 'U', 2+8+16+32+64 },
    { 'u', 8+16+32 },
    /* V is impossible */
    /* W is impossible */
    /* X is impossible */
    { 'Y', 2+4+8+16+64 },
    /* Z is impossible */
};

#define TIC33M_NUM_DIGITS   sizeof(tic33m_digits)/sizeof(ascii_symbols_t)

static inline void tic33m_delay(void) {
    /* 0.4 us min strobe length */
    /* 20 cycles on 48 MHz CPU */
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
    __asm("nop; nop; nop; nop; nop;");
}

static int tic33m_find_code(char symbol) {
    for (unsigned i = 0; i < TIC33M_NUM_DIGITS; i++) {
        if (tic33m_digits[i].symbol == symbol) {
            return tic33m_digits[i].code;
        }
    }
    
    /* 'space' if nothing found */
    return tic33m_digits[0].code;
}

static void tic33m_putchar(tic33m *dev, char symbol, bool point) {
    uint8_t code = tic33m_find_code(symbol);
    
    if (point) {
        code |= 1;
    }
    
    for (int i = 8; i; i--) {
        if (code & 128) {
            gpio_set(dev->din_port, dev->din_pin);
        } else {
            gpio_clear(dev->din_port, dev->din_pin);
        }
        gpio_set(dev->clk_port, dev->clk_pin);
        tic33m_delay();
        gpio_clear(dev->clk_port, dev->clk_pin);
        code <<= 1;
    }
}

int tic33m_display_string(tic33m *dev, char *str, int strlen) {
    /* 9 symbols on TIC33M */
    int strindex = 0;
    int lcdindex = 0;
    do {
        bool point = false;
        
        if (strindex < strlen) {
            if ((strindex < strlen - 1) && (str[strindex + 1] == '.')) {
                point = true;
            }
            
            if (str[strindex] != '.') {
                lcdindex++;
                tic33m_putchar(dev, str[strindex], point);
            }
            strindex++;
        } else {
            tic33m_putchar(dev, ' ', false);
            lcdindex++;
        }

    } while (lcdindex < 9);
    
    tic33m_delay();
    gpio_set(dev->load_port, dev->load_pin);
    tic33m_delay();
    gpio_clear(dev->load_port, dev->load_pin);
    
    return 0;
}

int tic33m_display_time(tic33m *dev, uint32_t seconds) {
    /* 9 digits on TIC33M */
    char digits[9];
    memset(digits, ' ', 8);
    
    uint16_t hours = seconds/3600;
    seconds = seconds % 3600;
    uint8_t  minutes = seconds / 60;
    seconds -= minutes*60;

    /* convert to ASCII symbols */
    digits[8] = '0' + seconds % 10;
    digits[7] = '0' + seconds / 10;
    digits[6] = '-';
    
    digits[5] = '0' + minutes%10;
    digits[4] = '0' + minutes/10;
    digits[3] = '-';
    
    if (hours < 10) {
        digits[2] = '0' + hours;
    } else if (hours < 100) {
        digits[2] = '0' + hours % 10;
        digits[1] = '0' + hours / 10;
    } else {
        digits[2] = '0' + hours % 10;
        digits[1] = '0' + (hours / 10) % 10;
        digits[1] = '0' + hours / 100;
    }
    
    for (int i = 0; i < 9; i++) {
        tic33m_putchar(dev, digits[i], false);
    }
    
    gpio_set(dev->load_port, dev->load_pin);
    tic33m_delay();
    gpio_clear(dev->load_port, dev->load_pin);
    
    return 0;
}

void tic33m_lclk(tic33m *dev) {
    gpio_toggle(dev->lclk_port, dev->lclk_pin);
}

void tic33m_init(tic33m *dev) {
    gpio_set_output_options(dev->load_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev->load_pin);
    gpio_mode_setup(dev->load_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev->load_pin);
    gpio_clear(dev->load_port, dev->load_pin);
    
    gpio_set_output_options(dev->din_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev->din_pin);
    gpio_mode_setup(dev->din_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev->din_pin);
    gpio_clear(dev->din_port, dev->din_pin);
    
    gpio_set_output_options(dev->clk_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev->clk_pin);
    gpio_mode_setup(dev->clk_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev->clk_pin);
    gpio_clear(dev->clk_port, dev->clk_pin);
    
    gpio_set_output_options(dev->lclk_port, GPIO_OTYPE_PP, GPIO_OSPEED_HIGH, dev->lclk_pin);
    gpio_mode_setup(dev->lclk_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, dev->lclk_pin);
    gpio_clear(dev->lclk_port, dev->clk_pin);
}