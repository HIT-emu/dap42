#include "battery_model.h"

/* ---------- Преобразование Q16.16 ↔ double ---------- */
double fixed_to_double(fixed_t x) {
    return (double)x / 65536.0;
}

fixed_t double_to_fixed(double x) {
    return (fixed_t)(x * 65536.0 + (x >= 0 ? 0.5 : -0.5));
}

/* ---------- Базовые операции с фиксированной точкой ---------- */
fixed_t fixed_add(fixed_t a, fixed_t b) {
    return a + b;
}

fixed_t fixed_sub(fixed_t a, fixed_t b) {
    return a - b;
}

fixed_t fixed_mul(fixed_t a, fixed_t b) {
    int64_t prod = (int64_t)a * b;
    return (fixed_t)(prod >> FIXED_SHIFT);
}

fixed_t fixed_div(fixed_t a, fixed_t b) {
    int64_t num = (int64_t)a << FIXED_SHIFT;
    return (fixed_t)(num / b);
}

/* ---------- CORDIC: натуральный логарифм ---------- */
#define CORDIC_ITER 16
#define LN2_Q16  0x0000B172  // ln(2) ≈ 0.693147 в Q16.16

// Предвычисленные atanh(2^-i) в Q16.16 (для i = 1..15)
static const fixed_t atanh_table[CORDIC_ITER] = {
    0,
    0x00008C9F,  // atanh(0.5)        = 0.5493061
    0x00004163,  // atanh(0.25)       = 0.2554128
    0x0000202B,  // atanh(0.125)      = 0.1256572
    0x00001005,  // atanh(0.0625)     = 0.0625816
    0x00000801,  // atanh(0.03125)    = 0.0312602
    0x00000400,  // atanh(0.015625)   = 0.0156263
    0x00000200,  // atanh(0.0078125)  = 0.0078127
    0x00000100,
    0x00000080,
    0x00000040,
    0x00000020,
    0x00000010,
    0x00000008,
    0x00000004,
    0x00000002
};

fixed_t fixed_ln(fixed_t x) {
    if (x <= 0) return 0;  // ошибка

    // 1. Нормализация аргумента (x >= 0.5)
    fixed_t shift_correction = 0;
    while (x < FIXED_HALF) {
        x <<= 1;
        shift_correction = fixed_add(shift_correction, LN2_Q16);
    }

    fixed_t x_cur = fixed_add(x, FIXED_ONE);
    fixed_t y_cur = fixed_sub(x, FIXED_ONE);
    fixed_t z_cur = 0;

    for (int i = 1; i < CORDIC_ITER; i++) {
        fixed_t dx = x_cur >> i;
        fixed_t dy = y_cur >> i;

        if (y_cur < 0) {
            fixed_t x_next = fixed_add(x_cur, dy);
            y_cur = fixed_add(y_cur, dx);
            x_cur = x_next;
            z_cur = fixed_sub(z_cur, atanh_table[i]);
        } else {
            fixed_t x_next = fixed_sub(x_cur, dy);
            y_cur = fixed_sub(y_cur, dx);
            x_cur = x_next;
            z_cur = fixed_add(z_cur, atanh_table[i]);
        }

        // Повтор 4-й итерации для сходимости
        if (i == 4) {
            dx = x_cur >> 4;
            dy = y_cur >> 4;

            if (y_cur < 0) {
                fixed_t x_next = fixed_add(x_cur, dy);
                y_cur = fixed_add(y_cur, dx);
                x_cur = x_next;
                z_cur = fixed_sub(z_cur, atanh_table[4]);
            } else {
                fixed_t x_next = fixed_sub(x_cur, dy);
                y_cur = fixed_sub(y_cur, dx);
                x_cur = x_next;
                z_cur = fixed_add(z_cur, atanh_table[4]);
            }
        }
    }

    fixed_t result = z_cur << 1;

    return fixed_sub(result, shift_correction);
}

/* ---------- Быстрая экспонента (основание e) ---------- */
#define INV_LN2  0x00017154  // 1/ln(2) ≈ 1.442695 в Q16.16

// Коэффициенты полинома Тейлора для exp(r)
#define P0  FIXED_ONE
#define P1  FIXED_ONE
#define P2  0x00008000  // 1/2
#define P3  0x00002AAB  // 1/6
#define P4  0x00000AAB  // 1/24

fixed_t fixed_exp(fixed_t x) {
    if (x < -0x00100000) return 0; // exp(большое отрицательное) ~ 0

    // 1. Выделяем целую часть k = round(x / ln2) с корректным округлением
    fixed_t k_f = fixed_mul(x, INV_LN2);
    int16_t k = (int16_t)((k_f + FIXED_HALF) >> FIXED_SHIFT);

    // 2. Вычисляем остаток r = x - k * ln2
    fixed_t k_ln2 = k * LN2_Q16;
    fixed_t r = fixed_sub(x, k_ln2);

    // 3. Полиномиальное приближение (схема Горнера)
    fixed_t poly = fixed_mul(r, P4);                     // r * P4
    poly = fixed_add(P3, poly);                          // P3 + r*P4
    poly = fixed_mul(r, poly);                           // r * (P3 + r*P4)
    poly = fixed_add(P2, poly);                          // P2 + ...
    poly = fixed_mul(r, poly);
    poly = fixed_add(P1, poly);
    poly = fixed_mul(r, poly);
    poly = fixed_add(P0, poly);

    // 4. Умножаем на 2^k (сдвиг)
    if (k >= 0) {
        return poly << k;
    } else {
        return poly >> (-k);
    }
}
