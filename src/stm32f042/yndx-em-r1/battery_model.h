#ifndef BATTERY_MODEL_H_INCLUDED
#define BATTERY_MODEL_H_INCLUDED

/* ---------- Базовые типы Q16.16 ---------- */
#include <stdint.h>

typedef int32_t fixed_t;
#define FIXED_SHIFT 16
#define FIXED_ONE   (1 << FIXED_SHIFT)
#define FIXED_HALF  (1 << (FIXED_SHIFT - 1))

/* ---------- Преобразование Q16.16 ↔ double ---------- */
double fixed_to_double(fixed_t x);
fixed_t double_to_fixed(double x);

/* ---------- Базовые операции с фиксированной точкой ---------- */
fixed_t fixed_add(fixed_t a, fixed_t b);
fixed_t fixed_sub(fixed_t a, fixed_t b);
fixed_t fixed_mul(fixed_t a, fixed_t b);
fixed_t fixed_div(fixed_t a, fixed_t b);

/* ---------- CORDIC: натуральный логарифм ---------- */
fixed_t fixed_ln(fixed_t x);

/* ---------- Быстрая экспонента (основание e) ---------- */
fixed_t fixed_exp(fixed_t x);

#endif
