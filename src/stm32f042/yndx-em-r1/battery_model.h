/*
 * Copyright (c) 2026, Oleg Artamonov <oleg@olegart.ru>
 * 
 * Copyright (c) 2026, Egor Mikerin <egor4kus@yandex.ru>
 * Copyright (c) 2026, Arkady Pavlov <bylotonix@gmail.com>
 * Copyright (c) 2026, Raul Alimbekov <raulalimbekov@gmail.com>
 * Copyright (c) 2026, Victoria Patokova <vpatokova@gmail.com>
 * Copyright (c) 2026, Iaroslav Muravev <yaroslav.muravev.work@yandex.ru>
 * Copyright (c) 2026, Igor Kim <kimigor157@gmail.com>
 * Copyright (c) 2026, Naum Novikov <naumnovikov.it@gmail.com>
 * Copyright (c) 2026, Ilya Simonov <simalin2020@gmail.com>
 * Copyright (c) 2026, Dmitry Malinitskiy <malinitckiydimitry@gmail.com>
 * Copyright (c) 2026, Ivan Vetlugin <ivanvet31@ivanvet31.ru>
 * Copyright (c) 2026, Igor Dobritsa <igordobrica60@gmail.com>
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
