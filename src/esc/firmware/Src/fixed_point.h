#pragma once

/*
 * UTILITAS FIXED-POINT UNTUK FOC STM32F103
 * =======================================
 * Hot-path FOC tetap integer. Perkalian Q14 memakai intermediate 32-bit
 * (cukup untuk int16 x int16), pembulatan simetris ke nilai terdekat, lalu
 * saturasi ke int16. Ini mengurangi bias truncation tanpa biaya float/double.
 */

#include <stdint.h>
#include <limits.h>

static inline int16_t fp_sat_s16(int32_t value)
{
  if (value > INT16_MAX) return INT16_MAX;
  if (value < INT16_MIN) return INT16_MIN;
  return (int16_t)value;
}

static inline int16_t fp_mul_q14_s16(int16_t a, int16_t b)
{
  int32_t product = (int32_t)a * (int32_t)b;
  if (product >= 0) {
    product = (product + (1 << 13)) >> 14;
  } else {
    product = -(((-product) + (1 << 13)) >> 14);
  }
  return fp_sat_s16(product);
}

static inline int16_t fp_add_sat_s16(int16_t a, int16_t b)
{
  return fp_sat_s16((int32_t)a + (int32_t)b);
}

static inline int16_t fp_sub_sat_s16(int16_t a, int16_t b)
{
  return fp_sat_s16((int32_t)a - (int32_t)b);
}
