/*
 * CONTROLLER FOC MOTOR - IMPLEMENTASI C FIXED-POINT
 * ==================================================
 *
 * File ini mempertahankan persamaan, urutan operasi, saturasi, lookup table,
 * state delay, PI controller, Clarke/Park transform, dan inverse transform dari
 * controller sumber hoverboard-firmware-hack-FOC. Penamaan antarmuka telah
 * dirapikan menjadi C biasa agar alur algoritma lebih mudah dipelajari.
 *
 * PENTING: tipe integer dan urutan perhitungan sengaja tidak diubah karena
 * keduanya memengaruhi hasil numerik fixed-point controller.
 */

#include "foc_motor.h"
#include "fixed_point.h"

#define IN_ACTIVE                      ((uint8_t)1U)
#define IN_NO_ACTIVE_CHILD             ((uint8_t)0U)
#define IN_OPEN                        ((uint8_t)2U)
#define IN_SPEED_MODE                  ((uint8_t)1U)
#define IN_TORQUE_MODE                 ((uint8_t)2U)
#define IN_VOLTAGE_MODE                ((uint8_t)3U)
#define OPEN_MODE                      ((uint8_t)0U)
#define SPD_MODE                       ((uint8_t)2U)
#define TRQ_MODE                       ((uint8_t)3U)
#define VLT_MODE                       ((uint8_t)1U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

/* Tipe numerik controller memakai stdint.h sehingga lebar bit eksplisit. */

static uint8_t LookupIndexS16Even(int16_t u, int16_t bp0, uint16_t bpSpace, uint32_t
  maxIndex);
static uint8_t LookupIndexU16Even(uint16_t u, uint16_t bp0, uint16_t bpSpace, uint32_t
  maxIndex);
static int32_t DivideS32Floor(int32_t numerator, int32_t denominator);
static void CounterI16_Init(CounterI16State *state, int16_t initialCount);
static int16_t CounterI16_Update(int16_t increment, int16_t maximum, bool reset,
  CounterI16State *state);
static void LowPass2_Reset(LowPassFilterState *state);
static void LowPass2_Update(const int16_t input_u[2], uint16_t coefficient, int16_t
  output_y[2], LowPassFilterState *state);
static void CounterU16_Init(CounterU16State *state, uint16_t initialCount);
static void CounterU16_Update(uint16_t increment, uint16_t maximum, bool reset,
                      uint16_t *output_cnt, CounterU16State *state);
static void EdgeDetector_Update(bool input_u, bool *output_y, EdgeDetectorState
  *state);
static void DebounceFilter_Init(DebounceFilterState *state);
static void DebounceFilter_Update(bool input_u, uint16_t activationTime, uint16_t
  deactivationTime, bool *output_y, DebounceFilterState *state);
static void BackCalculation_Init(BackCalculationState *state, int32_t
  initialIntegrator);
static void BackCalculation_Reset(BackCalculationState *state, int32_t
  initialIntegrator);
static void BackCalculation_Update(int16_t error, uint16_t integralGain, uint16_t backCalcGain,
  int16_t saturationMax, int16_t saturationMin, int16_t *outputValue, BackCalculationState *
  state);
static void PidCurrent_Init(PidCurrentState *state);
static void PidCurrent_Reset(PidCurrentState *state);
static void PidCurrent_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain,
  int32_t initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
  externalLimitCorrection, int16_t *outputValue, PidCurrentState *state);
static void PidSpeed_Init(PidSpeedState *state);
static void PidSpeed_Reset(PidSpeedState *state);
static void PidSpeed_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain,
  int16_t initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
  externalLimitCorrection, int16_t *outputValue, PidSpeedState *state);
static void PidTorque_Init(PidTorqueState *state);
static void PidTorque_Reset(PidTorqueState *state);
static void PidTorque_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain,
  int16_t initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
  externalLimitCorrection, int16_t *outputValue, PidTorqueState *state);
/**
 * Mencari indeks lookup table berjarak seragam untuk input signed 16-bit. Operasi indeks dan saturasi sama dengan controller sumber.
 */
static uint8_t LookupIndexS16Even(int16_t u, int16_t bp0, uint16_t bpSpace, uint32_t
  maxIndex)
{
  uint8_t bpIndex;
  uint16_t fbpIndex;

  /*
   * Lookup memakai breakpoint berjarak seragam. Input di bawah batas pertama
   * dipaksa ke indeks 0 dan input di atas tabel dipotong ke indeks terakhir.
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    fbpIndex = (uint16_t)((uint32_t)(uint16_t)(u - bp0) / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_t)fbpIndex;
    } else {
      bpIndex = (uint8_t)maxIndex;
    }
  }

  return bpIndex;
}

/**
 * Mencari indeks lookup table berjarak seragam untuk input unsigned 16-bit. Dipakai oleh interpolasi fixed-point.
 */
static uint8_t LookupIndexU16Even(uint16_t u, uint16_t bp0, uint16_t bpSpace, uint32_t
  maxIndex)
{
  uint8_t bpIndex;
  uint16_t fbpIndex;

  /*
   * Lookup memakai breakpoint berjarak seragam. Input di bawah batas pertama
   * dipaksa ke indeks 0 dan input di atas tabel dipotong ke indeks terakhir.
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    fbpIndex = (uint16_t)((uint32_t)(uint16_t)((uint32_t)u - bp0) / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_t)fbpIndex;
    } else {
      bpIndex = (uint8_t)maxIndex;
    }
  }

  return bpIndex;
}

/**
 * Membagi dua integer signed 32-bit dengan pembulatan ke bawah seperti semantik controller sumber.
 */
static int32_t DivideS32Floor(int32_t numerator, int32_t denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/**
 * Menginisialisasi state counter signed 16-bit ke nilai awal parameter.
 */
static void CounterI16_Init(CounterI16State *state, int16_t initialCount)
{

  state->previousValue = initialCount;
}

/**
 * Memperbarui counter signed 16-bit dengan increment, reset dan batas maksimum yang sama seperti algoritma asli.
 */
static int16_t CounterI16_Update(int16_t increment, int16_t maximum, bool reset, CounterI16State *
                state)
{
  int16_t previousCount;
  int16_t updatedCount;

  if (reset) {
    previousCount = 0;
  } else {
    previousCount = state->previousValue;
  }

  updatedCount = (int16_t)(increment + previousCount);

  if (updatedCount < maximum) {

    state->previousValue = updatedCount;
  } else {

    state->previousValue = maximum;
  }

  return updatedCount;
}

/**
 * Mereset state low-pass filter dua kanal.
 */
static void LowPass2_Reset(LowPassFilterState *state)
{

  state->previousValue1[0] = 0;
  state->previousValue1[1] = 0;
}

/**
 * Menjalankan low-pass filter fixed-point dua kanal tanpa mengubah skala maupun urutan operasi.
 */
static void LowPass2_Update(const int16_t input_u[2], uint16_t coefficient, int16_t output_y[2],
                     LowPassFilterState *state)
{
  int32_t calc_Sum3_g;

  calc_Sum3_g = input_u[0] - (state->previousValue1[0] >> 16);
  if (calc_Sum3_g > 32767) {
    calc_Sum3_g = 32767;
  } else {
    if (calc_Sum3_g < -32768) {
      calc_Sum3_g = -32768;
    }
  }

  calc_Sum3_g = coefficient * calc_Sum3_g + state->previousValue1[0];

  output_y[0] = (int16_t)(calc_Sum3_g >> 16);

  state->previousValue1[0] = calc_Sum3_g;

  calc_Sum3_g = input_u[1] - (state->previousValue1[1] >> 16);
  if (calc_Sum3_g > 32767) {
    calc_Sum3_g = 32767;
  } else {
    if (calc_Sum3_g < -32768) {
      calc_Sum3_g = -32768;
    }
  }

  calc_Sum3_g = coefficient * calc_Sum3_g + state->previousValue1[1];

  output_y[1] = (int16_t)(calc_Sum3_g >> 16);

  state->previousValue1[1] = calc_Sum3_g;
}

/**
 * Menginisialisasi state counter unsigned 16-bit.
 */
static void CounterU16_Init(CounterU16State *state, uint16_t initialCount)
{

  state->previousValue = initialCount;
}

/**
 * Memperbarui counter unsigned 16-bit untuk timer/debounce internal controller.
 */
static void CounterU16_Update(uint16_t increment, uint16_t maximum, bool reset, uint16_t
               *output_cnt, CounterU16State *state)
{
  uint16_t previousCount;

  if (reset) {
    previousCount = 0U;
  } else {
    previousCount = state->previousValue;
  }

  *output_cnt = (uint16_t)((uint32_t)increment + previousCount);

  if (*output_cnt < maximum) {

    state->previousValue = *output_cnt;
  } else {

    state->previousValue = maximum;
  }

}

/**
 * Mendeteksi perubahan tepi sinyal boolean dengan menyimpan keadaan sampel sebelumnya.
 */
static void EdgeDetector_Update(bool input_u, bool *output_y, EdgeDetectorState *state)
{

  *output_y = (input_u != state->previousValue);

  state->previousValue = input_u;
}

/**
 * Menginisialisasi state debounce input digital.
 */
static void DebounceFilter_Init(DebounceFilterState *state)
{

  CounterU16_Init(&state->activationCounter, 0U);

  CounterU16_Init(&state->transitionCounter, 0U);

}

/**
 * Menyaring perubahan input digital menggunakan waktu aktivasi/deaktivasi yang telah ditentukan.
 */
static void DebounceFilter_Update(bool input_u, uint16_t activationTime, uint16_t deactivationTime,
                     bool *output_y, DebounceFilterState *state)
{
  uint16_t calc_Sum1_n;
  bool calc_RelationalOperator_g;

  EdgeDetector_Update(input_u, &calc_RelationalOperator_g, &state->edgeDetector);

  if (input_u && (!state->previousValue)) {

    CounterU16_Update(1U, activationTime, calc_RelationalOperator_g, &calc_Sum1_n,
              &state->activationCounter);

    *output_y = ((calc_Sum1_n > activationTime) || state->previousValue);

  } else if ((!input_u) && state->previousValue) {

    CounterU16_Update(1U, deactivationTime, calc_RelationalOperator_g, &calc_Sum1_n,
              &state->transitionCounter);

    *output_y = ((!(calc_Sum1_n > deactivationTime)) && state->previousValue);

  } else {

    *output_y = state->previousValue;

  }

  state->previousValue = *output_y;
}

/**
 * Menginisialisasi integrator back-calculation untuk anti-windup PI.
 */
static void BackCalculation_Init(BackCalculationState *state, int32_t initialIntegrator)
{

  state->integratorAccumulator = initialIntegrator;
}

/**
 * Mereset integrator back-calculation ke nilai awal.
 */
static void BackCalculation_Reset(BackCalculationState *state, int32_t initialIntegrator)
{

  state->previousValue = 0;

  state->integratorAccumulator = initialIntegrator;
}

/**
 * Menghitung integrator fixed-point dengan jalur back-calculation anti-windup; persamaan dan saturasi dipertahankan.
 */
static void BackCalculation_Update(int16_t error, uint16_t integralGain, uint16_t backCalcGain, int16_t
                      saturationMax, int16_t saturationMin, int16_t *outputValue,
                      BackCalculationState *state)
{
  int32_t calc_Sum1_o;
  int16_t calc_DataTypeConversion1_gf;

  calc_Sum1_o = (error * integralGain) >> 4;
  if ((calc_Sum1_o < 0) && (state->previousValue < INT32_MIN - calc_Sum1_o))
  {
    calc_Sum1_o = INT32_MIN;
  } else if ((calc_Sum1_o > 0) && (state->previousValue > INT32_MAX
              - calc_Sum1_o)) {
    calc_Sum1_o = INT32_MAX;
  } else {
    calc_Sum1_o += state->previousValue;
  }

  calc_Sum1_o += state->integratorAccumulator;

  calc_DataTypeConversion1_gf = (int16_t)(calc_Sum1_o >> 12);

  if (calc_DataTypeConversion1_gf > saturationMax) {
    *outputValue = saturationMax;
  } else if (calc_DataTypeConversion1_gf < saturationMin) {

    *outputValue = saturationMin;
  } else {
    *outputValue = calc_DataTypeConversion1_gf;
  }

  state->previousValue = (int16_t)(*outputValue - calc_DataTypeConversion1_gf) *
    backCalcGain;

  state->integratorAccumulator = calc_Sum1_o;
}

/**
 * Menginisialisasi state PID loop arus d-axis FOC.
 */
static void PidCurrent_Init(PidCurrentState *state)
{

  state->loadInitialState = 1U;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;
}

/**
 * Mereset state PID loop arus d-axis FOC.
 */
static void PidCurrent_Reset(PidCurrentState *state)
{

  state->previousValue1 = false;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;

  state->loadInitialState = 1U;
}

/**
 * Menghitung PID arus d-axis dengan jalur integral dan anti-windup asli; Kd=0 identik dengan PI sumber.
 */
static void PidCurrent_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain, int32_t
                    initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
                    externalLimitCorrection, int16_t *outputValue, PidCurrentState
                    *state)
{
  bool calc_LowerRelop1_c0;
  bool calc_UpperRelop_f;
  int32_t calc_Sum1_p0;
  int32_t q0;
  int32_t tmp;
  int16_t tmp_0;

  q0 = error * integralGain;
  if ((q0 < 0) && (externalLimitCorrection < INT32_MIN - q0)) {
    q0 = INT32_MIN;
  } else if ((q0 > 0) && (externalLimitCorrection > INT32_MAX - q0)) {
    q0 = INT32_MAX;
  } else {
    q0 += externalLimitCorrection;
  }

  if (state->loadInitialState != 0) {
    state->integratorState = initialValue;
  }

  if (state->previousValue1) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  calc_Sum1_p0 = tmp + state->integratorState;

  tmp = (error * proportionalGain) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else if (tmp < -32768) {
    tmp = -32768;
  }
  state->lastP = (int16_t)tmp;

  /* Integral tetap memakai mekanisme clamp/anti-windup lama. */
  state->lastI = (int16_t)(calc_Sum1_p0 >> 16);

  /* Komponen D ditambahkan setelah jalur PI lama. Kd=0 membuat hasil identik dengan PI sebelumnya. */
  {
    int64_t derivative64 = ((int64_t)error - (int64_t)state->previousError) * (int64_t)derivativeGain;
    derivative64 >>= 11;
    if (derivative64 > 32767) derivative64 = 32767;
    if (derivative64 < -32768) derivative64 = -32768;
    state->lastD = (int16_t)derivative64;
  }
  state->previousError = error;

  tmp = (((calc_Sum1_p0 >> 16) << 1) + tmp) >> 1;
  tmp += state->lastD;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  calc_LowerRelop1_c0 = ((int16_t)tmp > saturationMax);

  calc_UpperRelop_f = ((int16_t)tmp < saturationMin);

  if (calc_LowerRelop1_c0) {
    *outputValue = saturationMax;
  } else if (calc_UpperRelop_f) {

    *outputValue = saturationMin;
  } else {
    *outputValue = (int16_t)tmp;
  }
  state->lastOutput = *outputValue;

  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  if ((int16_t)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_t)((int16_t)tmp > 0);
  }

  state->previousValue1 = ((q0 == tmp_0) && (calc_LowerRelop1_c0 ||
    calc_UpperRelop_f));

  state->loadInitialState = 0U;
  state->integratorState = calc_Sum1_p0;
}

/**
 * Menginisialisasi state PID kecepatan.
 */
static void PidSpeed_Init(PidSpeedState *state)
{

  state->loadInitialState = 1U;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;
}

/**
 * Mereset state PID kecepatan.
 */
static void PidSpeed_Reset(PidSpeedState *state)
{

  state->previousValue1 = false;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;

  state->loadInitialState = 1U;
}

/**
 * Menghitung PID kecepatan fixed-point beserta clamp dan anti-windup.
 */
static void PidSpeed_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain, int16_t
                      initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
                      externalLimitCorrection, int16_t *outputValue, PidSpeedState
                      *state)
{
  bool calc_LowerRelop1_l;
  bool calc_UpperRelop_l;
  int32_t calc_Sum1_ni;
  int32_t q0;
  int32_t tmp;
  int16_t tmp_0;

  q0 = error * integralGain;
  if ((q0 < 0) && (externalLimitCorrection < INT32_MIN - q0)) {
    q0 = INT32_MIN;
  } else if ((q0 > 0) && (externalLimitCorrection > INT32_MAX - q0)) {
    q0 = INT32_MAX;
  } else {
    q0 += externalLimitCorrection;
  }

  if (state->loadInitialState != 0) {
    state->integratorState = initialValue << 16;
  }

  if (state->previousValue1) {
    tmp = 0;
  } else {
    tmp = q0;
  }

  calc_Sum1_ni = tmp + state->integratorState;

  tmp = (error * proportionalGain) >> 11;
  if (tmp > 32767) {
    tmp = 32767;
  } else if (tmp < -32768) {
    tmp = -32768;
  }
  state->lastP = (int16_t)tmp;
  state->lastI = (int16_t)(calc_Sum1_ni >> 16);
  {
    int64_t derivative64 = ((int64_t)error - (int64_t)state->previousError) * (int64_t)derivativeGain;
    derivative64 >>= 11;
    if (derivative64 > 32767) derivative64 = 32767;
    if (derivative64 < -32768) derivative64 = -32768;
    state->lastD = (int16_t)derivative64;
  }
  state->previousError = error;

  tmp = (((calc_Sum1_ni >> 16) << 1) + tmp) >> 1;
  tmp += state->lastD;
  if (tmp > 32767) {
    tmp = 32767;
  } else {
    if (tmp < -32768) {
      tmp = -32768;
    }
  }

  calc_LowerRelop1_l = ((int16_t)tmp > saturationMax);

  calc_UpperRelop_l = ((int16_t)tmp < saturationMin);

  if (calc_LowerRelop1_l) {
    *outputValue = saturationMax;
  } else if (calc_UpperRelop_l) {

    *outputValue = saturationMin;
  } else {
    *outputValue = (int16_t)tmp;
  }
  state->lastOutput = *outputValue;

  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  if ((int16_t)tmp < 0) {
    tmp_0 = -1;
  } else {
    tmp_0 = (int16_t)((int16_t)tmp > 0);
  }

  state->previousValue1 = ((q0 == tmp_0) && (calc_LowerRelop1_l ||
    calc_UpperRelop_l));

  state->loadInitialState = 0U;
  state->integratorState = calc_Sum1_ni;
}

/**
 * Menginisialisasi state PID Iq/torsi.
 */
static void PidTorque_Init(PidTorqueState *state)
{

  state->loadInitialState = 1U;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;
}

/**
 * Mereset state PID Iq/torsi.
 */
static void PidTorque_Reset(PidTorqueState *state)
{

  state->previousValue1 = false;
  state->previousError = 0;
  state->lastP = state->lastI = state->lastD = state->lastOutput = 0;

  state->loadInitialState = 1U;
}

/**
 * Menghitung PID Iq/torsi fixed-point beserta clamp dan anti-windup.
 */
static void PidTorque_Update(int16_t error, uint16_t proportionalGain, uint16_t integralGain, uint16_t derivativeGain, int16_t
                      initialValue, int16_t saturationMax, int16_t saturationMin, int32_t
                      externalLimitCorrection, int16_t *outputValue, PidTorqueState
                      *state)
{
  bool calc_LowerRelop1_i3;
  bool calc_UpperRelop_i;
  int16_t calc_Sum1_bm;
  int16_t tmp;
  int32_t tmp_0;
  int32_t q0;

  q0 = error * integralGain;
  if ((q0 < 0) && (externalLimitCorrection < INT32_MIN - q0)) {
    q0 = INT32_MIN;
  } else if ((q0 > 0) && (externalLimitCorrection > INT32_MAX - q0)) {
    q0 = INT32_MAX;
  } else {
    q0 += externalLimitCorrection;
  }

  if (state->loadInitialState != 0) {
    state->integratorState = initialValue;
  }

  if (state->previousValue1) {
    tmp = 0;
  } else {
    tmp = (int16_t)(((q0 < 0 ? 65535 : 0) + q0) >> 16);
  }

  calc_Sum1_bm = (int16_t)(tmp + state->integratorState);

  tmp_0 = (error * proportionalGain) >> 11;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else if (tmp_0 < -32768) {
    tmp_0 = -32768;
  }
  state->lastP = (int16_t)tmp_0;
  state->lastI = calc_Sum1_bm;
  {
    int64_t derivative64 = ((int64_t)error - (int64_t)state->previousError) * (int64_t)derivativeGain;
    derivative64 >>= 11;
    if (derivative64 > 32767) derivative64 = 32767;
    if (derivative64 < -32768) derivative64 = -32768;
    state->lastD = (int16_t)derivative64;
  }
  state->previousError = error;

  tmp_0 = ((calc_Sum1_bm << 1) + tmp_0) >> 1;
  tmp_0 += state->lastD;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else {
    if (tmp_0 < -32768) {
      tmp_0 = -32768;
    }
  }

  calc_LowerRelop1_i3 = ((int16_t)tmp_0 > saturationMax);

  calc_UpperRelop_i = ((int16_t)tmp_0 < saturationMin);

  if (calc_LowerRelop1_i3) {
    *outputValue = saturationMax;
  } else if (calc_UpperRelop_i) {

    *outputValue = saturationMin;
  } else {
    *outputValue = (int16_t)tmp_0;
  }
  state->lastOutput = *outputValue;

  if (q0 < 0) {
    q0 = -1;
  } else {
    q0 = (q0 > 0);
  }

  if ((int16_t)tmp_0 < 0) {
    tmp = -1;
  } else {
    tmp = (int16_t)((int16_t)tmp_0 > 0);
  }

  state->previousValue1 = ((q0 == tmp) && (calc_LowerRelop1_i3 ||
    calc_UpperRelop_i));

  state->loadInitialState = 0U;
  state->integratorState = calc_Sum1_bm;
}

/* ====================== LANGKAH UTAMA FOC ====================== */
/**
 * Satu langkah lengkap algoritma kontrol motor. Fungsi ini menjalankan estimasi posisi Hall, Clarke/Park, loop PI, pembatas, inverse transform, modulasi dan diagnostik sesuai urutan asli.
 */
void MotorController_Update(MotorController *const controller)
{
  MotorParameters *params = ((MotorParameters *) controller->parameters);
  MotorControlState *state = ((MotorControlState *) controller->state);
  MotorInput *input = (MotorInput *) controller->input;
  MotorOutput *output = (MotorOutput *) controller->output;
  bool calc_LogicalOperator;
  int8_t calc_Sum2_h;
  bool calc_RelationalOperator4_d;
  bool controlDelayPrevious;
  uint8_t calc_a_elecAngle_XA_g;
  bool calc_LogicalOperator1_j;
  bool calc_LogicalOperator2_p;
  bool calc_RelationalOperator1_mv;
  int16_t calc_Switch1_l;
  int16_t calc_Saturation;
  int16_t calc_Saturation1;
  int32_t calc_Sum1_jt;
  int16_t calc_Merge_m;
  int16_t calc_Merge1;
  int16_t calc_TmpSignalConversionAtLow_Pa[2];
  int32_t calc_Switch1;
  int32_t calc_Sum1;
  int32_t calc_Gain3;
  uint8_t Sum;
  int16_t Switch2;
  int16_t Abs5;
  int16_t DataTypeConversion2;
  int16_t tmp[4];
  int8_t selectedBranch;

  Sum = (uint8_t)((uint32_t)(uint8_t)((uint32_t)(uint8_t)(input->hallA << 2) +
    (uint8_t)(input->hallB << 1)) + input->hallC);

  calc_LogicalOperator = (bool)((input->hallA != 0) ^ (input->hallB != 0) ^
    (input->hallC != 0) ^ (state->previousHallA != 0) ^
    (state->previousHallB != 0)) ^ (state->previousHallC != 0);

  if (calc_LogicalOperator) {

    selectedBranch = state->rotationDirection;

    calc_Sum2_h = (int8_t)(motorLookupTables.hallToPosition[Sum] -
                          state->previousHallPosition);

    if ((calc_Sum2_h == 1) || (calc_Sum2_h == -5)) {
      state->rotationDirection = 1;
    } else {
      state->rotationDirection = -1;
    }

    state->previousHallPosition = motorLookupTables.hallToPosition[Sum];

    state->previousRawCounter = state->speedPeriodDelayCurrent;

    Switch2 = (int16_t)(state->previousRawCounter - state->previousRawCounter2);

    if (Switch2 < 0) {
      calc_Switch1_l = (int16_t)-Switch2;
    } else {
      calc_Switch1_l = Switch2;
    }

    if (calc_Switch1_l >= params->transitionDetectHigh) {
      state->transitionDetectedState = true;
    } else {
      if (calc_Switch1_l <= params->transitionDetectLow) {
        state->transitionDetectedState = false;
      }
    }

    state->transitionDetected = state->transitionDetectedState;

    calc_RelationalOperator4_d = (state->rotationDirection != selectedBranch);

    if (calc_RelationalOperator4_d && state->previousDirectionChange) {
      calc_Switch1_l = 0;
    } else if (calc_RelationalOperator4_d) {

      calc_Switch1_l = state->previousAbsSpeed;
    } else if (state->transitionDetected) {

      calc_Switch1_l = (int16_t)((params->speedCoefficient << 4) /
        state->previousRawCounter);
    } else {

      calc_Switch1_l = (int16_t)(((uint16_t)(params->speedCoefficient << 2) << 4) /
        (int16_t)(((state->speedPeriodDelay1 + state->speedPeriodDelay2) +
                   state->speedPeriodDelay3) + state->previousRawCounter));
    }

    state->signedSpeedEstimate = (int16_t)(calc_Switch1_l * state->rotationDirection);

    state->previousRawCounter2 = state->previousRawCounter;

    state->speedPeriodDelay1 = state->speedPeriodDelay2;

    state->speedPeriodDelay2 = state->speedPeriodDelay3;

    state->speedPeriodDelay3 = state->previousRawCounter;

    state->previousDirectionChange = calc_RelationalOperator4_d;

  }

  calc_Switch1_l = (int16_t) CounterI16_Update(1, params->counterResetMax, calc_LogicalOperator,
    &state->transitionCounter);

  if (calc_Switch1_l > params->counterResetMax) {
    Switch2 = 0;
  } else {
    Switch2 = state->signedSpeedEstimate;
  }

  if (Switch2 < 0) {
    Abs5 = (int16_t)-Switch2;
  } else {
    Abs5 = Switch2;
  }

  if (Abs5 >= params->commutationDeactivateSpeedHigh) {
    state->commutationAtSpeed = true;
  } else {
    if (Abs5 <= params->commutationActivateSpeedLow) {
      state->commutationAtSpeed = false;
    }
  }

  calc_LogicalOperator = (params->useMeasuredAngle || (state->commutationAtSpeed &&
    (!state->transitionDetected)));

  calc_RelationalOperator4_d = state->controlDelayA;

  controlDelayPrevious = state->controlDelayB;

  DataTypeConversion2 = (int16_t)(input->targetInput << 4);

  calc_Gain3 = input->phaseCurrentAB << 4;
  if (calc_Gain3 >= 27200) {
    calc_Saturation = 27200;
  } else if (calc_Gain3 <= -27200) {
    calc_Saturation = -27200;
  } else {
    calc_Saturation = (int16_t)(input->phaseCurrentAB << 4);
  }

  calc_Gain3 = input->phaseCurrentBC << 4;
  if (calc_Gain3 >= 27200) {
    calc_Saturation1 = 27200;
  } else if (calc_Gain3 <= -27200) {
    calc_Saturation1 = -27200;
  } else {
    calc_Saturation1 = (int16_t)(input->phaseCurrentBC << 4);
  }

  if (!params->useMeasuredAngle) {

    if (calc_LogicalOperator) {

      calc_Merge_m = calc_Switch1_l;
      if (!(calc_Merge_m < state->previousRawCounter)) {
        calc_Merge_m = state->previousRawCounter;
      }

      if (state->rotationDirection == 1) {
        calc_Sum2_h = motorLookupTables.hallToPosition[Sum];
      } else {
        calc_Sum2_h = (int8_t)(motorLookupTables.hallToPosition[Sum] + 1);
      }

      calc_Merge_m = (int16_t)(((int16_t)((int16_t)((calc_Merge_m << 14) /
        state->previousRawCounter) * state->rotationDirection) + (calc_Sum2_h << 14)) >> 2);
    } else {
      if (state->rotationDirection == 1) {

        calc_Sum2_h = motorLookupTables.hallToPosition[Sum];
      } else {

        calc_Sum2_h = (int8_t)(motorLookupTables.hallToPosition[Sum] + 1);
      }

      calc_Merge_m = (int16_t)(calc_Sum2_h << 12);
    }

    if (!(calc_Merge_m > 0)) {
      calc_Merge_m = 0;
    }

    calc_Merge_m = (int16_t)((15 * calc_Merge_m) >> 4);

  } else {

    calc_Sum1_jt = input->mechanicalAngle * params->polePairs - 480;

    calc_Merge_m = (int16_t)((int16_t)(calc_Sum1_jt - ((int16_t)((int16_t)
      DivideS32Floor(calc_Sum1_jt, 5760) * 360) << 4)) << 2);

  }

  calc_Sum2_h = state->activeBranch1;
  selectedBranch = -1;
  if (params->controlType == 2) {
    selectedBranch = 0;
  }

  state->activeBranch1 = selectedBranch;
  if ((calc_Sum2_h != selectedBranch) && (calc_Sum2_h == 0)) {

    if (state->activeBranch7 == 0) {

      state->filteredCurrentDQ[0] = 0;

      state->absCurrentD = 0;

      state->filteredCurrentDQ[1] = 0;
    }

    state->activeBranch7 = -1;

    state->sinElectrical = 0;

    state->cosElectrical = 0;

    state->filteredCurrentDQ[0] = 0;

    state->filteredCurrentDQ[1] = 0;

    state->absCurrentD = 0;
  }

  if (selectedBranch == 0) {

    if (params->phaseCurrentMeasurementMode == 0) {

      calc_Gain3 = 18919 * calc_Saturation;

      calc_Sum1_jt = 18919 * calc_Saturation1;

      calc_Gain3 = (((calc_Gain3 < 0 ? 32767 : 0) + calc_Gain3) >> 15) + (int16_t)
        (((calc_Sum1_jt < 0 ? 16383 : 0) + calc_Sum1_jt) >> 14);
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_Merge1 = (int16_t)calc_Gain3;

    } else if (params->phaseCurrentMeasurementMode == 1) {

      calc_Gain3 = calc_Saturation - calc_Saturation1;
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_Gain3 *= 18919;
      calc_Merge1 = (int16_t)(((calc_Gain3 < 0 ? 32767 : 0) + calc_Gain3) >> 15);

      calc_Gain3 = -calc_Saturation - calc_Saturation1;
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_Saturation = (int16_t)calc_Gain3;

    } else {

      calc_Gain3 = 18919 * calc_Saturation;

      calc_Sum1_jt = 18919 * calc_Saturation1;

      calc_Gain3 = -(((calc_Gain3 < 0 ? 32767 : 0) + calc_Gain3) >> 15) - (int16_t)
        (((calc_Sum1_jt < 0 ? 16383 : 0) + calc_Sum1_jt) >> 14);
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_Merge1 = (int16_t)calc_Gain3;

    }

    calc_a_elecAngle_XA_g = LookupIndexS16Even(calc_Merge_m, 0, 128U, 180U);

    state->sinElectrical = motorLookupTables.sinTable[calc_a_elecAngle_XA_g];

    state->cosElectrical = motorLookupTables.cosTable[calc_a_elecAngle_XA_g];

    calc_Sum2_h = state->activeBranch7;
    selectedBranch = -1;
    if (input->motorEnable) {
      selectedBranch = 0;
    }

    state->activeBranch7 = selectedBranch;
    if ((calc_Sum2_h != selectedBranch) && (calc_Sum2_h == 0)) {

      state->filteredCurrentDQ[0] = 0;

      state->absCurrentD = 0;

      state->filteredCurrentDQ[1] = 0;
    }

    if (selectedBranch == 0) {
      if (0 != calc_Sum2_h) {

        LowPass2_Reset(&state->currentDqLowPass);

      }

      calc_Gain3 = fp_sub_sat_s16(
        fp_mul_q14_s16((int16_t)calc_Merge1, state->cosElectrical),
        fp_mul_q14_s16((int16_t)calc_Saturation, state->sinElectrical));
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_TmpSignalConversionAtLow_Pa[0] = (int16_t)calc_Gain3;

      calc_Gain3 = fp_add_sat_s16(
        fp_mul_q14_s16((int16_t)calc_Saturation, state->cosElectrical),
        fp_mul_q14_s16((int16_t)calc_Merge1, state->sinElectrical));
      if (calc_Gain3 > 32767) {
        calc_Gain3 = 32767;
      } else {
        if (calc_Gain3 < -32768) {
          calc_Gain3 = -32768;
        }
      }

      calc_TmpSignalConversionAtLow_Pa[1] = (int16_t)calc_Gain3;

      LowPass2_Update(calc_TmpSignalConversionAtLow_Pa, params->currentFilterCoefficient,
                      state->filteredCurrentDQ, &state->currentDqLowPass);

      if (state->filteredCurrentDQ[0] < 0) {
        state->absCurrentD = (int16_t)-state->filteredCurrentDQ[0];
      } else {
        state->absCurrentD = state->filteredCurrentDQ[0];
      }

    }

  }

  if (state->controlDelayA) {

    if (params->diagnosticsEnabled) {

      if ((state->previousErrorCode & 4) != 0) {
        calc_RelationalOperator1_mv = true;
      } else {
        if (state->previousControlTarget < 0) {

          calc_Saturation1 = (int16_t)-state->previousControlTarget;
        } else {

          calc_Saturation1 = state->previousControlTarget;
        }

        calc_RelationalOperator1_mv = (input->motorEnable && (Abs5 <
          params->standstillSpeedThreshold) && (calc_Saturation1 > params->inputTargetErrorThreshold));
      }

      calc_a_elecAngle_XA_g = (uint8_t)(((uint32_t)((Sum == 7) << 1) + (Sum == 0))
        + (calc_RelationalOperator1_mv << 2));

      DebounceFilter_Update(calc_a_elecAngle_XA_g != 0, params->errorQualifyTime,
                      params->errorDequalifyTime, &state->diagnosticFaultStable, &state->diagnosticDebounce);

      EdgeDetector_Update(state->diagnosticFaultStable, &calc_RelationalOperator1_mv,
                  &state->diagnosticErrorEdge);

      if (calc_RelationalOperator1_mv) {

        output->errorCode = calc_a_elecAngle_XA_g;
      } else {

        output->errorCode = state->previousErrorCode;
      }

      state->previousErrorCode = output->errorCode;

    }

    calc_RelationalOperator1_mv = (state->diagnosticFaultStable || (!input->motorEnable) ||
      (input->controlModeRequest == 0));

    calc_LogicalOperator1_j = ((input->controlModeRequest == 2) || params->cruiseControlEnabled);

    calc_LogicalOperator2_p = ((input->controlModeRequest == 3) && (!params->cruiseControlEnabled));

    if (state->controlModeMachineActive == 0U) {
      state->controlModeMachineActive = 1U;
      state->controlModeMachineState = IN_OPEN;
      state->activeControlMode = OPEN_MODE;
    } else if (state->controlModeMachineState == IN_ACTIVE) {
      if (calc_RelationalOperator1_mv) {
        state->activeControlSubmode = IN_NO_ACTIVE_CHILD;
        state->controlModeMachineState = IN_OPEN;
        state->activeControlMode = OPEN_MODE;
      } else {
        switch (state->activeControlSubmode) {
         case IN_SPEED_MODE:
          state->activeControlMode = SPD_MODE;
          if (!calc_LogicalOperator1_j) {
            state->activeControlSubmode = IN_NO_ACTIVE_CHILD;
            if (calc_LogicalOperator2_p) {
              state->activeControlSubmode = IN_TORQUE_MODE;
              state->activeControlMode = TRQ_MODE;
            } else {
              state->activeControlSubmode = IN_VOLTAGE_MODE;
              state->activeControlMode = VLT_MODE;
            }
          }
          break;

         case IN_TORQUE_MODE:
          state->activeControlMode = TRQ_MODE;
          if (!calc_LogicalOperator2_p) {
            state->activeControlSubmode = IN_NO_ACTIVE_CHILD;
            if (calc_LogicalOperator1_j) {
              state->activeControlSubmode = IN_SPEED_MODE;
              state->activeControlMode = SPD_MODE;
            } else {
              state->activeControlSubmode = IN_VOLTAGE_MODE;
              state->activeControlMode = VLT_MODE;
            }
          }
          break;

         default:
          state->activeControlMode = VLT_MODE;
          if (calc_LogicalOperator2_p || calc_LogicalOperator1_j) {
            state->activeControlSubmode = IN_NO_ACTIVE_CHILD;
            if (calc_LogicalOperator2_p) {
              state->activeControlSubmode = IN_TORQUE_MODE;
              state->activeControlMode = TRQ_MODE;
            } else if (calc_LogicalOperator1_j) {
              state->activeControlSubmode = IN_SPEED_MODE;
              state->activeControlMode = SPD_MODE;
            } else {
              state->activeControlSubmode = IN_VOLTAGE_MODE;
              state->activeControlMode = VLT_MODE;
            }
          }
          break;
        }
      }
    } else {
      state->activeControlMode = OPEN_MODE;
      if ((!calc_RelationalOperator1_mv) && ((input->controlModeRequest == 1) ||
           calc_LogicalOperator1_j || calc_LogicalOperator2_p)) {
        state->controlModeMachineState = IN_ACTIVE;
        if (calc_LogicalOperator2_p) {
          state->activeControlSubmode = IN_TORQUE_MODE;
          state->activeControlMode = TRQ_MODE;
        } else if (calc_LogicalOperator1_j) {
          state->activeControlSubmode = IN_SPEED_MODE;
          state->activeControlMode = SPD_MODE;
        } else {
          state->activeControlSubmode = IN_VOLTAGE_MODE;
          state->activeControlMode = VLT_MODE;
        }
      }
    }

    if (params->controlType == 2) {

      tmp[0] = 0;
      tmp[1] = params->voltageDMax;
      tmp[2] = params->maxSpeed;
      tmp[3] = params->maxCurrent;

      if (DataTypeConversion2 > 16000) {
        DataTypeConversion2 = 16000;
      } else {
        if (DataTypeConversion2 < -16000) {
          DataTypeConversion2 = -16000;
        }
      }

      calc_Saturation = (int16_t)(((uint16_t)((tmp[input->controlModeRequest] << 5) / 125)
        * DataTypeConversion2) >> 12);

    } else if (DataTypeConversion2 > 16000) {

      calc_Saturation = 16000;

    } else if (DataTypeConversion2 < -16000) {

      calc_Saturation = -16000;

    } else {

      calc_Saturation = DataTypeConversion2;

    }

    calc_Sum2_h = state->activeBranch6;
    selectedBranch = (int8_t)!(state->activeControlMode == 0);
    state->activeBranch6 = selectedBranch;
    switch (selectedBranch) {
     case 0:
      if (selectedBranch != calc_Sum2_h) {

        state->openLoopInitialize = true;

        state->previousValue = 0;

      }

      calc_Gain3 = state->previousControlTarget << 12;
      calc_Sum1_jt = (calc_Gain3 & 134217728) != 0 ? calc_Gain3 | -134217728 :
        calc_Gain3 & 134217727;

      calc_RelationalOperator1_mv = state->openLoopInitialize;

      state->openLoopInitialize = false;

      if (calc_RelationalOperator1_mv) {
        calc_Switch1 = calc_Sum1_jt;
      } else {
        calc_Switch1 = state->previousValue;
      }

      calc_Gain3 = -calc_Switch1;
      calc_Sum1 = (calc_Gain3 & 134217728) != 0 ? calc_Gain3 | -134217728 :
        calc_Gain3 & 134217727;

      if (calc_Sum1 > params->openLoopVoltageRate) {
        calc_Sum1 = params->openLoopVoltageRate;
      } else {

        calc_Gain3 = -params->openLoopVoltageRate;
        calc_Gain3 = (calc_Gain3 & 134217728) != 0 ? calc_Gain3 | -134217728 :
          calc_Gain3 & 134217727;

        if (calc_Sum1 < calc_Gain3) {
          calc_Sum1 = calc_Gain3;
        }

      }

      calc_Gain3 = calc_Sum1 + calc_Switch1;
      calc_Switch1 = (calc_Gain3 & 134217728) != 0 ? calc_Gain3 | -134217728 :
        calc_Gain3 & 134217727;

      if (calc_RelationalOperator1_mv) {

        state->previousValue = calc_Sum1_jt;
      } else {

        state->previousValue = calc_Switch1;
      }

      state->controlTarget = (int16_t)(calc_Switch1 >> 12);

      break;

     case 1:

      state->controlTarget = calc_Saturation;

      break;
    }

    if (state->controlTarget < 0) {
      state->absControlTarget = (int16_t)-state->controlTarget;
    } else {
      state->absControlTarget = state->controlTarget;
    }

  } else if (state->controlDelayB) {

    if (params->fieldWeakeningEnabled) {

      if (DataTypeConversion2 < 0) {
        DataTypeConversion2 = (int16_t)-DataTypeConversion2;
      }

      if (DataTypeConversion2 > params->fieldWeakeningHigh) {
        DataTypeConversion2 = params->fieldWeakeningHigh;
      } else {
        if (DataTypeConversion2 < params->fieldWeakeningLow) {

          DataTypeConversion2 = params->fieldWeakeningLow;
        }
      }

      if (params->controlType == 2) {
        calc_Saturation1 = params->fieldWeakeningCurrentMax;
      } else {
        calc_Saturation1 = params->phaseAdvanceMax;
      }

      if (Abs5 > params->fieldWeakeningSpeedHigh) {
        calc_Saturation = params->fieldWeakeningSpeedHigh;
      } else if (Abs5 < params->fieldWeakeningSpeedLow) {

        calc_Saturation = params->fieldWeakeningSpeedLow;
      } else {
        calc_Saturation = Abs5;
      }

      state->fieldWeakeningCommand = (int16_t)(((uint16_t)(((uint32_t)(uint16_t)(((int16_t)
        (DataTypeConversion2 - params->fieldWeakeningLow) << 15) / (int16_t)
        (params->fieldWeakeningHigh - params->fieldWeakeningLow)) * (uint16_t)(((int16_t)
        (calc_Saturation - params->fieldWeakeningSpeedLow) << 15) / (int16_t)
        (params->fieldWeakeningSpeedHigh - params->fieldWeakeningSpeedLow))) >> 15) *
        calc_Saturation1) >> 15);

    }

    calc_Sum2_h = state->activeBranch5;
    selectedBranch = -1;
    if (params->controlType == 2) {
      selectedBranch = 0;
    }

    state->activeBranch5 = selectedBranch;
    if ((calc_Sum2_h != selectedBranch) && (calc_Sum2_h == 0)) {

      state->activeSwitchCase2 = -1;
    }

    if (selectedBranch == 0) {

      state->voltageDMax = params->voltageDMax;

      state->voltageDMin = (int16_t)-state->voltageDMax;

      if (state->voltageDCommand < 0) {
        calc_Saturation1 = (int16_t)-state->voltageDCommand;
      } else {
        calc_Saturation1 = state->voltageDCommand;
      }

      state->voltageQMaxLimit = params->voltageQMaxTable[LookupIndexS16Even(calc_Saturation1,
        params->voltageQAxisTable[0], (uint16_t)(params->voltageQAxisTable[1] - params->voltageQAxisTable[0]),
        45U)];

      state->voltageQMin = (int16_t)-state->voltageQMaxLimit;
      state->maxCurrent = params->maxCurrent;

      calc_Gain3 = state->fieldWeakeningCommand << 16;
      calc_Gain3 = (calc_Gain3 == INT32_MIN) && (state->maxCurrent == -1) ?
        INT32_MAX : calc_Gain3 / state->maxCurrent;
      if (calc_Gain3 < 0) {
        calc_Gain3 = 0;
      } else {
        if (calc_Gain3 > 65535) {
          calc_Gain3 = 65535;
        }
      }

      state->currentQMax = (int16_t)
        ((motorLookupTables.qCurrentScaleTable[LookupIndexU16Even((uint16_t)calc_Gain3,
           0U, 1311U, 49U)] * state->maxCurrent) >> 16);

      state->currentQMin = (int16_t)-state->currentQMax;

      calc_Sum2_h = state->activeSwitchCase2;
      selectedBranch = -1;
      switch (state->activeControlMode) {
       case 1:
        selectedBranch = 0;
        break;

       case 2:
        selectedBranch = 1;
        break;

       case 3:
        selectedBranch = 2;
        break;
      }

      state->activeSwitchCase2 = selectedBranch;
      switch (selectedBranch) {
       case 0:
        if (selectedBranch != calc_Sum2_h) {

          BackCalculation_Reset(&state->qCurrentBackCalc, 65536000);

          BackCalculation_Reset(&state->speedBackCalc, 65536000);

        }

        BackCalculation_Update((int16_t)(state->currentQMax - state->absCurrentD),
                         params->currentQKiLimitProtection, params->backCalculationGain, state->absControlTarget, 0,
                         &state->qCurrentBackCalcLimit, &state->qCurrentBackCalc);

        BackCalculation_Update((int16_t)(params->maxSpeed - Abs5), params->speedKiLimitProtection,
                         params->backCalculationGain, state->absControlTarget, 0, &state->speedBackCalcLimit,
                         &state->speedBackCalc);

        break;

       case 1:

        if (state->filteredCurrentDQ[0] > state->currentQMax) {
          calc_Saturation1 = state->currentQMax;
        } else if (state->filteredCurrentDQ[0] < state->currentQMin) {

          calc_Saturation1 = state->currentQMin;
        } else {
          calc_Saturation1 = state->filteredCurrentDQ[0];
        }

        state->qCurrentLimitCorrection = (int16_t)(calc_Saturation1 - state->filteredCurrentDQ[0])
          * params->currentQKiLimitProtection;

        break;

       case 2:
        if (selectedBranch != calc_Sum2_h) {

          BackCalculation_Reset(&state->voltageBackCalc, 58982400);

        }

        BackCalculation_Update((int16_t)(params->maxSpeed - Abs5), params->speedKiLimitProtection,
                         params->backCalculationGain, state->voltageQMaxLimit, 0, &state->voltageBackCalcLimit,
                         &state->voltageBackCalc);

        break;
      }

      state->motorCurrentMin = (int16_t)-state->maxCurrent;

    }

  } else {
    if (state->controlDelayC) {

      calc_Sum2_h = state->activeBranch3;
      selectedBranch = -1;
      if (params->controlType == 2) {
        selectedBranch = 0;
      }

      state->activeBranch3 = selectedBranch;
      if ((calc_Sum2_h != selectedBranch) && (calc_Sum2_h == 0)) {

        state->activeSwitchCase1 = -1;

        state->activeBranch4 = -1;
      }

      if (selectedBranch == 0) {

        calc_Sum2_h = state->activeSwitchCase1;
        switch (state->activeControlMode) {
         case 1:
          break;

         case 2:
          selectedBranch = 1;
          break;

         case 3:
          selectedBranch = 2;
          break;

         default:
          selectedBranch = 3;
          break;
        }

        state->activeSwitchCase1 = selectedBranch;
        switch (selectedBranch) {
         case 0:

          if (state->absControlTarget < state->qCurrentBackCalcLimit) {
            DataTypeConversion2 = state->absControlTarget;
          } else {
            DataTypeConversion2 = state->qCurrentBackCalcLimit;
          }

          if (!(DataTypeConversion2 < state->speedBackCalcLimit)) {
            DataTypeConversion2 = state->speedBackCalcLimit;
          }

          if (state->controlTarget < 0) {
            calc_Saturation1 = -1;
          } else {
            calc_Saturation1 = (int16_t)(state->controlTarget > 0);
          }

          calc_Saturation = (int16_t)(DataTypeConversion2 * calc_Saturation1);

          if (calc_Saturation > state->voltageQMaxLimit) {

            state->voltageQCommand = state->voltageQMaxLimit;
          } else if (calc_Saturation < state->voltageQMin) {

            state->voltageQCommand = state->voltageQMin;
          } else {

            state->voltageQCommand = calc_Saturation;
          }

          break;

         case 1:
          if (selectedBranch != calc_Sum2_h) {

            PidSpeed_Reset(&state->speedPid);

          }

          calc_Saturation = (int16_t)(params->cruiseSpeedTarget << 4);

          if (params->cruiseControlEnabled && (calc_Saturation != 0)) {

            if (calc_Saturation > 0) {
              calc_TmpSignalConversionAtLow_Pa[0] = state->voltageQMaxLimit;

              if (state->controlTarget > state->voltageQMin) {
                calc_TmpSignalConversionAtLow_Pa[1] = state->controlTarget;
              } else {
                calc_TmpSignalConversionAtLow_Pa[1] = state->voltageQMin;
              }

            } else {
              if (state->voltageQMaxLimit < state->controlTarget) {

                calc_TmpSignalConversionAtLow_Pa[0] = state->voltageQMaxLimit;
              } else {
                calc_TmpSignalConversionAtLow_Pa[0] = state->controlTarget;
              }

              calc_TmpSignalConversionAtLow_Pa[1] = state->voltageQMin;
            }

          } else {
            calc_TmpSignalConversionAtLow_Pa[0] = state->voltageQMaxLimit;
            calc_TmpSignalConversionAtLow_Pa[1] = state->voltageQMin;
          }

          if (!params->cruiseControlEnabled) {
            calc_Saturation = state->controlTarget;
          }

          calc_Gain3 = calc_Saturation - Switch2;
          if (calc_Gain3 > 32767) {
            calc_Gain3 = 32767;
          } else {
            if (calc_Gain3 < -32768) {
              calc_Gain3 = -32768;
            }
          }

          PidSpeed_Update((int16_t)calc_Gain3, params->speedKp, params->speedKi, params->speedKd,
                           state->previousControlTarget,
                           calc_TmpSignalConversionAtLow_Pa[0],
                           calc_TmpSignalConversionAtLow_Pa[1], state->qCurrentLimitCorrection,
                           &state->voltageQCommand, &state->speedPid);

          break;

         case 2:
          if (selectedBranch != calc_Sum2_h) {

            PidTorque_Reset(&state->torquePid);

          }

          calc_Saturation = (int16_t)-state->voltageBackCalcLimit;

          if (state->controlTarget > state->currentQMax) {
            calc_Saturation1 = state->currentQMax;
          } else if (state->controlTarget < state->currentQMin) {

            calc_Saturation1 = state->currentQMin;
          } else {
            calc_Saturation1 = state->controlTarget;
          }

          calc_Gain3 = calc_Saturation1 - state->filteredCurrentDQ[0];
          if (calc_Gain3 > 32767) {
            calc_Gain3 = 32767;
          } else {
            if (calc_Gain3 < -32768) {
              calc_Gain3 = -32768;
            }
          }

          if (state->voltageQMaxLimit < state->voltageBackCalcLimit) {
            calc_Saturation1 = state->voltageQMaxLimit;
          } else {
            calc_Saturation1 = state->voltageBackCalcLimit;
          }

          if (!(calc_Saturation > state->voltageQMin)) {
            calc_Saturation = state->voltageQMin;
          }

          PidTorque_Update((int16_t)calc_Gain3, params->currentQKp, params->currentQKi, params->currentQKd,
                           state->previousControlTarget, calc_Saturation1,
                           calc_Saturation, 0, &state->voltageQCommand,
                           &state->torquePid);

          break;

         case 3:

          state->voltageQCommand = state->controlTarget;

          break;
        }

        calc_Sum2_h = state->activeBranch4;
        selectedBranch = -1;
        if (calc_LogicalOperator) {
          selectedBranch = 0;
        }

        state->activeBranch4 = selectedBranch;
        if (selectedBranch == 0) {
          if (0 != calc_Sum2_h) {

            PidCurrent_Reset(&state->currentPid);

          }

          calc_Saturation = (int16_t)-state->fieldWeakeningCommand;

          if (calc_Saturation > state->maxCurrent) {
            calc_Saturation = state->maxCurrent;
          } else {
            if (calc_Saturation < state->motorCurrentMin) {

              calc_Saturation = state->motorCurrentMin;
            }
          }

          calc_Gain3 = calc_Saturation - state->filteredCurrentDQ[1];
          if (calc_Gain3 > 32767) {
            calc_Gain3 = 32767;
          } else {
            if (calc_Gain3 < -32768) {
              calc_Gain3 = -32768;
            }
          }

          PidCurrent_Update((int16_t)calc_Gain3, params->currentDKp, params->currentDKi, params->currentDKd, 0,
                         state->voltageDMax, state->voltageDMin, 0, &state->voltageDCommand,
                         &state->currentPid);

        }

      }

    }
  }

  calc_Sum2_h = state->activeBranch2;
  selectedBranch = -1;
  if (params->controlType == 2) {
    calc_Saturation = state->voltageQCommand;
    selectedBranch = 0;
  } else {
    calc_Saturation = state->controlTarget;
  }

  state->activeBranch2 = selectedBranch;
  if ((calc_Sum2_h != selectedBranch) && (calc_Sum2_h == 0)) {

    state->phaseModulation[0] = 0;
    state->phaseModulation[1] = 0;
    state->phaseModulation[2] = 0;
  }

  if (selectedBranch == 0) {

    calc_Gain3 = fp_sub_sat_s16(
      fp_mul_q14_s16(state->voltageDCommand, state->cosElectrical),
      fp_mul_q14_s16(state->voltageQCommand, state->sinElectrical));
    if (calc_Gain3 > 32767) {
      calc_Gain3 = 32767;
    } else {
      if (calc_Gain3 < -32768) {
        calc_Gain3 = -32768;
      }
    }

    calc_Sum1_jt = fp_add_sat_s16(
      fp_mul_q14_s16(state->voltageDCommand, state->sinElectrical),
      fp_mul_q14_s16(state->voltageQCommand, state->cosElectrical));
    if (calc_Sum1_jt > 32767) {
      calc_Sum1_jt = 32767;
    } else {
      if (calc_Sum1_jt < -32768) {
        calc_Sum1_jt = -32768;
      }
    }

    calc_Sum1_jt *= 14189;

    calc_Sum1_jt = (((calc_Sum1_jt < 0 ? 16383 : 0) + calc_Sum1_jt) >> 14) -
      ((int16_t)(((int16_t)calc_Gain3 < 0) + (int16_t)calc_Gain3) >> 1);
    if (calc_Sum1_jt > 32767) {
      calc_Sum1_jt = 32767;
    } else {
      if (calc_Sum1_jt < -32768) {
        calc_Sum1_jt = -32768;
      }
    }

    calc_Switch1 = -(int16_t)calc_Gain3 - (int16_t)calc_Sum1_jt;
    if (calc_Switch1 > 32767) {
      calc_Switch1 = 32767;
    } else {
      if (calc_Switch1 < -32768) {
        calc_Switch1 = -32768;
      }
    }

    DataTypeConversion2 = (int16_t)calc_Gain3;
    if (!((int16_t)calc_Gain3 < (int16_t)calc_Sum1_jt)) {
      DataTypeConversion2 = (int16_t)calc_Sum1_jt;
    }

    if (!(DataTypeConversion2 < (int16_t)calc_Switch1)) {
      DataTypeConversion2 = (int16_t)calc_Switch1;
    }

    calc_Saturation1 = (int16_t)calc_Gain3;
    if (!((int16_t)calc_Gain3 > (int16_t)calc_Sum1_jt)) {
      calc_Saturation1 = (int16_t)calc_Sum1_jt;
    }

    if (!(calc_Saturation1 > (int16_t)calc_Switch1)) {
      calc_Saturation1 = (int16_t)calc_Switch1;
    }

    calc_Sum1 = DataTypeConversion2 + calc_Saturation1;
    if (calc_Sum1 > 32767) {
      calc_Sum1 = 32767;
    } else {
      if (calc_Sum1 < -32768) {
        calc_Sum1 = -32768;
      }
    }

    calc_Merge1 = (int16_t)(calc_Sum1 >> 1);

    calc_Gain3 = (int16_t)calc_Gain3 - calc_Merge1;
    if (calc_Gain3 > 32767) {
      calc_Gain3 = 32767;
    } else {
      if (calc_Gain3 < -32768) {
        calc_Gain3 = -32768;
      }
    }

    state->phaseModulation[0] = fp_mul_q14_s16(18919, (int16_t)calc_Gain3);

    calc_Gain3 = (int16_t)calc_Sum1_jt - calc_Merge1;
    if (calc_Gain3 > 32767) {
      calc_Gain3 = 32767;
    } else {
      if (calc_Gain3 < -32768) {
        calc_Gain3 = -32768;
      }
    }

    state->phaseModulation[1] = fp_mul_q14_s16(18919, (int16_t)calc_Gain3);

    calc_Gain3 = (int16_t)calc_Switch1 - calc_Merge1;
    if (calc_Gain3 > 32767) {
      calc_Gain3 = 32767;
    } else {
      if (calc_Gain3 < -32768) {
        calc_Gain3 = -32768;
      }
    }

    state->phaseModulation[2] = fp_mul_q14_s16(18919, (int16_t)calc_Gain3);

  }

  if (calc_LogicalOperator && (params->controlType == 2)) {

    DataTypeConversion2 = state->phaseModulation[0];
    calc_Saturation1 = state->phaseModulation[1];
    calc_Merge1 = state->phaseModulation[2];

  } else if (calc_LogicalOperator && (params->controlType == 1)) {

    if (params->fieldWeakeningEnabled) {

      DataTypeConversion2 = (int16_t)((int16_t)((int16_t)(state->fieldWeakeningCommand *
        state->rotationDirection) << 2) + calc_Merge_m);
      DataTypeConversion2 -= (int16_t)((int16_t)((int16_t)DivideS32Floor
        (DataTypeConversion2, 23040) * 360) << 6);
    } else {
      DataTypeConversion2 = calc_Merge_m;
    }

    Sum = LookupIndexS16Even(DataTypeConversion2, 0, 128U, 180U);

    DataTypeConversion2 = fp_mul_q14_s16((int16_t)calc_Saturation,
      motorLookupTables.phaseASineTable[Sum]);
    calc_Saturation1 = fp_mul_q14_s16((int16_t)calc_Saturation,
      motorLookupTables.phaseBSineTable[Sum]);
    calc_Merge1 = fp_mul_q14_s16((int16_t)calc_Saturation,
      motorLookupTables.phaseCSineTable[Sum]);

  } else {

    if (motorLookupTables.hallToPosition[Sum] > 5) {

      calc_Sum2_h = 5;
    } else if (motorLookupTables.hallToPosition[Sum] < 0) {

      calc_Sum2_h = 0;
    } else {

      calc_Sum2_h = motorLookupTables.hallToPosition[Sum];
    }

    calc_Sum1_jt = calc_Sum2_h * 3;

    DataTypeConversion2 = (int16_t)(calc_Saturation *
      motorLookupTables.commutationMapTable[calc_Sum1_jt]);
    calc_Saturation1 = (int16_t)(motorLookupTables.commutationMapTable[1 + calc_Sum1_jt] *
      calc_Saturation);
    calc_Merge1 = (int16_t)(motorLookupTables.commutationMapTable[2 + calc_Sum1_jt] *
      calc_Saturation);

  }

  output->dutyPhaseA = (int16_t)(DataTypeConversion2 >> 4);

  output->dutyPhaseB = (int16_t)(calc_Saturation1 >> 4);

  state->previousHallA = input->hallA;

  state->previousHallB = input->hallB;

  state->previousHallC = input->hallC;

  state->speedPeriodDelayCurrent = calc_Switch1_l;

  state->previousAbsSpeed = Abs5;

  state->controlDelayA = state->controlDelayC;

  state->controlDelayB = calc_RelationalOperator4_d;

  state->controlDelayC = controlDelayPrevious;

  state->previousControlTarget = calc_Saturation;

  output->dutyPhaseC = (int16_t)(calc_Merge1 >> 4);

  output->motorSpeed = (int16_t)(Switch2 >> 4);

  output->electricalAngle = (int16_t)(calc_Merge_m >> 6);

  output->currentQ = state->filteredCurrentDQ[0];

  output->currentD = state->filteredCurrentDQ[1];
}

/* ===================== INISIALISASI FOC ======================= */
/**
 * Menginisialisasi seluruh state controller FOC sebelum loop motor 16 kHz mulai dijalankan.
 */
void MotorController_Init(MotorController *const controller)
{
  MotorParameters *params = ((MotorParameters *) controller->parameters);
  MotorControlState *state = ((MotorControlState *) controller->state);

  state->activeBranch1 = -1;

  state->activeBranch7 = -1;

  state->activeBranch6 = -1;

  state->activeBranch5 = -1;

  state->activeSwitchCase2 = -1;

  state->activeBranch3 = -1;

  state->activeSwitchCase1 = -1;

  state->activeBranch4 = -1;

  state->activeBranch2 = -1;

  state->speedPeriodDelayCurrent = params->counterResetMax;

  state->controlDelayA = true;

  state->previousRawCounter = params->counterResetMax;

  CounterI16_Init(&state->transitionCounter, params->counterResetMax);

  DebounceFilter_Init(&state->diagnosticDebounce);

  state->openLoopInitialize = true;

  BackCalculation_Init(&state->qCurrentBackCalc, 65536000);

  BackCalculation_Init(&state->speedBackCalc, 65536000);

  BackCalculation_Init(&state->voltageBackCalc, 58982400);

  state->voltageDMax = 14400;

  state->voltageDMin = -14400;

  state->voltageQMaxLimit = 14400;

  state->voltageQMin = -14400;

  state->maxCurrent = 12000;

  state->motorCurrentMin = -12000;

  state->currentQMax = 12000;

  state->currentQMin = -12000;

  PidSpeed_Init(&state->speedPid);

  PidTorque_Init(&state->torquePid);

  PidCurrent_Init(&state->currentPid);

}
