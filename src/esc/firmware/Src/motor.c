/*
* This file implements FOC motor control.
* This control method offers superior performanace
* compared to previous cummutation method. The new method features:
* ► reduced noise and vibrations
* ► smooth torque output
* ► improved motor efficiency -> lower energy consumption
*
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "runtime_control.h"

// Antarmuka controller FOC yang sudah diberi nama C biasa.
#include "foc_motor.h"           /* controller FOC */
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

extern MotorController *const motorControllerLeft;
extern MotorController *const motorControllerRight;

extern MotorControlState   motorStateLeft;                  /* state internal */
extern MotorInput motorInputLeft;                   /* input FOC */
extern MotorOutput motorOutputLeft;                   /* output FOC */
extern MotorParameters    motorParamsLeft;
extern MotorParameters    motorParamsRight;

extern MotorControlState   motorStateRight;                 /* state internal */
extern MotorInput motorInputRight;                  /* input FOC */
extern MotorOutput motorOutputRight;                  /* output FOC */

static int16_t pwm_margin;              /* This margin allows to have a window in the PWM signal for proper FOC Phase currents measurement */

static int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
int16_t curL_phaA = 0, curL_phaB = 0, curL_DC = 0;
int16_t curR_phaB = 0, curR_phaC = 0, curR_DC = 0;

volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;

uint8_t        enable       = 0;        // initially motors are disabled for SAFETY
static uint8_t enableFin    = 0;

static const uint16_t pwm_res  = 64000000 / 2 / PWM_FREQ; // = 2000

static uint16_t offsetcount = 0;
static int16_t offsetrlA    = 2000;
static int16_t offsetrlB    = 2000;
static int16_t offsetrrB    = 2000;
static int16_t offsetrrC    = 2000;
static int16_t offsetdcl    = 2000;
static int16_t offsetdcr    = 2000;

int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

int32_t odom_l = 0;
int32_t odom_r = 0;

static uint16_t wp_l_vorher = 0;
static uint16_t wp_r_vorher = 0;
static bool odom_l_initialized = false;
static bool odom_r_initialized = false;

/**
 * Modulo aman untuk nilai negatif. Dipakai saat odometri Hall berputar melewati
 * batas indeks. Rumus sengaja sama dengan implementasi sebelumnya.
 */
int16_t modulo(int16_t m, int16_t rest_classes){
  return (((m % rest_classes) + rest_classes) %rest_classes);
}

/**
 * Menentukan arah perpindahan sektor Hall dari posisi sebelumnya ke posisi baru.
 * Lookup enam elemen dipertahankan identik agar hitungan odometri tidak berubah.
 */
int16_t up_or_down(int16_t previous_position, int16_t current_position){
  const int16_t up_down[6] = {0, -1, -2, 0, 2, 1};
  return up_down[modulo(previous_position - current_position, 6)];
}

/**
 * Memperbarui posisi Hall signed tanpa wrap. Sampel valid pertama hanya dipakai
 * sebagai referensi agar boot tidak menghasilkan loncatan posisi palsu. Encoding
 * Hall 000 dan 111 diabaikan karena bukan sektor komutasi yang valid.
 */
static void update_signed_odometry(int32_t *odometry, uint16_t *previous_position,
                                   bool *initialized, uint8_t hall_encoding,
                                   int16_t direction_sign)
{
  if (hall_encoding == 0U || hall_encoding == 7U) {
    return;
  }

  const int16_t current_position = motorLookupTables.hallToPosition[hall_encoding];
  if (!*initialized) {
    *previous_position = (uint16_t)current_position;
    *initialized = true;
    return;
  }

  *odometry += (int32_t)direction_sign *
               (int32_t)up_or_down((int16_t)*previous_position, current_position);
  *previous_position = (uint16_t)current_position;
}

// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
/**
 * Interrupt utama motor sekitar 16 kHz. Di sinilah arus fase dibaca, proteksi
 * current chopping dijalankan, Hall dibaca, MotorController_Update() dipanggil
 * untuk motor kiri/kanan, lalu duty PWM tiga-fasa diterapkan ke TIM8/TIM1.
 */
void DMA1_Channel1_IRQHandler(void) {

  DMA1->IFCR = DMA_IFCR_CTCIF1;
  // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
  // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);

  if(offsetcount < 2000) {  // calibrate ADC offsets
    offsetcount++;
    offsetrlA = (adc_buffer.rlA + offsetrlA) / 2;
    offsetrlB = (adc_buffer.rlB + offsetrlB) / 2;
    offsetrrB = (adc_buffer.rrB + offsetrrB) / 2;
    offsetrrC = (adc_buffer.rrC + offsetrrC) / 2;
    offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
    offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
    return;
  }

  if (buzzerTimer % 1000 == 0) {  // Filter battery voltage at a slower sampling rate
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }

  // Get Left motor currents
  curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
  curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
  curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);

  // Get Right motor currents
  curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
  curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
  curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if(ABS(curL_DC) > curDC_max || enable == 0) {
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  if(ABS(curR_DC)  > curDC_max || enable == 0) {
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
  } else {
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
  }

  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
    if (buzzerPrev == 0) {
      buzzerPrev = 1;
      if (++buzzerIdx > (buzzerCount + 2)) {    // pause 2 periods
        buzzerIdx = 1;
      }
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0)) {
      HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
    }
  } else if (buzzerPrev) {
      HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
      buzzerPrev = 0;
  }

  // Adjust pwm_margin depending on the selected Control Type
  if (motorParamsLeft.controlType == FOC_CTRL || motorParamsRight.controlType == FOC_CTRL) {
    /* Satu margin aman dipakai bersama karena kedua timer membaca arus pada
     * jendela PWM yang sama. Jangan hanya mengikuti motor kiri. */
    pwm_margin = 110;
  } else {
    pwm_margin = 0;
  }

  // ############################### MOTOR CONTROL ###############################

  int ul, vl, wl;
  int ur, vr, wr;
  static bool overrunFlag = false;

  /* Check for overrun */
  if (overrunFlag) {
    return;
  }
  overrunFlag = true;

  /* Make sure to stop BOTH motors in case of an error */
  enableFin = enable && !motorOutputLeft.errorCode && !motorOutputRight.errorCode;

  // ========================= LEFT MOTOR ============================
    // Get hall sensors values
    uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
    uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
    uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

    /* Set motor inputs here */
    motorInputLeft.motorEnable     = enableFin;
    motorInputLeft.controlModeRequest = controlModeLeftFoc;
    motorInputLeft.targetInput     = pwml;
    motorInputLeft.hallA      = hall_ul;
    motorInputLeft.hallB      = hall_vl;
    motorInputLeft.hallC      = hall_wl;
    motorInputLeft.phaseCurrentAB      = curL_phaA;
    motorInputLeft.phaseCurrentBC      = curL_phaB;
    motorInputLeft.dcLinkCurrent     = curL_DC;
    // motorInputLeft.mechanicalAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`

    /* Step the controller */
    MotorController_Update(motorControllerLeft);

    /* Get motor outputs here */
    ul            = motorOutputLeft.dutyPhaseA;
    vl            = motorOutputLeft.dutyPhaseB;
    wl            = motorOutputLeft.dutyPhaseC;
  // errCodeLeft  = motorOutputLeft.errorCode;
  // motSpeedLeft = motorOutputLeft.motorSpeed;
  // motAngleLeft = motorOutputLeft.electricalAngle;
    uint8_t encoding = (uint8_t)((hall_ul<<2) + (hall_vl<<1) + hall_wl);
    update_signed_odometry(&odom_l, &wp_l_vorher, &odom_l_initialized, encoding, +1);

    /* Apply commands */
    LEFT_TIM->LEFT_TIM_U    = (uint16_t)CLAMP(ul + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_V    = (uint16_t)CLAMP(vl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    LEFT_TIM->LEFT_TIM_W    = (uint16_t)CLAMP(wl + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
  // =================================================================

  // ========================= RIGHT MOTOR ===========================
    // Get hall sensors values
    uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
    uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
    uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

    /* Set motor inputs here */
    motorInputRight.motorEnable      = enableFin;
    motorInputRight.controlModeRequest  = controlModeRightFoc;
    motorInputRight.targetInput      = pwmr;
    motorInputRight.hallA       = hall_ur;
    motorInputRight.hallB       = hall_vr;
    motorInputRight.hallC       = hall_wr;
    motorInputRight.phaseCurrentAB       = curR_phaB;
    motorInputRight.phaseCurrentBC       = curR_phaC;
    motorInputRight.dcLinkCurrent      = curR_DC;
    // motorInputRight.mechanicalAngle   = ...; // Angle input in DEGREES [0,360] in fixdt(1,16,4) data type. If `angle` is float use `= (int16_t)floor(angle * 16.0F)` If `angle` is integer use `= (int16_t)(angle << 4)`

    /* Step the controller */
    MotorController_Update(motorControllerRight);

    /* Get motor outputs here */
    ur            = motorOutputRight.dutyPhaseA;
    vr            = motorOutputRight.dutyPhaseB;
    wr            = motorOutputRight.dutyPhaseC;

    encoding = (uint8_t)((hall_ur<<2) + (hall_vr<<1) + hall_wr);
    update_signed_odometry(&odom_r, &wp_r_vorher, &odom_r_initialized, encoding, -1);

    RIGHT_TIM->RIGHT_TIM_U  = (uint16_t)CLAMP(ur + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_V  = (uint16_t)CLAMP(vr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);
    RIGHT_TIM->RIGHT_TIM_W  = (uint16_t)CLAMP(wr + pwm_res / 2, pwm_margin, pwm_res-pwm_margin);

  overrunFlag = false;
}
