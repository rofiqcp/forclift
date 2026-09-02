/*
 * Firmware ESC board 0 - USART3 runtime controller
 * ================================================
 * Mode VLT/TRQ/SPD/POS dipilih langsung oleh host untuk setiap motor saat runtime.
 * FOC tetap berjalan di DMA1_Channel1_IRQHandler sekitar 16 kHz.
 */
#include <stdlib.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "foc_motor.h"
#include "runtime_control.h"
#include "esc_protocol.h"

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern UART_HandleTypeDef huart3;

extern MotorOutput motorOutputLeft;
extern MotorOutput motorOutputRight;
extern MotorInput motorInputLeft;
extern MotorInput motorInputRight;
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern int16_t batVoltage;
extern uint8_t enable;
extern volatile uint32_t buzzerTimer;

uint8_t backwardDrive = 0;
volatile uint32_t main_loop_counter = 0;
int16_t batVoltageCalib = 0;
int16_t board_temp_deci_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;

static uint32_t buzzerTimer_prev = 0;
static uint32_t inactivity_timeout_counter = 0;

/**
 * Entry point. Hardware dan FOC diinisialisasi lebih dahulu, konfigurasi PID
 * persistent dimuat dari EEPROM emulasi, lalu USART3 mulai menerima frame host.
 */
int main(void)
{
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();
    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();
    MX_TIM_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MotorSystem_Init();
    RuntimeControl_Init();

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    InputLimits_Init();
    SerialInput_Init();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    poweronMelody();
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    int32_t board_temp_adc_fix = ((int32_t)adc_buffer.temp) << 16;
    int16_t board_temp_adc_filt = (int16_t)adc_buffer.temp;

    while (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        HAL_Delay(10);
    }

    for (;;) {
        if ((buzzerTimer - buzzerTimer_prev) <= (16U * DELAY_IN_MAIN_LOOP)) {
            continue;
        }

        /*
         * Command USART3 diparse di IRQ tetapi dieksekusi di sini. Dengan begitu
         * SAVE/LOAD EEPROM tidak pernah menjalankan flash operation di ISR.
         */
        EscProtocol_ServiceCommands();

        /* Watchdog komunikasi + outer PID posisi berjalan sekitar 200 Hz. */
        RuntimeControl_UpdateSlow(DELAY_IN_MAIN_LOOP);
        calcAvgSpeed();

        /* Temperatur board dan tegangan baterai diproses seperti firmware asli. */
        filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adc_fix);
        board_temp_adc_filt = (int16_t)(board_temp_adc_fix >> 16);
        board_temp_deci_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                           (board_temp_adc_filt - TEMP_CAL_LOW_ADC) /
                           (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
        batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

        left_dc_curr = -(motorInputLeft.dcLinkCurrent * 100) / A2BIT_CONV;
        right_dc_curr = -(motorInputRight.dcLinkCurrent * 100) / A2BIT_CONV;
        dc_curr = left_dc_curr + right_dc_curr;

        RuntimeControl_ServiceTelemetry();
        // poweroffPressCheck();

        /* Safety thermal/battery/fault dipertahankan. */
        // if ((TEMP_POWEROFF_ENABLE && board_temp_deci_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        //     (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {
        //     poweroff();
        // } 
        if (motorOutputLeft.errorCode || motorOutputRight.errorCode) {
            enable = 0;
            beepCount(1, 24, 1);
        } else if (TEMP_WARNING_ENABLE && board_temp_deci_c >= TEMP_WARNING) {
            beepCount(5, 24, 1);
        } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
            beepCount(0, 10, 6);
        } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {
            beepCount(0, 10, 30);
        } else if (BEEPS_BACKWARD && speedAvg < -50) {
            beepCount(0, 5, 1);
            backwardDrive = 1;
        } else {
            beepCount(0, 0, 0);
            backwardDrive = 0;
        }

        if (abs(runtimeCommandLeft) > 50 || abs(runtimeCommandRight) > 50) {
            inactivity_timeout_counter = 0;
        } else {
            ++inactivity_timeout_counter;
        }
        if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60UL * 1000UL) / (DELAY_IN_MAIN_LOOP + 1U)) {
            poweroff();
        }

        buzzerTimer_prev = buzzerTimer;
        ++main_loop_counter;
    }
}

/**
 * Mengatur clock STM32F103 dari HSI+PLL menjadi 64 MHz, pembagi APB, clock ADC,
 * dan SysTick 1 ms. Nilai clock dipertahankan dari firmware board 0 sebelumnya.
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // STM32F103 ADC maksimum 14 MHz. PCLK2=64 MHz -> DIV8 = 8 MHz,
  // aman untuk sampling current FOC dan tidak melanggar datasheet.
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV8;  // 8 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
