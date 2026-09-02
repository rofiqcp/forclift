#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

/*
 * KONFIGURASI FIRMWARE TUNGGAL
 * ============================
 * Firmware ini sengaja hanya mendukung hardware yang digunakan sekarang:
 * - Board mapping : BOARD_VARIANT 0
 * - Perintah      : USART3 pada PB10/PB11
 * - Feedback      : USART3 pada PB10/PB11
 * - PA2 dan PA3   : tetap sebagai ADC2 channel 2 dan channel 3
 *
 * Tidak ada cabang konfigurasi hardware lain pada source aktif ini.
 * PWM tiga-fasa untuk inverter motor tetap dipakai karena merupakan bagian wajib
 * dari penggerak FOC, bukan sebagai antarmuka input eksternal.
 */

/* Timing utama */
#define PWM_FREQ                 16000
#define DEAD_TIME               48
#define DELAY_IN_MAIN_LOOP       5
#define TIMEOUT                  20
#define A2BIT_CONV               50

/* Timing ADC */
#define ADC_CONV_TIME_1C5        14
#define ADC_CONV_TIME_7C5        20
#define ADC_CONV_TIME_13C5       26
#define ADC_CONV_TIME_28C5       41
#define ADC_CONV_TIME_41C5       54
#define ADC_CONV_TIME_55C5       68
#define ADC_CONV_TIME_71C5       84
#define ADC_CONV_TIME_239C5      252
#define ADC_CONV_CLOCK_CYCLES    ADC_CONV_TIME_7C5
#define ADC_CLOCK_DIV            4
#define ADC_TOTAL_CONV_TIME      (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)

/* Mapping board yang digunakan */
#define BOARD_VARIANT            0

/* Battery */
#define BAT_FILT_COEF            655
#define BAT_CALIB_REAL_VOLTAGE   3970
#define BAT_CALIB_ADC            1492
#define BAT_CELLS                10
#define BAT_LVL2_ENABLE          0
#define BAT_LVL1_ENABLE          1
#define BAT_BLINK_INTERVAL       80
#define BAT_LVL5                 ((390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL4                 ((380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL3                 ((370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL2                 ((360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL1                 ((350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_DEAD                 ((337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)

/* Temperature */
#define TEMP_FILT_COEF           655
#define TEMP_CAL_LOW_ADC         1655
#define TEMP_CAL_LOW_DEG_C       358
#define TEMP_CAL_HIGH_ADC        1588
#define TEMP_CAL_HIGH_DEG_C      489
#define TEMP_WARNING_ENABLE      0
#define TEMP_WARNING             600
#define TEMP_POWEROFF_ENABLE     0
#define TEMP_POWEROFF            650

/* Jenis dan mode controller motor */
#define COM_CTRL                 0
#define SIN_CTRL                 1
#define FOC_CTRL                 2
#define OPEN_MODE                0
#define VLT_MODE                 1
#define SPD_MODE                 2
#define TRQ_MODE                 3
#define POS_MODE                 4   /* mode posisi: outer PID posisi -> inner FOC speed PID */

#define MOTOR_LEFT_ENA
#define MOTOR_RIGHT_ENA
#define CTRL_TYP_SEL             FOC_CTRL
#define DIAG_ENA                 1

/* Batas motor - nilai dipertahankan sama dengan konfigurasi USART yang digunakan */
#define I_MOT_MAX                15
#define I_DC_MAX                 17
#define N_MOT_MAX               1000
#define FIELD_WEAK_ENA           0
#define FIELD_WEAK_MAX           5
#define PHASE_ADV_MAX            25
#define FIELD_WEAK_HI            1000
#define FIELD_WEAK_LO            750

/* Safety dan penyimpanan */
#define INACTIVITY_TIMEOUT       30
#define BEEPS_BACKWARD           0
#define ADC_MARGIN               100
#define ADC_PROTECT_TIMEOUT      100
#define ADC_PROTECT_THRESH       200
#define AUTO_CALIBRATION_ENA

/* Filter perintah, fixed-point sama seperti konfigurasi asli */
#define RATE                     480
#define FILTER                   6553
#define SPEED_COEFFICIENT        16384
#define STEER_COEFFICIENT        8192

/* Protokol USART3 runtime. Mode tidak lagi ditentukan compile-time. */
#define INPUTS_NR                1
#define PRI_INPUT1               3, -1000, 0, 1000, 0
#define PRI_INPUT2               3, -1000, 0, 1000, 0
#define FLASH_WRITE_KEY          0xEC03
#define SERIAL_START_FRAME       0xABCD
#define SERIAL_BUFFER_SIZE       256
#define SERIAL_TIMEOUT_MS        300
#define TELEMETRY_RATE_DEFAULT   50
#define TELEMETRY_RATE_MAX       100
#define POSITION_MIN_DEFAULT    (-200000L)
#define POSITION_MAX_DEFAULT     200000L
#define POSITION_DEADBAND_DEFAULT 2U

/*
 * Konvensi command mekanik -> core FOC.
 * Firmware asli membalik command motor kanan sebelum masuk BLDC controller:
 *   pwml =  cmdL;
 *   pwmr = -cmdR;
 * Konvensi ini wajib dipertahankan agar setpoint positif berarti arah mekanik
 * yang sama pada kedua motor meskipun pemasangan motor kanan bercermin.
 */
#define MOTOR_COMMAND_SIGN_LEFT   1
#define MOTOR_COMMAND_SIGN_RIGHT -1

#define USART3_BAUD              115200
#define USART3_WORDLENGTH        UART_WORDLENGTH_8B

#endif /* CONFIG_H */
