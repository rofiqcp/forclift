#ifndef ESC_PROTOCOL_H
#define ESC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define ESC_PROTOCOL_VERSION 4U
#define ESC_FRAME_SIZE       64U
#define ESC_FRAME_START      0xABCDU

/* Jenis pesan host -> STM32. Semua frame command berukuran tetap 64 byte. */
typedef enum {
    ESC_MSG_HELLO            = 1,
    ESC_MSG_CONTROL          = 2,
    ESC_MSG_PID_CONFIG       = 3,
    ESC_MSG_TELEMETRY_CONFIG = 4,
    ESC_MSG_SAVE_EEPROM      = 5,
    ESC_MSG_LOAD_EEPROM      = 6,
    ESC_MSG_ZERO_POSITION    = 7,
    ESC_MSG_DISARM           = 8,
    ESC_MSG_REQUEST_CONFIG   = 9,
    ESC_MSG_TELEMETRY        = 0x80
} EscMessageType;

/* Motor target pada command parameter. */
typedef enum {
    ESC_MOTOR_LEFT  = 0,
    ESC_MOTOR_RIGHT = 1,
    ESC_MOTOR_BOTH  = 2
} EscMotorSelect;

/* Mode runtime. POS adalah outer position PID yang menghasilkan target speed. */
typedef enum {
    ESC_MODE_OPEN = 0,
    ESC_MODE_VLT  = 1,
    ESC_MODE_SPD  = 2,
    ESC_MODE_TRQ  = 3,
    ESC_MODE_POS  = 4
} EscControlMode;

/* Loop PID yang dapat dituning dari interface. */
typedef enum {
    ESC_PID_CURRENT_D = 0,  /* Id */
    ESC_PID_TORQUE_Q  = 1,  /* Iq; ini adalah loop TRQ pada algoritma asli */
    ESC_PID_SPEED     = 2,
    ESC_PID_POSITION  = 3
} EscPidLoop;

/* Halaman telemetry. Interface hanya meminta data yang sedang diamati. */
typedef enum {
    ESC_TELEM_BASIC  = 0,
    ESC_TELEM_FOC    = 1,
    ESC_TELEM_PID    = 2,
    ESC_TELEM_RAW    = 3,
    ESC_TELEM_CONFIG = 4  /* one-shot response untuk membaca gain/limit aktif */
} EscTelemetryPage;

#define ESC_FLAG_ARM              (1U << 0)
#define ESC_FLAG_SAVE_AFTER_APPLY (1U << 1)
#define ESC_STATUS_LINK_OK         (1U << 0)
#define ESC_STATUS_ARMED           (1U << 1)
#define ESC_STATUS_EEPROM_OK       (1U << 2)
#define ESC_STATUS_TIMEOUT         (1U << 3)
#define ESC_STATUS_EEPROM_VERIFIED (1U << 4)
#define ESC_STATUS_SETTINGS_DIRTY   (1U << 5)

#if defined(__GNUC__)
#define ESC_PACKED __attribute__((packed))
#else
#define ESC_PACKED
#pragma pack(push, 1)
#endif

/*
 * Command 64 byte. Gain PID dikirim sebagai Q16.16 sehingga host dapat
 * mengirim nilai pecahan tanpa float pada STM32. Untuk loop FOC lama,
 * nilai dikonversi ke gain raw uint16 yang sudah dipakai algoritma asli.
 */
typedef struct ESC_PACKED {
    uint16_t start;
    uint8_t  version;
    uint8_t  type;
    uint16_t sequence;
    uint8_t  flags;
    uint8_t  motor;
    uint8_t  mode_left;
    uint8_t  mode_right;
    uint8_t  loop;
    uint8_t  telemetry_page;
    uint16_t telemetry_rate_hz;
    int32_t  setpoint_left;
    int32_t  setpoint_right;
    int32_t  kp_q16;
    int32_t  ki_q16;
    int32_t  kd_q16;
    int32_t  i_limit_q16;
    int32_t  output_min;
    int32_t  output_max;
    int32_t  position_min;
    int32_t  position_max;
    uint32_t telemetry_mask;
    uint16_t reserved0;
    uint16_t reserved1;
    uint16_t checksum;
} EscCommandFrame;

/* Feedback selalu 64 byte. Isi payload ditentukan oleh field page. */
typedef struct ESC_PACKED {
    uint16_t start;
    uint8_t  version;
    uint8_t  type;
    uint16_t sequence;
    uint8_t  page;
    uint8_t  status;
    uint32_t uptime_ms;
    uint8_t  mode_left;
    uint8_t  mode_right;
    uint8_t  error_left;
    uint8_t  error_right;
    uint8_t  payload[46];
    uint16_t checksum;
} EscFeedbackFrame;

/* Payload halaman BASIC (36 byte, sisanya payload diisi nol). */
typedef struct ESC_PACKED {
    int32_t position_left;
    int32_t position_right;
    int32_t setpoint_left;
    int32_t setpoint_right;
    int16_t speed_left;
    int16_t speed_right;
    int16_t battery_centi_volt;
    int16_t temperature_deci_c;
    int16_t dc_current_left_centi_amp;
    int16_t dc_current_right_centi_amp;
    int16_t command_left;
    int16_t command_right;
    uint16_t link_age_ms;
    uint16_t telemetry_rate_hz;
} EscTelemetryBasic;

/* Payload halaman FOC (42 byte). */
typedef struct ESC_PACKED {
    int16_t id_left;
    int16_t iq_left;
    int16_t id_right;
    int16_t iq_right;
    int16_t phase_a_left;
    int16_t phase_b_left;
    int16_t phase_b_right;
    int16_t phase_c_right;
    int16_t dc_link_left;
    int16_t dc_link_right;
    int16_t duty_u_left;
    int16_t duty_v_left;
    int16_t duty_w_left;
    int16_t duty_u_right;
    int16_t duty_v_right;
    int16_t duty_w_right;
    int16_t electrical_angle_left;
    int16_t electrical_angle_right;
    int16_t speed_left;
    int16_t speed_right;
    uint16_t hall_bits;
} EscTelemetryFoc;

/* Payload halaman PID (44 byte). */
typedef struct ESC_PACKED {
    uint8_t loop;
    uint8_t reserved;
    int32_t setpoint_left;
    int32_t setpoint_right;
    int32_t measured_left;
    int32_t measured_right;
    int32_t error_left;
    int32_t error_right;
    int16_t p_left;
    int16_t i_left;
    int16_t d_left;
    int16_t output_left;
    int16_t p_right;
    int16_t i_right;
    int16_t d_right;
    int16_t output_right;
    uint16_t antiwindup_flags;
} EscTelemetryPid;

/* Payload raw ADC/Hall untuk debugging dan kalibrasi. */
typedef struct ESC_PACKED {
    int16_t adc_phase_a_left;
    int16_t adc_phase_b_left;
    int16_t adc_phase_b_right;
    int16_t adc_phase_c_right;
    int16_t adc_dc_left;
    int16_t adc_dc_right;
    int16_t adc_battery;
    int16_t adc_temperature;
    uint8_t hall_left;
    uint8_t hall_right;
    uint16_t pwm_period;
    uint32_t main_loop_counter;
    uint32_t valid_frames;
    uint32_t bad_frames;
    uint32_t reconnect_counter;
    int16_t core_speed_left;
    int16_t core_speed_right;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
} EscTelemetryRaw;

/* Payload one-shot untuk membaca tuning yang sedang aktif dari STM32. */
typedef struct ESC_PACKED {
    uint8_t motor;
    uint8_t loop;
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t i_limit_q16;
    int32_t output_min;
    int32_t output_max;
    int32_t position_min;
    int32_t position_max;
    uint16_t position_deadband_ticks;
    uint16_t eeprom_crc;
    uint16_t eeprom_generation;
    uint16_t eeprom_verify_failures;
    uint8_t eeprom_verified;
    uint8_t settings_dirty;
    uint8_t reserved2;
    uint8_t reserved3;
} EscTelemetryConfig;

#if !defined(__GNUC__)
#pragma pack(pop)
#endif

/* CRC16-CCITT-FALSE untuk 62 byte pertama frame. */
uint16_t EscProtocol_Crc16(const uint8_t *data, uint16_t length);

/* Mulai RX circular DMA USART3. */
void EscProtocol_StartRx(void);

/* Eksekusi command queue di main-loop; operasi berat tidak dijalankan dari IRQ. */
void EscProtocol_ServiceCommands(void);

/* Dipanggil dari IRQ IDLE USART3 untuk memproses byte baru dari DMA. */
void usart3_rx_check(void);

/* Mengirim satu feedback frame bila DMA TX sedang bebas. */
bool EscProtocol_SendFeedback(const EscFeedbackFrame *frame);

/* Statistik parser untuk telemetry/debug. */
uint32_t EscProtocol_GetValidFrameCount(void);
uint32_t EscProtocol_GetBadFrameCount(void);

#endif /* ESC_PROTOCOL_H */
