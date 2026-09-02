#ifndef RUNTIME_CONTROL_H
#define RUNTIME_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "esc_protocol.h"

/* Konfigurasi PID posisi Q16.16. Output PID adalah target speed FOC [-1000..1000]. */
typedef struct {
    int32_t kp_q16;
    int32_t ki_q16;
    int32_t kd_q16;
    int32_t integral_limit;
    int32_t output_min;
    int32_t output_max;
    int32_t position_min;
    int32_t position_max;
    uint16_t deadband_ticks;
} PositionPidConfig;

typedef struct {
    int32_t integral;
    int32_t previous_error;
    int32_t last_error;
    int16_t last_p;
    int16_t last_i;
    int16_t last_d;
    int16_t last_output;
    bool antiwindup_active;
    bool initialized;
} PositionPidState;

/* Mode FOC efektif per motor yang dibaca interrupt motor.c. */
extern volatile uint8_t controlModeLeftFoc;
extern volatile uint8_t controlModeRightFoc;

/* Setpoint runtime host dan command akhir ke FOC. */
extern volatile int32_t runtimeSetpointLeft;
extern volatile int32_t runtimeSetpointRight;
extern volatile int16_t runtimeCommandLeft;
extern volatile int16_t runtimeCommandRight;

extern PositionPidConfig positionPidConfigLeft;
extern PositionPidConfig positionPidConfigRight;
extern PositionPidState positionPidStateLeft;
extern PositionPidState positionPidStateRight;

/* Inisialisasi runtime mode, PID posisi, EEPROM dan state link. */
void RuntimeControl_Init(void);

/* Dipanggil parser untuk setiap command valid 64-byte. */
void RuntimeControl_OnCommand(const EscCommandFrame *frame);

/* Dipanggil main loop setiap ~5 ms: watchdog, arming dan PID posisi. */
void RuntimeControl_UpdateSlow(uint32_t dt_ms);

/* Membentuk lalu mengirim telemetry bila periodenya tiba. */
void RuntimeControl_ServiceTelemetry(void);

/* Simpan/muat semua gain PID, limit posisi dan parameter motor ke EEPROM emulasi. */
bool RuntimeSettings_Save(void);
bool RuntimeSettings_Load(void);

/* Status runtime untuk GUI/ROS. */
bool RuntimeControl_LinkActive(void);
bool RuntimeControl_Armed(void);
uint16_t RuntimeControl_LinkAgeMs(void);
uint32_t RuntimeControl_GetReconnectCount(void);
uint16_t RuntimeSettings_GetStoredCrc(void);
uint16_t RuntimeSettings_GetGeneration(void);
uint16_t RuntimeSettings_GetVerifyFailures(void);
bool RuntimeSettings_Verified(void);

/* Nol-kan posisi signed tanpa membuat wrap. */
void MotorOdometry_Zero(uint8_t motor_select);

#endif /* RUNTIME_CONTROL_H */
