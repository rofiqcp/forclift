#include "runtime_control.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "esc_protocol.h"
#include "foc_motor.h"
#include "stm32f1xx_hal.h"
#include <limits.h>
#include <string.h>

extern MotorParameters motorParamsLeft;
extern MotorParameters motorParamsRight;
extern MotorControlState motorStateLeft;
extern MotorControlState motorStateRight;
extern MotorInput motorInputLeft;
extern MotorInput motorInputRight;
extern MotorOutput motorOutputLeft;
extern MotorOutput motorOutputRight;
extern volatile adc_buf_t adc_buffer;
extern volatile uint32_t main_loop_counter;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deci_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t curL_phaA, curL_phaB, curL_DC;
extern int16_t curR_phaB, curR_phaC, curR_DC;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int32_t odom_l;
extern int32_t odom_r;

volatile uint8_t controlModeLeftFoc = OPEN_MODE;
volatile uint8_t controlModeRightFoc = OPEN_MODE;
volatile int32_t runtimeSetpointLeft = 0;
volatile int32_t runtimeSetpointRight = 0;
volatile int16_t runtimeCommandLeft = 0;
volatile int16_t runtimeCommandRight = 0;

PositionPidConfig positionPidConfigLeft;
PositionPidConfig positionPidConfigRight;
PositionPidState positionPidStateLeft;
PositionPidState positionPidStateRight;

static uint8_t requestedModeLeft = ESC_MODE_OPEN;
static uint8_t requestedModeRight = ESC_MODE_OPEN;
static bool linkActive = false;
static bool armRequested = false;
static bool armed = false;
static bool eepromOk = false;
static bool eepromVerified = false;
static uint16_t eepromStoredCrc = 0U;
static uint16_t eepromGeneration = 0U;
static uint16_t eepromVerifyFailures = 0U;
static bool settingsDirty = false;
static uint32_t lastCommandTick = 0;
static uint32_t reconnectCounter = 0;
static uint16_t feedbackSequence = 0;
static uint16_t lastHostSequence = 0;
static uint8_t telemetryPage = ESC_TELEM_BASIC;
static uint8_t telemetryPidLoop = ESC_PID_SPEED;
static uint8_t configMotor = ESC_MOTOR_LEFT;
static uint8_t telemetryPageBeforeConfig = ESC_TELEM_BASIC;
static bool configOneShotPending = false;
static uint16_t telemetryRateHz = TELEMETRY_RATE_DEFAULT;
static uint32_t telemetryMask = 0xFFFFFFFFUL;
static uint32_t lastTelemetryTick = 0;

/* EEPROM emulation memerlukan tabel seluruh virtual address yang ikut page transfer. */
uint16_t VirtAddVarTab[NB_OF_VAR] = {
    2000,2001,2002,2003,2004,2005,2006,2007,2008,2009,2010,2011,2012,2013,2014,2015,
    2016,2017,2018,2019,2020,2021,2022,2023,2024,2025,2026,2027,2028,2029,2030,2031,
    2032,2033,2034,2035,2036,2037,2038,2039,2040,2041,2042,2043,2044,2045,2046,2047,
    2048,2049,2050,2051,2052,2053,2054,2055,2056,2057,2058,2059,2060,2061,2062,2063
};

#define EEPROM_CONFIG_VERSION 5U
#define EEPROM_WORD_KEY       0U
#define EEPROM_WORD_VERSION   1U
#define EEPROM_WORD_MAX_CURRENT 2U
#define EEPROM_WORD_MAX_SPEED   3U
#define EEPROM_LEFT_FOC_BASE    4U
#define EEPROM_RIGHT_FOC_BASE   16U
#define EEPROM_LEFT_POS_BASE    28U
#define EEPROM_RIGHT_POS_BASE   44U
#define EEPROM_WORD_TELEM_RATE  60U
#define EEPROM_WORD_TELEM_PAGE  61U
#define EEPROM_WORD_GENERATION  62U
#define EEPROM_WORD_CRC         63U

static int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/* Clamp hasil kalkulasi int64 sebelum dikonversi ke int32. Ini mencegah
 * overflow/implementation-defined cast ketika posisi atau gain mendekati batas. */
static int32_t clamp_i64_to_i32(int64_t value, int32_t lower, int32_t upper)
{
    if (value < (int64_t)lower) return lower;
    if (value > (int64_t)upper) return upper;
    return (int32_t)value;
}

static int16_t clamp_i16(int32_t value, int16_t lower, int16_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return (int16_t)value;
}

static uint16_t q16_to_gain_u16(int32_t value)
{
    int32_t rounded = (value >= 0) ? ((value + 32768) >> 16) : 0;
    if (rounded > 65535) rounded = 65535;
    return (uint16_t)rounded;
}


/* Default posisi sengaja konservatif. Pengguna dapat tuning dari GUI lalu Save EEPROM. */
static void PositionPid_SetDefaults(PositionPidConfig *cfg, PositionPidState *state)
{
    cfg->kp_q16 = 16384;        /* 0.25 output/tick */
    cfg->ki_q16 = 0;
    cfg->kd_q16 = 0;
    cfg->integral_limit = 200;
    cfg->output_min = -300;
    cfg->output_max = 300;
    cfg->position_min = POSITION_MIN_DEFAULT;
    cfg->position_max = POSITION_MAX_DEFAULT;
    cfg->deadband_ticks = POSITION_DEADBAND_DEFAULT;
    memset(state, 0, sizeof(*state));
}

/*
 * PID posisi 200 Hz dengan conditional-integration anti-windup.
 * P/I/D dihitung dengan int64 sementara agar perkalian Q16.16 tidak overflow.
 */
static int16_t PositionPid_Update(const PositionPidConfig *cfg,
                                  PositionPidState *state,
                                  int32_t setpoint,
                                  int32_t measured,
                                  uint32_t dt_ms)
{
    /* Outer-loop normalnya 5 ms. Batasi dt agar satu stall host/main-loop tidak
     * membuat komponen I/D melonjak ekstrem saat eksekusi dilanjutkan. */
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 100U) dt_ms = 100U;

    const int64_t error64 = (int64_t)setpoint - (int64_t)measured;
    const int32_t error = clamp_i64_to_i32(error64, INT32_MIN, INT32_MAX);

    /*
     * Deadband posisi mencegah hunting satu-dua sektor Hall di sekitar target.
     * Integrator direset ketika target sudah tercapai agar motor benar-benar STOP
     * dan tidak terus berjalan akibat sisa integral dari error sebelumnya.
     */
    if ((error64 >= -(int64_t)cfg->deadband_ticks) &&
        (error64 <=  (int64_t)cfg->deadband_ticks)) {
        state->integral = 0;
        state->previous_error = error;
        state->last_error = error;
        state->last_p = 0;
        state->last_i = 0;
        state->last_d = 0;
        state->last_output = 0;
        state->antiwindup_active = false;
        state->initialized = true;
        return 0;
    }

    if (!state->initialized) {
        state->previous_error = error;
        state->initialized = true;
    }

    /* Semua perkalian dilakukan pada int64 dan diclamp sebelum turun tipe. */
    const int64_t p64 = ((int64_t)cfg->kp_q16 * (int64_t)error) >> 16;

    int64_t delta_error64 = (int64_t)error - (int64_t)state->previous_error;
    if (delta_error64 > INT32_MAX) delta_error64 = INT32_MAX;
    if (delta_error64 < INT32_MIN) delta_error64 = INT32_MIN;
    const int64_t d_scaled = ((int64_t)cfg->kd_q16 * delta_error64) >> 16;
    const int64_t d64 = (d_scaled * 1000LL) / (int64_t)dt_ms;

    const int64_t i_per_second = ((int64_t)cfg->ki_q16 * (int64_t)error) >> 16;
    const int64_t delta_i64 = (i_per_second * (int64_t)dt_ms) / 1000LL;

    const int32_t p = clamp_i64_to_i32(p64, INT16_MIN, INT16_MAX);
    const int32_t d = clamp_i64_to_i32(d64, INT16_MIN, INT16_MAX);
    const int32_t candidate_i = clamp_i64_to_i32(
        (int64_t)state->integral + delta_i64,
        -cfg->integral_limit, cfg->integral_limit);

    const int64_t candidate_output = (int64_t)p + (int64_t)candidate_i + (int64_t)d;
    const bool upper_sat = candidate_output > (int64_t)cfg->output_max;
    const bool lower_sat = candidate_output < (int64_t)cfg->output_min;

    /* Anti-windup: integrator dibekukan bila saturasi dan error mendorong lebih jauh ke batas. */
    state->antiwindup_active = (upper_sat && error > 0) || (lower_sat && error < 0);
    if (!state->antiwindup_active) {
        state->integral = candidate_i;
    }

    const int32_t output = clamp_i64_to_i32(
        (int64_t)p + (int64_t)state->integral + (int64_t)d,
        cfg->output_min, cfg->output_max);
    state->last_error = error;
    state->last_p = clamp_i16(p, INT16_MIN, INT16_MAX);
    state->last_i = clamp_i16(state->integral, INT16_MIN, INT16_MAX);
    state->last_d = clamp_i16(d, INT16_MIN, INT16_MAX);
    state->last_output = clamp_i16(output, INT16_MIN, INT16_MAX);
    state->previous_error = error;
    return state->last_output;
}

static void PositionPid_Reset(PositionPidState *state)
{
    memset(state, 0, sizeof(*state));
}

static bool mode_valid(uint8_t mode)
{
    return mode <= ESC_MODE_POS;
}

/* Saat transisi DISARM->ARM, target harus aman agar hot-plug tidak menyebabkan lonjakan. */
static bool safe_to_arm_one(uint8_t mode, int32_t target, int32_t position,
                            const PositionPidConfig *position_cfg)
{
    if (mode == ESC_MODE_OPEN) return true;
    if (mode == ESC_MODE_POS) {
        /* Jangan ARM bila posisi aktual sendiri sudah di luar soft-limit. Jika
         * dibiarkan, target safe=current akan lolos lalu target efektif di-clamp
         * dan motor langsung bergerak menuju batas begitu ARM aktif. */
        if (position < position_cfg->position_min || position > position_cfg->position_max) return false;
        target = clamp_i32(target, position_cfg->position_min, position_cfg->position_max);
        const int64_t delta = (int64_t)target - (int64_t)position;
        return delta > -200LL && delta < 200LL;
    }
    return (target > -50) && (target < 50);
}

static bool safe_to_arm(void)
{
    return safe_to_arm_one(requestedModeLeft, runtimeSetpointLeft, odom_l, &positionPidConfigLeft) &&
           safe_to_arm_one(requestedModeRight, runtimeSetpointRight, odom_r, &positionPidConfigRight);
}

static void disarm_outputs(void)
{
    armed = false;
    enable = 0;
    runtimeCommandLeft = 0;
    runtimeCommandRight = 0;
    pwml = 0;
    pwmr = 0;
    controlModeLeftFoc = OPEN_MODE;
    controlModeRightFoc = OPEN_MODE;
    PositionPid_Reset(&positionPidStateLeft);
    PositionPid_Reset(&positionPidStateRight);
}

/* Memetakan mode host ke mode yang dimengerti core FOC. POS memakai inner SPD. */
static uint8_t to_foc_mode(uint8_t mode)
{
    if (mode == ESC_MODE_VLT) return VLT_MODE;
    if (mode == ESC_MODE_TRQ) return TRQ_MODE;
    if (mode == ESC_MODE_SPD || mode == ESC_MODE_POS) return SPD_MODE;
    return OPEN_MODE;
}

static void apply_pid_to_motor(MotorParameters *params, uint8_t loop, const EscCommandFrame *frame)
{
    uint16_t kp = q16_to_gain_u16(frame->kp_q16);
    uint16_t ki = q16_to_gain_u16(frame->ki_q16);
    uint16_t kd = q16_to_gain_u16(frame->kd_q16);

    switch (loop) {
        case ESC_PID_CURRENT_D:
            params->currentDKp = kp; params->currentDKi = ki; params->currentDKd = kd; break;
        case ESC_PID_TORQUE_Q:
            /* TRQ pada FOC asli adalah closed-loop Iq, jadi satu PID yang sama. */
            params->currentQKp = kp; params->currentQKi = ki; params->currentQKd = kd; break;
        case ESC_PID_SPEED:
            params->speedKp = kp; params->speedKi = ki; params->speedKd = kd; break;
        default: break;
    }
}

static void apply_position_pid(PositionPidConfig *cfg, const EscCommandFrame *frame)
{
    /* Gain posisi tidak boleh negatif. Host normal sudah membatasi, tetapi
     * firmware tetap melakukan validasi defensif terhadap frame biner mentah. */
    cfg->kp_q16 = frame->kp_q16 < 0 ? 0 : frame->kp_q16;
    cfg->ki_q16 = frame->ki_q16 < 0 ? 0 : frame->ki_q16;
    cfg->kd_q16 = frame->kd_q16 < 0 ? 0 : frame->kd_q16;
    cfg->integral_limit = frame->i_limit_q16 >> 16;
    if (cfg->integral_limit < 0) cfg->integral_limit = -cfg->integral_limit;
    if (cfg->integral_limit > INT16_MAX) cfg->integral_limit = INT16_MAX;
    cfg->output_min = clamp_i32(frame->output_min, -1000, 0);
    cfg->output_max = clamp_i32(frame->output_max, 0, 1000);
    cfg->position_min = frame->position_min;
    cfg->position_max = frame->position_max;
    if (cfg->position_min > cfg->position_max) {
        int32_t t = cfg->position_min; cfg->position_min = cfg->position_max; cfg->position_max = t;
    }
    cfg->deadband_ticks = frame->reserved1;
    if (cfg->deadband_ticks > 1000U) cfg->deadband_ticks = 1000U;
}

void RuntimeControl_Init(void)
{
    requestedModeLeft = requestedModeRight = ESC_MODE_OPEN;
    runtimeSetpointLeft = runtimeSetpointRight = 0;
    telemetryPage = ESC_TELEM_BASIC;
    telemetryPidLoop = ESC_PID_SPEED;
    telemetryRateHz = TELEMETRY_RATE_DEFAULT;
    PositionPid_SetDefaults(&positionPidConfigLeft, &positionPidStateLeft);
    PositionPid_SetDefaults(&positionPidConfigRight, &positionPidStateRight);
    disarm_outputs();

    HAL_FLASH_Unlock();
    const uint16_t ee_init_status = EE_Init();
    HAL_FLASH_Lock();

    eepromOk = (ee_init_status == HAL_OK);
    eepromVerified = false;
    if (eepromOk) {
        eepromOk = RuntimeSettings_Load();
        if (!eepromOk) {
            /* First boot / CRC salah: simpan default aman lalu lakukan read-back verification. */
            eepromOk = RuntimeSettings_Save();
        }
    }
}

void RuntimeControl_OnCommand(const EscCommandFrame *frame)
{
    lastHostSequence = frame->sequence;
    lastCommandTick = HAL_GetTick();

    if (!linkActive) {
        linkActive = true;
        ++reconnectCounter;
    }

    switch ((EscMessageType)frame->type) {
        case ESC_MSG_HELLO:
            /* Handshake selalu disarm. Host wajib kirim neutral/current-position lalu ARM. */
            armRequested = false;
            requestedModeLeft = requestedModeRight = ESC_MODE_OPEN;
            runtimeSetpointLeft = runtimeSetpointRight = 0;
            disarm_outputs();
            break;

        case ESC_MSG_CONTROL:
            if (mode_valid(frame->mode_left) && mode_valid(frame->mode_right)) {
                const bool modeChanged = (frame->mode_left != requestedModeLeft) ||
                                         (frame->mode_right != requestedModeRight);
                requestedModeLeft = frame->mode_left;
                requestedModeRight = frame->mode_right;
                runtimeSetpointLeft = frame->setpoint_left;
                runtimeSetpointRight = frame->setpoint_right;

                /* Perubahan mode ketika sedang ARMED wajib melewati safe-arm lagi. */
                if (modeChanged && armed) {
                    armRequested = false;
                    disarm_outputs();
                } else {
                    armRequested = (frame->flags & ESC_FLAG_ARM) != 0U;
                    if (!armRequested) disarm_outputs();
                }
            }
            break;

        case ESC_MSG_PID_CONFIG:
            if (frame->loop > ESC_PID_POSITION) break;
            /*
             * Gain dipakai FOC ISR frekuensi tinggi. DISARM dahulu agar satu
             * control step tidak membaca campuran Kp baru dan Ki/Kd lama.
             * Setelah apply, user/host melakukan safe-arm kembali.
             */
            armRequested = false;
            disarm_outputs();
            telemetryPidLoop = frame->loop;
            settingsDirty = true;
            if (frame->loop == ESC_PID_POSITION) {
                if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH)
                    apply_position_pid(&positionPidConfigLeft, frame);
                if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH)
                    apply_position_pid(&positionPidConfigRight, frame);
            } else {
                if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH)
                    apply_pid_to_motor(&motorParamsLeft, frame->loop, frame);
                if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH)
                    apply_pid_to_motor(&motorParamsRight, frame->loop, frame);
            }
            if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U) eepromOk = RuntimeSettings_Save();
            break;

        case ESC_MSG_TELEMETRY_CONFIG:
            if (frame->telemetry_page <= ESC_TELEM_RAW) telemetryPage = frame->telemetry_page;
            if (frame->loop <= ESC_PID_POSITION) telemetryPidLoop = frame->loop;
            configOneShotPending = false;
            telemetryRateHz = frame->telemetry_rate_hz;
            if (telemetryRateHz < 1U) telemetryRateHz = 1U;
            if (telemetryRateHz > TELEMETRY_RATE_MAX) telemetryRateHz = TELEMETRY_RATE_MAX;
            telemetryMask = frame->telemetry_mask;
            break;

        case ESC_MSG_SAVE_EEPROM:
            eepromOk = RuntimeSettings_Save();
            break;

        case ESC_MSG_LOAD_EEPROM:
            /* Gain/limit dapat berubah besar; load hanya dilakukan dalam kondisi DISARM. */
            armRequested = false;
            disarm_outputs();
            eepromOk = RuntimeSettings_Load();
            break;

        case ESC_MSG_ZERO_POSITION:
            /* Mengubah referensi posisi saat motor aktif dapat membuat POS melonjak. */
            armRequested = false;
            disarm_outputs();
            MotorOdometry_Zero(frame->motor);
            break;

        case ESC_MSG_DISARM:
            armRequested = false;
            disarm_outputs();
            break;

        case ESC_MSG_REQUEST_CONFIG:
            if (frame->loop <= ESC_PID_POSITION) telemetryPidLoop = frame->loop;
            configMotor = (frame->motor == ESC_MOTOR_RIGHT) ? ESC_MOTOR_RIGHT : ESC_MOTOR_LEFT;
            telemetryPageBeforeConfig = (telemetryPage <= ESC_TELEM_RAW) ? telemetryPage : ESC_TELEM_BASIC;
            telemetryPage = ESC_TELEM_CONFIG;
            configOneShotPending = true;
            break;

        default:
            break;
    }
}

void RuntimeControl_UpdateSlow(uint32_t dt_ms)
{
    uint32_t now = HAL_GetTick();
    uint32_t age = now - lastCommandTick;

    if (!linkActive || age > SERIAL_TIMEOUT_MS) {
        if (linkActive) linkActive = false;
        armRequested = false;
        requestedModeLeft = requestedModeRight = ESC_MODE_OPEN;
        disarm_outputs();
        return;
    }

    if (armed && (motorOutputLeft.errorCode != 0U || motorOutputRight.errorCode != 0U)) {
        armRequested = false;
        disarm_outputs();
        return;
    }

    if (!armed && armRequested) {
        if (safe_to_arm() && motorOutputLeft.errorCode == 0U && motorOutputRight.errorCode == 0U) {
            armed = true;
            enable = 1U;
            PositionPid_Reset(&positionPidStateLeft);
            PositionPid_Reset(&positionPidStateRight);
        }
    }

    if (!armed) {
        disarm_outputs();
        return;
    }

    /* Clamp target posisi terhadap soft-limit sebelum PID. */
    if (requestedModeLeft == ESC_MODE_POS) {
        int32_t sp = clamp_i32(runtimeSetpointLeft, positionPidConfigLeft.position_min, positionPidConfigLeft.position_max);
        runtimeCommandLeft = PositionPid_Update(&positionPidConfigLeft, &positionPidStateLeft, sp, odom_l, dt_ms);
    } else {
        runtimeCommandLeft = clamp_i16(runtimeSetpointLeft, -1000, 1000);
    }

    if (requestedModeRight == ESC_MODE_POS) {
        int32_t sp = clamp_i32(runtimeSetpointRight, positionPidConfigRight.position_min, positionPidConfigRight.position_max);
        runtimeCommandRight = PositionPid_Update(&positionPidConfigRight, &positionPidStateRight, sp, -odom_r, dt_ms);
    } else {
        runtimeCommandRight = clamp_i16(runtimeSetpointRight, -1000, 1000);
    }

    controlModeLeftFoc = to_foc_mode(requestedModeLeft);
    controlModeRightFoc = to_foc_mode(requestedModeRight);

    /*
     * PENTING: runtimeCommand* memakai koordinat mekanik host yang sama untuk
     * kiri/kanan. Core FOC kanan dipasang bercermin, sehingga command kanan
     * harus dibalik seperti firmware asli (pwmr = -cmdR). Hilangnya mapping ini
     * membuat POS kanan menjadi positive-feedback dan dapat berjalan terus.
     */
    pwml = MOTOR_COMMAND_SIGN_LEFT  * runtimeCommandLeft;
    pwmr = MOTOR_COMMAND_SIGN_RIGHT * runtimeCommandRight;
}

bool RuntimeControl_LinkActive(void) { return linkActive; }
bool RuntimeControl_Armed(void) { return armed; }
uint16_t RuntimeControl_LinkAgeMs(void)
{
    uint32_t age = HAL_GetTick() - lastCommandTick;
    return (uint16_t)(age > 65535U ? 65535U : age);
}
uint32_t RuntimeControl_GetReconnectCount(void) { return reconnectCounter; }
uint16_t RuntimeSettings_GetStoredCrc(void) { return eepromStoredCrc; }
uint16_t RuntimeSettings_GetGeneration(void) { return eepromGeneration; }
uint16_t RuntimeSettings_GetVerifyFailures(void) { return eepromVerifyFailures; }
bool RuntimeSettings_Verified(void) { return eepromVerified; }

void MotorOdometry_Zero(uint8_t motor_select)
{
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH) odom_l = 0;
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH) odom_r = 0;
}

/* ----------------------------- EEPROM ---------------------------------- */
static uint16_t eeprom_crc_words(const uint16_t *words, uint8_t count)
{
    /* STM32F103 adalah little-endian. Gunakan CRC16-CCITT-FALSE yang sama
     * dengan frame USART agar corruption detection EEPROM bukan checksum XOR. */
    return EscProtocol_Crc16((const uint8_t *)words, (uint16_t)count * (uint16_t)sizeof(uint16_t));
}

static void put_i32(uint16_t *words, uint8_t index, int32_t value)
{
    words[index] = (uint16_t)((uint32_t)value & 0xFFFFU);
    words[index + 1U] = (uint16_t)(((uint32_t)value >> 16) & 0xFFFFU);
}
static int32_t get_i32(const uint16_t *words, uint8_t index)
{
    return (int32_t)(((uint32_t)words[index + 1U] << 16) | words[index]);
}

static void store_foc_words(uint16_t *w, uint8_t base, const MotorParameters *p)
{
    w[base+0]=p->currentDKp; w[base+1]=p->currentDKi; w[base+2]=p->currentDKd;
    w[base+3]=p->currentQKp; w[base+4]=p->currentQKi; w[base+5]=p->currentQKd;
    w[base+6]=p->speedKp;    w[base+7]=p->speedKi;    w[base+8]=p->speedKd;
    /* base+9..11 dipakai runtime-control, bukan parameter FOC numerik. */
}
static void load_foc_words(const uint16_t *w, uint8_t base, MotorParameters *p)
{
    p->currentDKp=w[base+0]; p->currentDKi=w[base+1]; p->currentDKd=w[base+2];
    p->currentQKp=w[base+3]; p->currentQKi=w[base+4]; p->currentQKd=w[base+5];
    p->speedKp=w[base+6];    p->speedKi=w[base+7];    p->speedKd=w[base+8];
}
static void store_pos_words(uint16_t *w, uint8_t base, const PositionPidConfig *p)
{
    put_i32(w,base+0,p->kp_q16); put_i32(w,base+2,p->ki_q16); put_i32(w,base+4,p->kd_q16);
    put_i32(w,base+6,p->integral_limit); put_i32(w,base+8,p->output_min); put_i32(w,base+10,p->output_max);
    put_i32(w,base+12,p->position_min); put_i32(w,base+14,p->position_max);
}
static void load_pos_words(const uint16_t *w, uint8_t base, PositionPidConfig *p)
{
    p->kp_q16=get_i32(w,base+0); p->ki_q16=get_i32(w,base+2); p->kd_q16=get_i32(w,base+4);
    p->integral_limit=get_i32(w,base+6); p->output_min=get_i32(w,base+8); p->output_max=get_i32(w,base+10);
    p->position_min=get_i32(w,base+12); p->position_max=get_i32(w,base+14);
}

bool RuntimeSettings_Save(void)
{
    /*
     * Menulis/erase flash dapat menghentikan fetch CPU sesaat. Jangan pernah
     * melakukan operasi EEPROM saat output motor aktif; DISARM lebih dahulu
     * agar FOC tidak mengalami jeda PWM/ISR yang tidak terkontrol.
     */
    if (armed || armRequested) {
        armRequested = false;
        disarm_outputs();
    }

    uint16_t w[NB_OF_VAR];
    uint16_t verify[NB_OF_VAR];
    memset(w, 0, sizeof(w));
    memset(verify, 0, sizeof(verify));

    w[EEPROM_WORD_KEY] = FLASH_WRITE_KEY;
    w[EEPROM_WORD_VERSION] = EEPROM_CONFIG_VERSION;
    w[EEPROM_WORD_MAX_CURRENT] = (uint16_t)motorParamsLeft.maxCurrent;
    w[EEPROM_WORD_MAX_SPEED] = (uint16_t)motorParamsLeft.maxSpeed;
    store_foc_words(w, EEPROM_LEFT_FOC_BASE, &motorParamsLeft);
    store_foc_words(w, EEPROM_RIGHT_FOC_BASE, &motorParamsRight);
    /* Gunakan word reserved pada blok FOC untuk parameter outer-position tambahan. */
    w[EEPROM_LEFT_FOC_BASE + 9U] = positionPidConfigLeft.deadband_ticks;
    w[EEPROM_RIGHT_FOC_BASE + 9U] = positionPidConfigRight.deadband_ticks;
    store_pos_words(w, EEPROM_LEFT_POS_BASE, &positionPidConfigLeft);
    store_pos_words(w, EEPROM_RIGHT_POS_BASE, &positionPidConfigRight);
    w[EEPROM_WORD_TELEM_RATE] = telemetryRateHz;
    w[EEPROM_WORD_TELEM_PAGE] = telemetryPage;
    w[EEPROM_WORD_GENERATION] = (uint16_t)(eepromGeneration + 1U);
    if (w[EEPROM_WORD_GENERATION] == 0U) w[EEPROM_WORD_GENERATION] = 1U;
    w[EEPROM_WORD_CRC] = eeprom_crc_words(w, EEPROM_WORD_CRC);

    eepromVerified = false;

    HAL_FLASH_Unlock();
    for (uint8_t i = 0; i < NB_OF_VAR; ++i) {
        /* Kurangi wear flash: tulis hanya bila nilai terakhir berbeda / belum ada. */
        uint16_t current = 0U;
        const uint16_t read_status = EE_ReadVariable(VirtAddVarTab[i], &current);
        if (read_status != 0U || current != w[i]) {
            if (EE_WriteVariable(VirtAddVarTab[i], w[i]) != HAL_OK) {
                HAL_FLASH_Lock();
                ++eepromVerifyFailures;
                return false;
            }
        }
    }
    HAL_FLASH_Lock();

    /* Read-back verification: SAVE baru dianggap sukses bila 64 word identik. */
    for (uint8_t i = 0; i < NB_OF_VAR; ++i) {
        if (EE_ReadVariable(VirtAddVarTab[i], &verify[i]) != 0U) {
            ++eepromVerifyFailures;
            return false;
        }
    }
    if (memcmp(w, verify, sizeof(w)) != 0 ||
        verify[EEPROM_WORD_CRC] != eeprom_crc_words(verify, EEPROM_WORD_CRC)) {
        ++eepromVerifyFailures;
        return false;
    }

    eepromStoredCrc = verify[EEPROM_WORD_CRC];
    eepromGeneration = verify[EEPROM_WORD_GENERATION];
    eepromVerified = true;
    settingsDirty = false;
    return true;
}

static bool persistent_config_valid(const MotorParameters *left, const MotorParameters *right,
                                    const PositionPidConfig *pos_left, const PositionPidConfig *pos_right,
                                    uint16_t max_current_word, uint16_t max_speed_word,
                                    uint16_t telemetry_rate, uint8_t telemetry_page_value)
{
    if (max_current_word == 0U || max_current_word > INT16_MAX ||
        max_speed_word == 0U || max_speed_word > INT16_MAX) return false;

    /* Inner gain dikirim/ditampilkan sebagai signed Q16.16 pada protokol, jadi
     * batasi stored raw gain ke range yang dapat di-round-trip tanpa ambigu. */
    const uint16_t gains[] = {
        left->currentDKp, left->currentDKi, left->currentDKd,
        left->currentQKp, left->currentQKi, left->currentQKd,
        left->speedKp, left->speedKi, left->speedKd,
        right->currentDKp, right->currentDKi, right->currentDKd,
        right->currentQKp, right->currentQKi, right->currentQKd,
        right->speedKp, right->speedKi, right->speedKd
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(gains) / sizeof(gains[0])); ++i) {
        if (gains[i] > 32767U) return false;
    }

    const PositionPidConfig *pos[2] = {pos_left, pos_right};
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (pos[i]->kp_q16 < 0 || pos[i]->ki_q16 < 0 || pos[i]->kd_q16 < 0) return false;
        if (pos[i]->integral_limit < 0 || pos[i]->integral_limit > INT16_MAX) return false;
        if (pos[i]->output_min < -1000 || pos[i]->output_min > 0) return false;
        if (pos[i]->output_max < 0 || pos[i]->output_max > 1000) return false;
        if (pos[i]->output_min > pos[i]->output_max) return false;
        if (pos[i]->position_min > pos[i]->position_max) return false;
        if (pos[i]->deadband_ticks > 1000U) return false;
    }

    if (telemetry_rate < 1U || telemetry_rate > TELEMETRY_RATE_MAX) return false;
    if (telemetry_page_value > ESC_TELEM_RAW) return false;
    return true;
}

bool RuntimeSettings_Load(void)
{
    uint16_t w[NB_OF_VAR];
    memset(w, 0, sizeof(w));
    eepromVerified = false;

    /* Membaca flash tidak memerlukan unlock; unlock hanya untuk erase/program. */
    for (uint8_t i = 0; i < NB_OF_VAR; ++i) {
        if (EE_ReadVariable(VirtAddVarTab[i], &w[i]) != 0U) {
            return false;
        }
    }

    const uint16_t calculated_crc = eeprom_crc_words(w, EEPROM_WORD_CRC);
    if (w[EEPROM_WORD_KEY] != FLASH_WRITE_KEY ||
        w[EEPROM_WORD_VERSION] != EEPROM_CONFIG_VERSION ||
        w[EEPROM_WORD_CRC] != calculated_crc) {
        return false;
    }

    /* Transactional load: decode ke salinan temporary dahulu. Jika satu field
     * tidak valid, parameter aktif di RAM tidak berubah sama sekali. */
    MotorParameters left = motorParamsLeft;
    MotorParameters right = motorParamsRight;
    PositionPidConfig pos_left = positionPidConfigLeft;
    PositionPidConfig pos_right = positionPidConfigRight;

    left.maxCurrent = right.maxCurrent = (int16_t)w[EEPROM_WORD_MAX_CURRENT];
    left.maxSpeed = right.maxSpeed = (int16_t)w[EEPROM_WORD_MAX_SPEED];
    load_foc_words(w, EEPROM_LEFT_FOC_BASE, &left);
    load_foc_words(w, EEPROM_RIGHT_FOC_BASE, &right);
    load_pos_words(w, EEPROM_LEFT_POS_BASE, &pos_left);
    load_pos_words(w, EEPROM_RIGHT_POS_BASE, &pos_right);
    pos_left.deadband_ticks = w[EEPROM_LEFT_FOC_BASE + 9U];
    pos_right.deadband_ticks = w[EEPROM_RIGHT_FOC_BASE + 9U];

    const uint16_t loaded_rate = w[EEPROM_WORD_TELEM_RATE];
    const uint8_t loaded_page = (uint8_t)w[EEPROM_WORD_TELEM_PAGE];
    if (!persistent_config_valid(&left, &right, &pos_left, &pos_right,
                                 w[EEPROM_WORD_MAX_CURRENT], w[EEPROM_WORD_MAX_SPEED],
                                 loaded_rate, loaded_page)) {
        return false;
    }

    /* Commit atomik pada konteks main-loop setelah seluruh image lolos validasi. */
    motorParamsLeft = left;
    motorParamsRight = right;
    positionPidConfigLeft = pos_left;
    positionPidConfigRight = pos_right;
    PositionPid_Reset(&positionPidStateLeft);
    PositionPid_Reset(&positionPidStateRight);
    telemetryRateHz = loaded_rate;
    telemetryPage = loaded_page;

    eepromStoredCrc = w[EEPROM_WORD_CRC];
    eepromGeneration = w[EEPROM_WORD_GENERATION];
    eepromVerified = true;
    settingsDirty = false;
    return true;
}

/* ---------------------------- TELEMETRY -------------------------------- */
#define TELEM_ENABLED(bit) ((telemetryMask & (1UL << (bit))) != 0UL)

static int32_t gain_u16_to_q16(uint16_t gain)
{
    /* GUI membatasi gain raw FOC ke 0..32767 agar representasi Q16.16 tetap signed-int32 aman. */
    if (gain > 32767U) return INT32_MAX;
    return (int32_t)((uint32_t)gain << 16);
}

static void fill_basic(uint8_t payload[46])
{
    EscTelemetryBasic t;
    memset(&t, 0, sizeof(t));
    /* Posisi telemetry selalu memakai koordinat mekanik host yang sama dengan
     * setpoint. Motor kanan bercermin di hardware, sehingga odom_r dinormalisasi. */
    if (TELEM_ENABLED(0))  t.position_left = MOTOR_COMMAND_SIGN_LEFT * odom_l;
    if (TELEM_ENABLED(1))  t.position_right = MOTOR_COMMAND_SIGN_RIGHT * odom_r;
    if (TELEM_ENABLED(2))  t.setpoint_left = runtimeSetpointLeft;
    if (TELEM_ENABLED(3))  t.setpoint_right = runtimeSetpointRight;
    if (TELEM_ENABLED(4))  t.speed_left = (int16_t)(MOTOR_COMMAND_SIGN_LEFT * motorOutputLeft.motorSpeed);
    if (TELEM_ENABLED(5))  t.speed_right = (int16_t)(MOTOR_COMMAND_SIGN_RIGHT * motorOutputRight.motorSpeed);
    if (TELEM_ENABLED(6))  t.battery_centi_volt = batVoltageCalib;
    if (TELEM_ENABLED(7))  t.temperature_deci_c = board_temp_deci_c;
    if (TELEM_ENABLED(8))  t.dc_current_left_centi_amp = left_dc_curr;
    if (TELEM_ENABLED(9))  t.dc_current_right_centi_amp = right_dc_curr;
    if (TELEM_ENABLED(10)) t.command_left = runtimeCommandLeft;
    if (TELEM_ENABLED(11)) t.command_right = runtimeCommandRight;
    if (TELEM_ENABLED(12)) t.link_age_ms = RuntimeControl_LinkAgeMs();
    if (TELEM_ENABLED(13)) t.telemetry_rate_hz = telemetryRateHz;
    memcpy(payload, &t, sizeof(t));
}

static void fill_foc(uint8_t payload[46])
{
    EscTelemetryFoc t;
    memset(&t, 0, sizeof(t));
    if (TELEM_ENABLED(0))  t.id_left = motorOutputLeft.currentD;
    if (TELEM_ENABLED(1))  t.iq_left = motorOutputLeft.currentQ;
    if (TELEM_ENABLED(2))  t.id_right = motorOutputRight.currentD;
    if (TELEM_ENABLED(3))  t.iq_right = motorOutputRight.currentQ;
    if (TELEM_ENABLED(4))  t.phase_a_left = curL_phaA;
    if (TELEM_ENABLED(5))  t.phase_b_left = curL_phaB;
    if (TELEM_ENABLED(6))  t.phase_b_right = curR_phaB;
    if (TELEM_ENABLED(7))  t.phase_c_right = curR_phaC;
    if (TELEM_ENABLED(8))  t.dc_link_left = curL_DC;
    if (TELEM_ENABLED(9))  t.dc_link_right = curR_DC;
    if (TELEM_ENABLED(10)) t.duty_u_left = motorOutputLeft.dutyPhaseA;
    if (TELEM_ENABLED(11)) t.duty_v_left = motorOutputLeft.dutyPhaseB;
    if (TELEM_ENABLED(12)) t.duty_w_left = motorOutputLeft.dutyPhaseC;
    if (TELEM_ENABLED(13)) t.duty_u_right = motorOutputRight.dutyPhaseA;
    if (TELEM_ENABLED(14)) t.duty_v_right = motorOutputRight.dutyPhaseB;
    if (TELEM_ENABLED(15)) t.duty_w_right = motorOutputRight.dutyPhaseC;
    if (TELEM_ENABLED(16)) t.electrical_angle_left = motorOutputLeft.electricalAngle;
    if (TELEM_ENABLED(17)) t.electrical_angle_right = motorOutputRight.electricalAngle;
    if (TELEM_ENABLED(18)) t.speed_left = (int16_t)(MOTOR_COMMAND_SIGN_LEFT * motorOutputLeft.motorSpeed);
    if (TELEM_ENABLED(19)) t.speed_right = (int16_t)(MOTOR_COMMAND_SIGN_RIGHT * motorOutputRight.motorSpeed);
    if (TELEM_ENABLED(20)) {
        t.hall_bits=(uint16_t)((motorInputLeft.hallA<<5)|(motorInputLeft.hallB<<4)|(motorInputLeft.hallC<<3)|
                               (motorInputRight.hallA<<2)|(motorInputRight.hallB<<1)|motorInputRight.hallC);
    }
    memcpy(payload, &t, sizeof(t));
}

static void pid_values_for_motor(bool left, uint8_t loop, int32_t *setpoint, int32_t *measured,
                                 int32_t *error, int16_t *p, int16_t *i, int16_t *d, int16_t *output,
                                 bool *antiwindup)
{
    MotorControlState *state = left ? &motorStateLeft : &motorStateRight;
    MotorOutput *out = left ? &motorOutputLeft : &motorOutputRight;
    int32_t pos = left ? odom_l : odom_r;
    int32_t host_sp = left ? runtimeSetpointLeft : runtimeSetpointRight;
    const int32_t mechanical_sign = left ? MOTOR_COMMAND_SIGN_LEFT : MOTOR_COMMAND_SIGN_RIGHT;

    if (loop == ESC_PID_POSITION) {
        PositionPidState *ps = left ? &positionPidStateLeft : &positionPidStateRight;
        *setpoint = host_sp;
        *measured = mechanical_sign * pos;
        *error = ps->last_error;
        *p = ps->last_p;
        *i = ps->last_i;
        *d = ps->last_d;
        *output = left ? runtimeCommandLeft : runtimeCommandRight;
        *antiwindup = ps->antiwindup_active;
    } else if (loop == ESC_PID_SPEED) {
        /* Ubah data inner FOC kembali ke koordinat mekanik host agar kiri/kanan comparable. */
        *measured = mechanical_sign * out->motorSpeed;
        *error = mechanical_sign * state->speedPid.previousError;
        *setpoint = *measured + *error;
        *p = (int16_t)(mechanical_sign * state->speedPid.lastP);
        *i = (int16_t)(mechanical_sign * state->speedPid.lastI);
        *d = (int16_t)(mechanical_sign * state->speedPid.lastD);
        *output = (int16_t)(mechanical_sign * state->speedPid.lastOutput);
        *antiwindup = state->speedPid.previousValue1;
    } else if (loop == ESC_PID_CURRENT_D) {
        *measured = out->currentD;
        *error = state->currentPid.previousError;
        *setpoint = *measured + *error;
        *p = state->currentPid.lastP;
        *i = state->currentPid.lastI;
        *d = state->currentPid.lastD;
        *output = state->currentPid.lastOutput;
        *antiwindup = state->currentPid.previousValue1;
    } else { /* ESC_PID_TORQUE_Q */
        *measured = mechanical_sign * out->currentQ;
        *error = mechanical_sign * state->torquePid.previousError;
        *setpoint = *measured + *error;
        *p = (int16_t)(mechanical_sign * state->torquePid.lastP);
        *i = (int16_t)(mechanical_sign * state->torquePid.lastI);
        *d = (int16_t)(mechanical_sign * state->torquePid.lastD);
        *output = (int16_t)(mechanical_sign * state->torquePid.lastOutput);
        *antiwindup = state->torquePid.previousValue1;
    }
}

static void fill_pid(uint8_t payload[46])
{
    EscTelemetryPid t;
    memset(&t, 0, sizeof(t));
    bool aw_l=false, aw_r=false;
    int32_t sp_l=0,sp_r=0,meas_l=0,meas_r=0,err_l=0,err_r=0;
    int16_t p_l=0,i_l=0,d_l=0,out_l=0,p_r=0,i_r=0,d_r=0,out_r=0;
    t.loop=telemetryPidLoop;
    pid_values_for_motor(true,telemetryPidLoop,&sp_l,&meas_l,&err_l,&p_l,&i_l,&d_l,&out_l,&aw_l);
    pid_values_for_motor(false,telemetryPidLoop,&sp_r,&meas_r,&err_r,&p_r,&i_r,&d_r,&out_r,&aw_r);
    if (TELEM_ENABLED(0))  t.setpoint_left=sp_l;
    if (TELEM_ENABLED(1))  t.setpoint_right=sp_r;
    if (TELEM_ENABLED(2))  t.measured_left=meas_l;
    if (TELEM_ENABLED(3))  t.measured_right=meas_r;
    if (TELEM_ENABLED(4))  t.error_left=err_l;
    if (TELEM_ENABLED(5))  t.error_right=err_r;
    if (TELEM_ENABLED(6))  t.p_left=p_l;
    if (TELEM_ENABLED(7))  t.i_left=i_l;
    if (TELEM_ENABLED(8))  t.d_left=d_l;
    if (TELEM_ENABLED(9))  t.output_left=out_l;
    if (TELEM_ENABLED(10)) t.p_right=p_r;
    if (TELEM_ENABLED(11)) t.i_right=i_r;
    if (TELEM_ENABLED(12)) t.d_right=d_r;
    if (TELEM_ENABLED(13)) t.output_right=out_r;
    if (TELEM_ENABLED(14)) t.antiwindup_flags=(aw_l?1U:0U)|(aw_r?2U:0U);
    memcpy(payload,&t,sizeof(t));
}

static void fill_raw(uint8_t payload[46])
{
    EscTelemetryRaw t;
    memset(&t,0,sizeof(t));
    if (TELEM_ENABLED(0))  t.adc_phase_a_left=adc_buffer.rlA;
    if (TELEM_ENABLED(1))  t.adc_phase_b_left=adc_buffer.rlB;
    if (TELEM_ENABLED(2))  t.adc_phase_b_right=adc_buffer.rrB;
    if (TELEM_ENABLED(3))  t.adc_phase_c_right=adc_buffer.rrC;
    if (TELEM_ENABLED(4))  t.adc_dc_left=adc_buffer.dcl;
    if (TELEM_ENABLED(5))  t.adc_dc_right=adc_buffer.dcr;
    if (TELEM_ENABLED(6))  t.adc_battery=adc_buffer.batt1;
    if (TELEM_ENABLED(7))  t.adc_temperature=adc_buffer.temp;
    if (TELEM_ENABLED(8))  t.hall_left=(uint8_t)((motorInputLeft.hallA<<2)|(motorInputLeft.hallB<<1)|motorInputLeft.hallC);
    if (TELEM_ENABLED(9))  t.hall_right=(uint8_t)((motorInputRight.hallA<<2)|(motorInputRight.hallB<<1)|motorInputRight.hallC);
    if (TELEM_ENABLED(10)) t.pwm_period=2000U;
    if (TELEM_ENABLED(11)) t.main_loop_counter=main_loop_counter;
    if (TELEM_ENABLED(12)) t.valid_frames=EscProtocol_GetValidFrameCount();
    if (TELEM_ENABLED(13)) t.bad_frames=EscProtocol_GetBadFrameCount();
    if (TELEM_ENABLED(14)) t.reconnect_counter=reconnectCounter;
    t.core_speed_left = motorOutputLeft.motorSpeed;
    t.core_speed_right = motorOutputRight.motorSpeed;
    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    memcpy(payload,&t,sizeof(t));
}

static void fill_config(uint8_t payload[46])
{
    EscTelemetryConfig t;
    memset(&t, 0, sizeof(t));
    const bool left = configMotor != ESC_MOTOR_RIGHT;
    MotorParameters *params = left ? &motorParamsLeft : &motorParamsRight;
    PositionPidConfig *pos = left ? &positionPidConfigLeft : &positionPidConfigRight;
    t.motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    t.loop = telemetryPidLoop;
    t.position_min = pos->position_min;
    t.position_max = pos->position_max;

    if (telemetryPidLoop == ESC_PID_POSITION) {
        t.kp_q16 = pos->kp_q16;
        t.ki_q16 = pos->ki_q16;
        t.kd_q16 = pos->kd_q16;
        {
            int64_t ilim = (int64_t)pos->integral_limit << 16;
            if (ilim > INT32_MAX) ilim = INT32_MAX;
            t.i_limit_q16 = (int32_t)ilim;
        }
        t.output_min = pos->output_min;
        t.output_max = pos->output_max;
    } else if (telemetryPidLoop == ESC_PID_CURRENT_D) {
        t.kp_q16 = gain_u16_to_q16(params->currentDKp);
        t.ki_q16 = gain_u16_to_q16(params->currentDKi);
        t.kd_q16 = gain_u16_to_q16(params->currentDKd);
        t.output_min = INT16_MIN; t.output_max = INT16_MAX;
    } else if (telemetryPidLoop == ESC_PID_SPEED) {
        t.kp_q16 = gain_u16_to_q16(params->speedKp);
        t.ki_q16 = gain_u16_to_q16(params->speedKi);
        t.kd_q16 = gain_u16_to_q16(params->speedKd);
        t.output_min = INT16_MIN; t.output_max = INT16_MAX;
    } else { /* ESC_PID_TORQUE_Q */
        t.kp_q16 = gain_u16_to_q16(params->currentQKp);
        t.ki_q16 = gain_u16_to_q16(params->currentQKi);
        t.kd_q16 = gain_u16_to_q16(params->currentQKd);
        t.output_min = INT16_MIN; t.output_max = INT16_MAX;
    }
    t.position_deadband_ticks = pos->deadband_ticks;
    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    t.eeprom_verified = eepromVerified ? 1U : 0U;
    t.settings_dirty = settingsDirty ? 1U : 0U;
    memcpy(payload, &t, sizeof(t));
}

void RuntimeControl_ServiceTelemetry(void)
{
    uint32_t now=HAL_GetTick();
    uint32_t period=1000U/(telemetryRateHz?telemetryRateHz:1U);
    if ((now-lastTelemetryTick)<period) return;
    lastTelemetryTick=now;

    EscFeedbackFrame f;
    memset(&f,0,sizeof(f));
    f.start=ESC_FRAME_START; f.version=ESC_PROTOCOL_VERSION; f.type=ESC_MSG_TELEMETRY;
    f.sequence=++feedbackSequence; f.page=telemetryPage; f.uptime_ms=now;
    f.mode_left=requestedModeLeft; f.mode_right=requestedModeRight;
    f.error_left=motorOutputLeft.errorCode; f.error_right=motorOutputRight.errorCode;
    if (linkActive) f.status|=ESC_STATUS_LINK_OK;
    if (armed) f.status|=ESC_STATUS_ARMED;
    if (eepromOk) f.status|=ESC_STATUS_EEPROM_OK;
    if (eepromVerified) f.status|=ESC_STATUS_EEPROM_VERIFIED;
    if (settingsDirty) f.status|=ESC_STATUS_SETTINGS_DIRTY;
    if (!linkActive) f.status|=ESC_STATUS_TIMEOUT;

    switch (telemetryPage) {
        case ESC_TELEM_FOC: fill_foc(f.payload); break;
        case ESC_TELEM_PID: fill_pid(f.payload); break;
        case ESC_TELEM_RAW: fill_raw(f.payload); break;
        case ESC_TELEM_CONFIG: fill_config(f.payload); break;
        default: fill_basic(f.payload); break;
    }
    f.checksum=EscProtocol_Crc16((const uint8_t*)&f,sizeof(f)-sizeof(f.checksum));
    if (EscProtocol_SendFeedback(&f) && configOneShotPending && telemetryPage == ESC_TELEM_CONFIG) {
        telemetryPage = telemetryPageBeforeConfig;
        configOneShotPending = false;
    }
}

#undef TELEM_ENABLED
