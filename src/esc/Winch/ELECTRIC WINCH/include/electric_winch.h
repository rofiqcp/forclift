#ifndef ELECTRIC_WINCH_H
#define ELECTRIC_WINCH_H

#include <Arduino.h>

// ============================================================================
// PIN DEFINITIONS
// ============================================================================
// BTS7960 Motor Driver
// RPWM = PA2 (Winch UP)
// LPWM = PB8 (Winch DOWN)
// REN, LEN tied to 5V (not controlled by STM32)
static const uint8_t WINCH_PIN_RPWM = PA2;
static const uint8_t WINCH_PIN_LPWM = PB8;

// Limit Switches (Active LOW with INPUT_PULLUP)
// LS ATAS (Top)   = PB6
// LS BAWAH (Bottom) = PB7
static const uint8_t WINCH_PIN_LS_TOP    = PB6;
static const uint8_t WINCH_PIN_LS_BOTTOM = PB7;

// Servo
// SERVO DATA = PB9
static const uint8_t WINCH_PIN_SERVO = PB9;

// Serial Port: use the board's USB CDC connection so the Windows COM port matches
// the GUI/terminal control flow. If you need UART instead, switch back to Serial1.
#define WINCH_SERIAL Serial
#define WINCH_SERIAL_BAUD 115200

// ============================================================================
// CONFIGURATION CONSTANTS - Easy to modify
// ============================================================================
// PWM Configuration
static const uint32_t WINCH_PWM_FREQUENCY   = 1000;  // 1 kHz
static const uint8_t  WINCH_PWM_RESOLUTION  = 8;     // 8-bit (0-255)

// Trial PWM Settings (safe starting values)
static const uint8_t  WINCH_PWM_MAX         = 100;   // ~39% duty cycle (100/255)
static const uint8_t  WINCH_PWM_START       = 30;    // ~12% duty cycle for soft-start

// Soft Start/Stop Ramp Parameters
static const uint8_t  PWM_STEP              = 5;     // PWM increment/decrement per step
static const uint32_t RAMP_INTERVAL_MS      = 50;    // ms between ramp steps

// Direction Change Protection
static const uint32_t WINCH_DIR_CHANGE_DELAY_MS = 100;  // Dead time when changing direction

// Limit Switch Configuration
#define LIMIT_ACTIVE LOW          // LOW = triggered (switch to GND with INPUT_PULLUP)
static const uint32_t DEBOUNCE_MS = 30;  // Software debounce time

// Auto-Return Sequence
static const uint32_t AUTO_RETURN_DELAY_MS = 400;  // Delay after top limit before auto-down

// Servo Configuration
static const uint32_t SERVO_PWM_FREQUENCY = 50;   // 50 Hz
static const uint16_t SERVO_MIN_PULSE     = 1000; // 1.0 ms = 0°
static const uint16_t SERVO_MAX_PULSE     = 2000; // 2.0 ms = 180°

// Servo Angles
// NOTE: the stock Servo.write() clamps to 0-180°, so to reach 195° we drive
// the pin with writeMicroseconds() and an extended attach range.
static const uint16_t SERVO_HOME_ANGLE  = 0;    // Home position (0°)
static const uint16_t SERVO_TOP_ANGLE   = 195;  // Top limit position (195°)
// Pulse widths (us) used by writeMicroseconds()
static const uint16_t SERVO_HOME_US     = 1000; // 0°
static const uint16_t SERVO_TOP_US      = 2080; // ~195° (extended range)
// Attach range must exceed SERVO_TOP_US so the library does not clamp it
static const uint16_t SERVO_ATTACH_MIN  = 1000;
static const uint16_t SERVO_ATTACH_MAX  = 2100;

// Servo Smooth Movement
static const uint16_t SERVO_STEP_DELAY_MS = 15; // ms between each step
static const uint16_t SERVO_STEP_ANGLE    = 5;  // degrees per step

// ============================================================================
// STATE MACHINE
// ============================================================================
enum WinchState {
    WINCH_STOPPED        = 0,  // Idle, waiting for command
    WINCH_UP             = 1,  // Moving UP (soft start -> running)
    WINCH_DOWN           = 2,  // Moving DOWN (soft start -> running)
    WINCH_UP_STOPPING    = 3,  // Soft stop from UP (ramp down)
    WINCH_DOWN_STOPPING  = 4,  // Soft stop from DOWN (ramp down)
    WINCH_AUTO_RETURN    = 5,  // Auto return DOWN after top limit hit
    WINCH_AUTO_DELAY     = 6   // Brief delay between stop and auto-down
};

// ============================================================================
// PUBLIC API
// ============================================================================
void Winch_Init(void);
void Winch_Update(void);           // Call repeatedly in loop() - non-blocking
void Winch_ProcessCommand(const String &command);

// Direct control (used by command processor)
void Winch_Up(void);
void Winch_Down(void);
void Winch_Stop(void);
void Winch_EmergencyStop(void);

// Servo functions
void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);
void Servo_MoveSmooth(uint16_t targetAngle);
void Servo_Update(void);  // Non-blocking update for smooth movement

// Status getters
WinchState Winch_GetState(void);
uint8_t    Winch_GetCurrentPWM(void);
bool       Winch_IsTopLimitActive(void);
bool       Winch_IsBottomLimitActive(void);
bool       Winch_IsMoving(void);
const char* Winch_GetStateName(WinchState state);
uint16_t   Servo_GetCurrentAngle(void);
uint16_t   Servo_GetTargetAngle(void);
bool       Servo_IsMoving(void);

// Limit switch helpers
bool Winch_ReadTopLimitRaw(void);
bool Winch_ReadBottomLimitRaw(void);

#endif // ELECTRIC_WINCH_H