#ifndef SERVO_LS_H
#define SERVO_LS_H

#include <Arduino.h>

// Pin definitions
static const uint8_t LS_UP_PIN    = PB6;   // Limit Switch Atas (batas atas fork)
static const uint8_t LS_DOWN_PIN  = PB7;   // Limit Switch Bawah (pengaman batas bawah)
static const uint8_t SERVO_PIN    = PB9;   // Servo signal — TIM4_CH4 hardware PWM
static const uint8_t LED_BUILTIN_PIN = PC13; // LED onboard (active LOW)

// Servo pulse calibration (microseconds) @ 50 Hz
static const uint16_t SERVO_PULSE_MIN_US = 500;   // 0°
static const uint16_t SERVO_PULSE_MAX_US = 2500;  // 180°
static const uint32_t SERVO_PWM_FREQ_HZ  = 50;    // 50 Hz -> 20 ms period

// Debounce delay (ms)
static const uint32_t DEBOUNCE_MS = 50;

void ServoLS_Init(void);
void ServoLS_Loop(void);
void Servo_SetAngle(uint16_t angle);

#endif // SERVO_LS_H