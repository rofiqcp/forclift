#ifndef SERVO_TEST_H
#define SERVO_TEST_H

#include <Arduino.h>
#include <Servo.h>

// Pin definitions
static const uint8_t SERVO_PIN = PB9;

// PWM configuration for standard servo
static const uint32_t SERVO_PWM_FREQUENCY = 50;   // 50 Hz
static const uint16_t SERVO_MIN_PULSE = 1000;     // 1.0 ms = 0°
static const uint16_t SERVO_MAX_PULSE = 2000;     // 2.0 ms = 180°

void Servo_Init(void);
void Servo_SetAngle(uint16_t angle);
void Servo_Test(void);

#endif // SERVO_TEST_H