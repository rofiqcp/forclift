// ============================================================================
// servo_ls TRIAL — Limit Switch Atas + LED Onboard + Servo
// Board   : Black Pill STM32F411CEU6 (WeAct V2.0)
// LS_UP   : PB6 (INPUT_PULLUP, LOW = triggered)
// Servo   : PB9 (TIM11_CH1 via Servo library)
// LED     : PC13 (LED_BUILTIN, active LOW)
// ============================================================================

#include <Arduino.h>

#include "servo_ls.h"

void setup() {
  Serial.begin(115200);
  delay(50);

  ServoLS_Init();
}

void loop() {
  ServoLS_Loop();
}