// ============================================================================
// SERVO TEST (NADIA)
// Board   : Black Pill STM32F411CEU6 (WeAct V2.0)
// Servo   : Signal = PB9 (TIM11_CH1 ALT1)
// ============================================================================

#include <Arduino.h>

#include "servo_test.h"

// ============================================================================
// ARDUINO ENTRY
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println(F("NADIA Servo Test — boot"));

  Servo_Init();

  // Test sequence: 0° -> wait 2s -> 180° -> hold 5s
  Servo_Test();
}

void loop() {
  // Servo stays at 180° after test
  delay(1000);
}