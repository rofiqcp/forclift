#include "servo_ls.h"

static bool   lsUpLastState   = HIGH;
static bool   lsDownLastState = HIGH;
static bool   lsInitialized   = false;
static uint16_t currentAngle  = 0;   // 0-180
static uint16_t targetAngle   = 0;   // target for smooth movement

// Convert angle (0-180) to PWM ticks for 16-bit resolution @ 50 Hz (20ms period)
static inline uint16_t angleToTicks(uint16_t angle) {
  if (angle > 180) angle = 180;
  // pulse = SERVO_PULSE_MIN_US + (angle/180) * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US)
  uint32_t pulse_us = SERVO_PULSE_MIN_US + (uint32_t)angle * (SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 180;
  // ticks = pulse_us / 20000 * 65535
  return (uint16_t)(pulse_us * 65535UL / 20000UL);
}

// Smooth move: ramp toward targetAngle
static void Servo_UpdateRamp(void) {
  if (currentAngle == targetAngle) return;

  if (currentAngle < targetAngle) {
    currentAngle += 2;  // step size per loop
    if (currentAngle > targetAngle) currentAngle = targetAngle;
  } else {
    if (currentAngle <= 2) currentAngle = 0;
    else currentAngle -= 2;
    if (currentAngle < targetAngle) currentAngle = targetAngle;
  }
  analogWrite(SERVO_PIN, angleToTicks(currentAngle));
}

void ServoLS_Init(void) {
  // LS inputs: PB6, PB7 -> GND, INPUT_PULLUP
  pinMode(LS_UP_PIN, INPUT_PULLUP);
  lsUpLastState = digitalRead(LS_UP_PIN);
  pinMode(LS_DOWN_PIN, INPUT_PULLUP);
  lsDownLastState = digitalRead(LS_DOWN_PIN);

  // LED onboard (active LOW)
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, HIGH);  // OFF

  // Servo hardware PWM on PB9 (TIM4_CH4)
  pinMode(SERVO_PIN, OUTPUT);
  analogWriteFrequency(SERVO_PWM_FREQ_HZ);  // 50 Hz
  analogWriteResolution(16);                 // 16-bit (0-65535)

  // Start at 0°
  currentAngle = 0;
  targetAngle = 0;
  analogWrite(SERVO_PIN, angleToTicks(0));

  lsInitialized = true;

  Serial.println(F("SERVO LS TEST START (HW PWM)"));
  Serial.println(F("LS UP: RELEASED"));
  Serial.println(F("LS DOWN: RELEASED"));
  Serial.println(F("LED: OFF"));
  Serial.println(F("SERVO: 0 DEG"));
}

// Call this in loop for smooth movement
void ServoLS_Loop(void) {
  if (!lsInitialized) return;

  // --- LS_UP (PB6): batas atas fork ---
  bool lsUpNow = digitalRead(LS_UP_PIN);
  if (lsUpNow != lsUpLastState) {
    delay(DEBOUNCE_MS);
    lsUpNow = digitalRead(LS_UP_PIN);
    if (lsUpNow != lsUpLastState) {
      lsUpLastState = lsUpNow;

      if (lsUpNow == LOW) {
        // Triggered: fork naik ke batas atas -> servo 180°
        Serial.println(F("LS UP: TRIGGERED (BATAS ATAS)"));
        digitalWrite(LED_BUILTIN_PIN, LOW);   // LED ON
        Serial.println(F("LED: ON"));
        targetAngle = 180;
        Serial.println(F("SERVO: TARGET 180 DEG"));
      } else {
        // Released: kembali ke posisi awal
        Serial.println(F("LS UP: RELEASED"));
        digitalWrite(LED_BUILTIN_PIN, HIGH);  // LED OFF
        Serial.println(F("LED: OFF"));
        targetAngle = 0;
        Serial.println(F("SERVO: TARGET 0 DEG"));
      }
    }
  }

  // --- LS_DOWN (PB7): batas bawah fork (pengaman) ---
  bool lsDownNow = digitalRead(LS_DOWN_PIN);
  if (lsDownNow != lsDownLastState) {
    delay(DEBOUNCE_MS);
    lsDownNow = digitalRead(LS_DOWN_PIN);
    if (lsDownNow != lsDownLastState) {
      lsDownLastState = lsDownNow;

      if (lsDownNow == LOW) {
        // Triggered: fork di batas bawah -> servo 0°
        Serial.println(F("LS DOWN: TRIGGERED (BATAS BAWAH)"));
        digitalWrite(LED_BUILTIN_PIN, HIGH);  // LED OFF
        Serial.println(F("LED: OFF"));
        targetAngle = 0;
        Serial.println(F("SERVO: TARGET 0 DEG (SAFETY LOWER LIMIT)"));
      } else {
        Serial.println(F("LS DOWN: RELEASED"));
      }
    }
  }

  // Smooth movement update
  Servo_UpdateRamp();
}