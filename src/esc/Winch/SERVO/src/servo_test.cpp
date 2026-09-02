#include "servo_test.h"

// Servo object
static Servo servo;
static bool servoInitialized = false;

void Servo_Init(void) {
  // Attach servo to PB9
  // attach(pin, min_pulse_us, max_pulse_us, initial_angle)
  servo.attach(SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE, 0);

  // Set initial position to 0°
  servo.write(0);

  servoInitialized = true;

  Serial.println(F("[SERVO] Initialization complete"));
  Serial.println(F("[SERVO] Position = 0 degree"));
}

void Servo_SetAngle(uint16_t angle) {
  if (!servoInitialized) return;

  // Clamp angle to 0-180
  if (angle > 180) {
    angle = 180;
  }

  servo.write(angle);

  Serial.print(F("[SERVO] Position = "));
  Serial.print(angle);
  Serial.println(F(" degree"));
}

void Servo_Test(void) {
  if (!servoInitialized) return;

  Serial.println(F("\n========================================"));
  Serial.println(F("  SERVO TEST SEQUENCE"));
  Serial.println(F("========================================\n"));

  // Step 1: Go to 0°
  Serial.println(F("[TEST] Step 1: Go to 0 degree"));
  Servo_SetAngle(0);
  Serial.println(F("[SERVO] Waiting 2 seconds..."));
  delay(2000);

  // Step 2: Move to 180° with smooth motion
  Serial.println(F("\n[TEST] Step 2: Moving to 180 degree (smooth)..."));
  for (uint16_t angle = 0; angle <= 180; angle += 5) {
    servo.write(angle);
    delay(20);
  }
  Servo_SetAngle(180);

  // Step 3: Hold at 180° for 5 seconds
  Serial.println(F("[SERVO] Holding 180 degree for 5 seconds"));
  delay(5000);

  Serial.println(F("\n========================================"));
  Serial.println(F("  SERVO TEST COMPLETE"));
  Serial.println(F("  Servo holding at 180 degree"));
  Serial.println(F("========================================\n"));
}