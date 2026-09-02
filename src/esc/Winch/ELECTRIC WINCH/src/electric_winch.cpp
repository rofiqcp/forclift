#include "electric_winch.h"
#include <Servo.h>

// ============================================================================
// INTERNAL STATE VARIABLES
// ============================================================================
static bool winchInitialized = false;
static WinchState currentState = WINCH_STOPPED;

// PWM Control
static uint8_t currentPWM = 0;
static uint8_t targetPWM = 0;
static uint32_t lastRampTime = 0;
static int8_t rampDirection = 0;  // 1 = ramping up, -1 = ramping down, 0 = steady

// Direction tracking
static int8_t currentDirection = 0;  // 1 = UP, -1 = DOWN, 0 = STOP
static uint32_t lastStopTime = 0;

// Limit Switch Debounce
static bool lastTopLimitState = HIGH;
static bool lastBottomLimitState = HIGH;
static uint32_t lastTopLimitChange = 0;
static uint32_t lastBottomLimitChange = 0;
static bool debouncedTopLimit = false;
static bool debouncedBottomLimit = false;

// Auto-return sequence
static bool autoReturnActive = false;
static uint32_t autoReturnDelayStart = 0;

// Previous limit states for edge detection
static bool prevTopLimit = false;
static bool prevBottomLimit = false;

// Direction reversal config (easy to swap if motor polarity is reversed)
static const bool SWAP_DIRECTION = true;   // Set to true if UP/DOWN are reversed

// Servo
static Servo servo;
static bool servoInitialized = false;
static uint16_t servoCurrentAngle = SERVO_HOME_ANGLE;
static uint16_t servoTargetAngle = SERVO_HOME_ANGLE;
static bool servoMoving = false;
static uint32_t servoLastStepTime = 0;
static int16_t servoStepDirection = 0;

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
static inline uint8_t pctToPWM(uint8_t pct) {
    if (pct > 100) pct = 100;
    return (uint8_t)((pct * 255UL) / 100UL);
}

static inline uint8_t pwmToPct(uint8_t pwm) {
    return (uint8_t)((pwm * 100UL) / 255UL);
}

// Set PWM with safety interlock (never both channels active)
static void setMotorPWM(uint8_t rpwm, uint8_t lpwm) {
    // Safety interlock
    if (rpwm > 0 && lpwm > 0) {
        rpwm = 0;
        lpwm = 0;
    }
    analogWrite(WINCH_PIN_RPWM, rpwm);
    analogWrite(WINCH_PIN_LPWM, lpwm);
}

// Read raw limit switch (with INPUT_PULLUP: HIGH = open, LOW = triggered)
static bool readTopLimitRaw() {
    return digitalRead(WINCH_PIN_LS_TOP) == LIMIT_ACTIVE;
}

static bool readBottomLimitRaw() {
    return digitalRead(WINCH_PIN_LS_BOTTOM) == LIMIT_ACTIVE;
}

// Debounced limit switch reading
static void updateLimitSwitches() {
    uint32_t now = millis();
    
    // Top limit
    bool rawTop = readTopLimitRaw();
    if (rawTop != lastTopLimitState) {
        lastTopLimitChange = now;
        lastTopLimitState = rawTop;
    }
    if (now - lastTopLimitChange >= DEBOUNCE_MS) {
        debouncedTopLimit = rawTop;
    }
    
    // Bottom limit
    bool rawBottom = readBottomLimitRaw();
    if (rawBottom != lastBottomLimitState) {
        lastBottomLimitChange = now;
        lastBottomLimitState = rawBottom;
    }
    if (now - lastBottomLimitChange >= DEBOUNCE_MS) {
        debouncedBottomLimit = rawBottom;
    }
}

// Check if we can move UP (top limit not active)
static bool canMoveUp() {
    return !debouncedTopLimit;
}

// Check if we can move DOWN (bottom limit not active)
static bool canMoveDown() {
    return !debouncedBottomLimit;
}

// Apply direction with SWAP_DIRECTION support
static void applyDirection(int8_t dir, uint8_t pwm) {
    int8_t actualDir = SWAP_DIRECTION ? -dir : dir;
    
    if (actualDir == 1) {      // UP
        setMotorPWM(pwm, 0);
    } else if (actualDir == -1) {  // DOWN
        setMotorPWM(0, pwm);
    } else {  // STOP
        setMotorPWM(0, 0);
    }
    // Keep the logical command direction for reversal/dead-time checks.
    // actualDir is only the electrical pin polarity after SWAP_DIRECTION.
    currentDirection = dir;
}

// State transition helper
static void changeState(WinchState newState) {
    if (currentState != newState) {
        WINCH_SERIAL.print(F("[STATE] "));
        WINCH_SERIAL.print(Winch_GetStateName(currentState));
        WINCH_SERIAL.print(F(" -> "));
        WINCH_SERIAL.println(Winch_GetStateName(newState));
        currentState = newState;
    }
}

// ============================================================================
// PUBLIC API IMPLEMENTATION
// ============================================================================

void Winch_Init(void) {
    // Configure motor PWM pins
    pinMode(WINCH_PIN_RPWM, OUTPUT);
    pinMode(WINCH_PIN_LPWM, OUTPUT);
    
    // Configure limit switches with INPUT_PULLUP
    pinMode(WINCH_PIN_LS_TOP, INPUT_PULLUP);
    pinMode(WINCH_PIN_LS_BOTTOM, INPUT_PULLUP);
    
    // PWM setup
    analogWriteFrequency(WINCH_PWM_FREQUENCY);
    analogWriteResolution(WINCH_PWM_RESOLUTION);
    
    // Ensure stopped on boot
    setMotorPWM(0, 0);
    
    // Initialize state
    currentState = WINCH_STOPPED;
    currentPWM = 0;
    targetPWM = 0;
    rampDirection = 0;
    currentDirection = 0;
    lastStopTime = millis();
    autoReturnActive = false;
    
    // Initial limit switch read
    lastTopLimitState = readTopLimitRaw();
    lastBottomLimitState = readBottomLimitRaw();
    debouncedTopLimit = lastTopLimitState;
    debouncedBottomLimit = lastBottomLimitState;
    prevTopLimit = debouncedTopLimit;
    prevBottomLimit = debouncedBottomLimit;
    lastTopLimitChange = millis();
    lastBottomLimitChange = millis();

    // Initialize servo at HOME position (PB9)
    Servo_Init();

    winchInitialized = true;

        // Startup banner
        WINCH_SERIAL.println(F("====================================="));
        WINCH_SERIAL.println(F("NADIA ELECTRIC WINCH TEST"));
        WINCH_SERIAL.println(F("STM32F411CEU6"));
        WINCH_SERIAL.println(F("====================================="));
        WINCH_SERIAL.println(F(""));
        WINCH_SERIAL.print(F("RPWM      : ")); WINCH_SERIAL.println(F("PA2"));
        WINCH_SERIAL.print(F("LPWM      : ")); WINCH_SERIAL.println(F("PB8"));
        WINCH_SERIAL.print(F("LS TOP    : ")); WINCH_SERIAL.println(F("PB6"));
        WINCH_SERIAL.print(F("LS BOTTOM : ")); WINCH_SERIAL.println(F("PB7"));
        WINCH_SERIAL.println(F(""));
        WINCH_SERIAL.println(F("[WINCH] READY"));
    }

void Winch_Update(void) {
    if (!winchInitialized) return;

    // Non-blocking servo update (must run every cycle for smooth movement)
    Servo_Update();

    uint32_t now = millis();
    
    // Update debounced limit switches
    updateLimitSwitches();
    
    // ================================================================
    // EDGE DETECTION FOR LIMIT SWITCHES
    // Detect rising edge (CLEAR -> TRIGGERED) to trigger servo + stop motor
    //
    // Wiring note: what the user calls "LS atas" (top) is physically
    // connected to firmware PB7 (bottom), and "LS bawah" (bottom) to PB6 (top).
    // So: firmware TOP    = physical BOTTOM switch -> servo HOME (0°)
    //     firmware BOTTOM = physical TOP    switch -> servo TOP   (195°)
    // ================================================================
    if (debouncedTopLimit && !prevTopLimit) {
        // Firmware TOP (physical LS TOP PB6) limit triggered
        Serial.println(F("[EVENT] TOP LIMIT (PB6) ACTIVATED"));
        // Stop winch if moving
        if (currentState == WINCH_UP || currentState == WINCH_AUTO_RETURN) {
            targetPWM = 0;
            if (currentState == WINCH_UP) {
                changeState(WINCH_UP_STOPPING);
            } else {
                changeState(WINCH_DOWN_STOPPING);
            }
            rampDirection = -1;
            lastRampTime = now;
            autoReturnActive = false;
        }
        // Smooth servo movement to TOP position
        Servo_MoveSmooth(SERVO_TOP_ANGLE);
        Serial.println(F("[TEST SERVO] TOP LIMIT -> SERVO 195"));
    }

    if (debouncedBottomLimit && !prevBottomLimit) {
        // Firmware BOTTOM (physical LS BOTTOM PB7) limit triggered
        Serial.println(F("[EVENT] BOTTOM LIMIT (PB7) ACTIVATED"));
        // Stop winch if moving
        if (currentState == WINCH_DOWN) {
            targetPWM = 0;
            changeState(WINCH_DOWN_STOPPING);
            rampDirection = 0;
            lastRampTime = now;
            autoReturnActive = false;
        }
        // Smooth servo movement to HOME position
        Servo_MoveSmooth(SERVO_HOME_ANGLE);
        Serial.println(F("[TEST SERVO] BOTTOM LIMIT -> SERVO HOME"));
    }
    
    // Update previous states
    prevTopLimit = debouncedTopLimit;
    prevBottomLimit = debouncedBottomLimit;
    
    // Check for both limits active simultaneously (abnormal condition)
    if (debouncedTopLimit && debouncedBottomLimit) {
        // Emergency stop - both limits active
        Serial.println(F("[FAULT] BOTH LIMITS ACTIVE - EMERGENCY STOP"));
        Winch_EmergencyStop();
        // Don't move servo in this fault condition
    }
    
    // ================================================================
    // STATE MACHINE
    // ================================================================
    switch (currentState) {
        case WINCH_STOPPED:
            // Waiting for command, motor is stopped
            currentPWM = 0;
            targetPWM = 0;
            rampDirection = 0;
            autoReturnActive = false;
            break;
            
        case WINCH_UP:
            // Check top limit
            if (debouncedTopLimit) {
                Serial.println(F("[LIMIT] TOP TRIGGERED"));
                Serial.println(F("[WINCH] Stopping UP"));
                changeState(WINCH_UP_STOPPING);
                targetPWM = 0;
                rampDirection = 0;
                break;
            }
            
            // Soft start / ramp up
            if (rampDirection >= 0) {
                if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                    if (currentPWM < targetPWM) {
                        currentPWM += PWM_STEP;
                        if (currentPWM > targetPWM) currentPWM = targetPWM;
                        applyDirection(1, currentPWM);
                        Serial.print(F("[PWM] "));
                        Serial.println(pwmToPct(currentPWM));
                    } else {
                        rampDirection = 0;  // Reached target
                    }
                    lastRampTime = now;
                }
            }
            break;
            
        case WINCH_DOWN:
            // Check bottom limit
            if (debouncedBottomLimit) {
                Serial.println(F("[LIMIT] BOTTOM TRIGGERED"));
                Serial.println(F("[WINCH] Stopping DOWN"));
                changeState(WINCH_DOWN_STOPPING);
                targetPWM = 0;
                rampDirection = -1;
                break;
            }
            
            // Soft start / ramp up
            if (rampDirection >= 0) {
                if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                    if (currentPWM < targetPWM) {
                        currentPWM += PWM_STEP;
                        if (currentPWM > targetPWM) currentPWM = targetPWM;
                        applyDirection(-1, currentPWM);
                        Serial.print(F("[PWM] "));
                        Serial.println(pwmToPct(currentPWM));
                    } else {
                        rampDirection = 0;  // Reached target
                    }
                    lastRampTime = now;
                }
            }
            break;
            
        case WINCH_UP_STOPPING:
            // Ramp down to stop
            if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                if (currentPWM > 0) {
                    if (currentPWM >= PWM_STEP) {
                        currentPWM -= PWM_STEP;
                    } else {
                        currentPWM = 0;
                    }
                    applyDirection(1, currentPWM);
                    Serial.print(F("[PWM] "));
                    Serial.println(pwmToPct(currentPWM));
                } else {
                    // Fully stopped
                    applyDirection(0, 0);
                    lastStopTime = now;
                    
                    // Check if this was part of auto-return sequence
                    if (autoReturnActive) {
                        Serial.println(F("[AUTO] Returning DOWN"));
                        changeState(WINCH_AUTO_DELAY);
                        autoReturnDelayStart = now;
                    } else {
                        changeState(WINCH_STOPPED);
                    }
                }
                lastRampTime = now;
            }
            break;
            
        case WINCH_DOWN_STOPPING:
            // Ramp down to stop
            if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                if (currentPWM > 0) {
                    if (currentPWM >= PWM_STEP) {
                        currentPWM -= PWM_STEP;
                    } else {
                        currentPWM = 0;
                    }
                    applyDirection(-1, currentPWM);
                    Serial.print(F("[PWM] "));
                    Serial.println(pwmToPct(currentPWM));
                } else {
                    // Fully stopped
                    applyDirection(0, 0);
                    lastStopTime = now;
                    changeState(WINCH_STOPPED);
                    autoReturnActive = false;
                    Serial.println(F("[WINCH] STOPPED"));
                }
                lastRampTime = now;
            }
            break;
            
        case WINCH_AUTO_DELAY:
            // Brief delay before auto-down
            if (now - autoReturnDelayStart >= AUTO_RETURN_DELAY_MS) {
                if (canMoveDown()) {
                    Serial.println(F("[AUTO] Soft Start DOWN"));
                    targetPWM = pctToPWM(WINCH_PWM_MAX);
                    currentPWM = pctToPWM(WINCH_PWM_START);
                    applyDirection(-1, currentPWM);
                    rampDirection = 1;
                    lastRampTime = now;
                    changeState(WINCH_AUTO_RETURN);
                } else {
                    // Bottom limit already active - just stop
                    Serial.println(F("[AUTO] Bottom limit already active"));
                    changeState(WINCH_STOPPED);
                    autoReturnActive = false;
                }
            }
            break;
            
        case WINCH_AUTO_RETURN:
            // Auto return DOWN - check bottom limit
            if (debouncedBottomLimit) {
                Serial.println(F("[LIMIT] BOTTOM TRIGGERED"));
                Serial.println(F("[WINCH] Stopping DOWN"));
                changeState(WINCH_DOWN_STOPPING);
                targetPWM = 0;
                rampDirection = -1;
                break;
            }
            
            // Continue ramping up to max PWM
            if (rampDirection > 0) {
                if (now - lastRampTime >= RAMP_INTERVAL_MS) {
                    if (currentPWM < targetPWM) {
                        currentPWM += PWM_STEP;
                        if (currentPWM > targetPWM) currentPWM = targetPWM;
                        applyDirection(-1, currentPWM);
                        Serial.print(F("[PWM] "));
                        Serial.println(pwmToPct(currentPWM));
                    } else {
                        rampDirection = 0;  // Running at full speed
                    }
                    lastRampTime = now;
                }
            }
            break;
    }
    
    // Send periodic status (optional - can be triggered by STATUS command)
    // This is handled by Winch_ProcessCommand for STATUS
}

// Direct control functions (called by command processor)
void Winch_Up(void) {
    if (!winchInitialized) return;
    
    // STOP has highest priority - if already stopping, let it finish
    if (currentState == WINCH_UP_STOPPING || currentState == WINCH_DOWN_STOPPING) {
        return;
    }
    
    // Check top limit
    if (!canMoveUp()) {
        Serial.println(F("[CMD] UP blocked - TOP LIMIT ACTIVE"));
        Serial.println(F("LIMIT_TOP_ACTIVE"));
        return;
    }
    
    // Direction change protection: force both BTS7960 PWM inputs LOW before
    // changing polarity. The old code checked lastStopTime without actually
    // stopping first, so a direct DOWN->UP command could reverse immediately.
    if (currentDirection == -1) {
        setMotorPWM(0, 0);
        currentPWM = 0;
        currentDirection = 0;
        lastStopTime = millis();
        delay(WINCH_DIR_CHANGE_DELAY_MS);
    }
    
    Serial.println(F("[CMD] UP"));
    Serial.println(F("[WINCH] Soft Start UP"));
    
    targetPWM = pctToPWM(WINCH_PWM_MAX);
    currentPWM = pctToPWM(WINCH_PWM_START);
    applyDirection(1, currentPWM);
    rampDirection = 1;
    lastRampTime = millis();
    autoReturnActive = false;
    changeState(WINCH_UP);
}

void Winch_Down(void) {
    if (!winchInitialized) return;
    
    // STOP has highest priority
    if (currentState == WINCH_UP_STOPPING || currentState == WINCH_DOWN_STOPPING) {
        return;
    }
    
    // Check bottom limit
    if (!canMoveDown()) {
        Serial.println(F("[CMD] DOWN blocked - BOTTOM LIMIT ACTIVE"));
        Serial.println(F("LIMIT_BOTTOM_ACTIVE"));
        return;
    }
    
    // Direction change protection: force a real dead-time before reversal.
    if (currentDirection == 1) {
        setMotorPWM(0, 0);
        currentPWM = 0;
        currentDirection = 0;
        lastStopTime = millis();
        delay(WINCH_DIR_CHANGE_DELAY_MS);
    }
    
    // Cancel auto-return if active
    if (autoReturnActive) {
        autoReturnActive = false;
        Serial.println(F("[AUTO] AUTO RETURN CANCELLED by DOWN command"));
    }
    
    Serial.println(F("[CMD] DOWN"));
    Serial.println(F("[WINCH] Soft Start DOWN"));
    
    targetPWM = pctToPWM(WINCH_PWM_MAX);
    currentPWM = pctToPWM(WINCH_PWM_START);
    applyDirection(-1, currentPWM);
    rampDirection = 1;
    lastRampTime = millis();
    changeState(WINCH_DOWN);
}

void Winch_Stop(void) {
    if (!winchInitialized) return;
    
    Serial.println(F("[CMD] STOP"));
    
    // Cancel auto-return
    if (autoReturnActive) {
        autoReturnActive = false;
        Serial.println(F("[AUTO] AUTO RETURN CANCELLED"));
    }
    
    // Immediate stop for safety (or could ramp down)
    targetPWM = 0;
    
    if (currentState == WINCH_UP || currentState == WINCH_AUTO_RETURN) {
        changeState(WINCH_UP_STOPPING);
        rampDirection = -1;
        lastRampTime = millis();
    } else if (currentState == WINCH_DOWN) {
        changeState(WINCH_DOWN_STOPPING);
        rampDirection = -1;
        lastRampTime = millis();
    } else {
        // Already stopped or stopping
        applyDirection(0, 0);
        currentPWM = 0;
        changeState(WINCH_STOPPED);
    }
}

void Winch_EmergencyStop(void) {
    // Immediate hard stop - no ramp, no delay
    setMotorPWM(0, 0);
    currentPWM = 0;
    targetPWM = 0;
    rampDirection = 0;
    currentDirection = 0;
    autoReturnActive = false;
    lastStopTime = millis();
    changeState(WINCH_STOPPED);
    Serial.println(F("[CMD] EMERGENCY/MANUAL STOP"));
    Serial.println(F("[WINCH] STOPPED"));
    Serial.println(F("[AUTO] AUTO RETURN CANCELLED"));
}

// Process serial commands from GUI
void Winch_ProcessCommand(const String &command) {
    String cmd = command;
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "UP") {
        Winch_Up();
    } else if (cmd == "DOWN") {
        Winch_Down();
    } else if (cmd == "STOP") {
        Winch_EmergencyStop();
    } else if (cmd == "STATUS") {
        // Send structured status
        Serial.print(F("STATE:"));
        Serial.println(Winch_GetStateName(currentState));

        Serial.print(F("TOP:"));
        Serial.println(debouncedTopLimit ? 1 : 0);

        Serial.print(F("BOTTOM:"));
        Serial.println(debouncedBottomLimit ? 1 : 0);

        Serial.print(F("PWM:"));
        Serial.println(pwmToPct(currentPWM));

        Serial.print(F("DIR:"));
        Serial.println(currentDirection);

        Serial.print(F("SERVO:"));
        if (servoCurrentAngle == SERVO_HOME_ANGLE) {
            Serial.println(F("HOME"));
        } else if (servoCurrentAngle == SERVO_TOP_ANGLE) {
            Serial.println(F("195"));
        } else {
            Serial.println(servoCurrentAngle);
        }
    } else if (cmd == "LIMITS") {
        // Debug: print raw and debounced limit switch states
        bool rawTop = digitalRead(WINCH_PIN_LS_TOP) == LIMIT_ACTIVE;
        bool rawBot = digitalRead(WINCH_PIN_LS_BOTTOM) == LIMIT_ACTIVE;
        Serial.print(F("[DEBUG] RAW  - TOP:"));
        Serial.print(rawTop ? 1 : 0);
        Serial.print(F("  BOTTOM:"));
        Serial.println(rawBot ? 1 : 0);
        Serial.print(F("[DEBUG] DEB  - TOP:"));
        Serial.print(debouncedTopLimit ? 1 : 0);
        Serial.print(F("  BOTTOM:"));
        Serial.println(debouncedBottomLimit ? 1 : 0);
    } else if (cmd.startsWith("SERVO ")) {
        // Manual servo test: SERVO <angle>
        int angle = cmd.substring(6).toInt();
        if (angle >= 0 && angle <= (int)SERVO_TOP_ANGLE) {
            Servo_SetAngle((uint16_t)angle);
        } else {
            Serial.println(F("[CMD] Usage: SERVO <0-195>"));
        }
    } else if (cmd == "SERVOTEST") {
        // Full servo sweep test: 0 -> 195 -> 0
        Serial.println(F("[SERVO] Sweep test: 0 -> 195 -> 0"));
        Servo_SetAngle(0);
        delay(2000);
        Servo_SetAngle(SERVO_TOP_ANGLE);
        delay(2000);
        Servo_SetAngle(0);
        Serial.println(F("[SERVO] Sweep complete"));
    } else {
        Serial.print(F("[CMD] Unknown: "));
        Serial.println(cmd);
    }
}

// ============================================================================
// STATUS GETTERS
// ============================================================================

WinchState Winch_GetState(void) {
    return currentState;
}

uint8_t Winch_GetCurrentPWM(void) {
    return currentPWM;
}

bool Winch_IsTopLimitActive(void) {
    return debouncedTopLimit;
}

bool Winch_IsBottomLimitActive(void) {
    return debouncedBottomLimit;
}

bool Winch_IsMoving(void) {
    return (currentState == WINCH_UP || currentState == WINCH_DOWN || 
            currentState == WINCH_AUTO_RETURN);
}

const char* Winch_GetStateName(WinchState state) {
    switch (state) {
        case WINCH_STOPPED:       return "STOPPED";
        case WINCH_UP:            return "UP";
        case WINCH_DOWN:          return "DOWN";
        case WINCH_UP_STOPPING:   return "UP_STOPPING";
        case WINCH_DOWN_STOPPING: return "DOWN_STOPPING";
        case WINCH_AUTO_RETURN:   return "AUTO_RETURN";
        case WINCH_AUTO_DELAY:    return "AUTO_DELAY";
        default:                  return "UNKNOWN";
    }
}

bool Winch_ReadTopLimitRaw(void) {
    return readTopLimitRaw();
}

bool Winch_ReadBottomLimitRaw(void) {
    return readBottomLimitRaw();
}

// ============================================================================
// SERVO FUNCTIONS
// ============================================================================

// Map an arbitrary angle (0-195) to a microsecond pulse width.
// Uses linear interpolation between HOME (0°) and TOP (195°).
static uint16_t angleToMicros(uint16_t angle) {
    if (angle <= SERVO_HOME_ANGLE) return SERVO_HOME_US;
    if (angle >= SERVO_TOP_ANGLE) return SERVO_TOP_US;
    // Linear interpolation
    uint32_t range = SERVO_TOP_US - SERVO_HOME_US;
    uint32_t span = SERVO_TOP_ANGLE - SERVO_HOME_ANGLE;
    return (uint16_t)(SERVO_HOME_US + (range * (angle - SERVO_HOME_ANGLE) / span));
}

void Servo_Init(void) {
    // Attach with an extended range so 195° (2080 us) is NOT clamped.
    // The stock Servo.attach clamps min/max to 544-2400, so 2100 is safe.
    servo.attach(WINCH_PIN_SERVO, SERVO_ATTACH_MIN, SERVO_ATTACH_MAX);
    servo.writeMicroseconds(SERVO_HOME_US);
    servoCurrentAngle = SERVO_HOME_ANGLE;
    servoTargetAngle = SERVO_HOME_ANGLE;
    servoMoving = false;
    servoInitialized = true;

    Serial.println(F("[SERVO] Initialization complete"));
    Serial.print(F("[SERVO] Position = "));
    Serial.print(SERVO_HOME_ANGLE);
    Serial.println(F(" degree"));
    Serial.print(F("[SERVO] Pulse range: "));
    Serial.print(SERVO_HOME_US);
    Serial.print(F("-"));
    Serial.print(SERVO_TOP_US);
    Serial.println(F(" us"));
}

// Internal: write a degree angle via writeMicroseconds()
static void servoWriteAngle(uint16_t angle) {
    servo.writeMicroseconds(angleToMicros(angle));
}

void Servo_MoveSmooth(uint16_t targetAngle) {
    if (!servoInitialized) return;

    if (targetAngle > SERVO_TOP_ANGLE) targetAngle = SERVO_TOP_ANGLE;

    if (targetAngle == servoCurrentAngle) {
        servoWriteAngle(targetAngle);  // confirm position
        servoMoving = false;
        return;
    }

    // Drive the first step immediately so the servo ALWAYS starts moving
    int16_t step = (targetAngle > servoCurrentAngle) ? (int16_t)SERVO_STEP_ANGLE : -(int16_t)SERVO_STEP_ANGLE;
    int16_t firstStep = servoCurrentAngle + step;
    if (firstStep < 0) firstStep = 0;
    if (firstStep > (int16_t)SERVO_TOP_ANGLE) firstStep = SERVO_TOP_ANGLE;
    servoWriteAngle((uint16_t)firstStep);
    servoCurrentAngle = (uint16_t)firstStep;

    servoTargetAngle = targetAngle;
    servoStepDirection = (targetAngle > servoCurrentAngle) ? 1 : -1;
    servoMoving = (servoCurrentAngle != servoTargetAngle);
    servoLastStepTime = millis();

    Serial.print(F("[SERVO] Moving to "));
    Serial.print(targetAngle);
    Serial.println(F(" degree"));
}

void Servo_Update(void) {
    if (!servoInitialized || !servoMoving) return;

    uint32_t now = millis();
    if (now - servoLastStepTime < SERVO_STEP_DELAY_MS) return;
    servoLastStepTime = now;

    int16_t next = servoCurrentAngle + (servoStepDirection * SERVO_STEP_ANGLE);

    if (servoStepDirection > 0) {
        if (next >= (int16_t)servoTargetAngle) { next = servoTargetAngle; servoMoving = false; }
    } else {
        if (next <= (int16_t)servoTargetAngle) { next = servoTargetAngle; servoMoving = false; }
    }
    if (next < 0) next = 0;
    if (next > (int16_t)SERVO_TOP_ANGLE) next = SERVO_TOP_ANGLE;

    servoWriteAngle((uint16_t)next);
    servoCurrentAngle = (uint16_t)next;

    if (!servoMoving) {
        Serial.print(F("[SERVO] Reached "));
        Serial.print(servoCurrentAngle);
        Serial.println(F(" degree"));
    }
}

// Direct (immediate) servo control — use for limit-switch callbacks
void Servo_SetAngle(uint16_t angle) {
    if (!servoInitialized) { Serial.println(F("[SERVO] Not initialized")); return; }
    if (angle > SERVO_TOP_ANGLE) angle = SERVO_TOP_ANGLE;
    servoWriteAngle(angle);
    servoCurrentAngle = angle;
    servoTargetAngle = angle;
    servoMoving = false;
    Serial.print(F("[SERVO] Set angle = "));
    Serial.print(angle);
    Serial.println(F(" degree"));
}

uint16_t Servo_GetCurrentAngle(void) { return servoCurrentAngle; }
uint16_t Servo_GetTargetAngle(void)  { return servoTargetAngle; }
bool     Servo_IsMoving(void)         { return servoMoving; }