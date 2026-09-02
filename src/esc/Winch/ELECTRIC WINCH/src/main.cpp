// ============================================================================
// ELECTRIC WINCH TEST (BTS7960)
// Board   : Black Pill STM32F411CEU6 (WeAct V2.0)
// Driver  : BTS7960 (RPWM=PA2, LPWM=PB8)
// Limits  : LS_TOP=PB6, LS_BOTTOM=PB7
// Serial  : USB CDC (native COM port on Windows)
// ============================================================================

#include <Arduino.h>
#include "electric_winch.h"

// ============================================================================
// SERIAL COMMAND PROCESSING
// ============================================================================
static String serialBuffer = "";
static const size_t MAX_BUFFER_SIZE = 64;

void processSerial() {
    while (WINCH_SERIAL.available()) {
        char c = WINCH_SERIAL.read();
        
        // Ignore carriage return
        if (c == '\r') continue;
        
        // Newline = end of command
        if (c == '\n') {
            if (serialBuffer.length() > 0) {
                Winch_ProcessCommand(serialBuffer);
                serialBuffer = "";
            }
        } else {
            // Accumulate command
            if (serialBuffer.length() < MAX_BUFFER_SIZE) {
                serialBuffer += c;
            }
        }
    }
}

// ============================================================================
// ARDUINO ENTRY POINTS
// ============================================================================

void setup() {
    // CDC smoke test: initialize immediately without waiting for DTR/host.
    Serial.begin(115200);
    delay(200);
    Serial.println("BLACKPILL USB CDC OK");

    // Built-in LED for heartbeat
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);  // LED on (active low)

    // Small delay for serial to stabilize
    delay(100);

    // Initialize winch system
    Winch_Init();
    
    // Send ready message
    WINCH_SERIAL.println(F("[WINCH] Setup complete, ready for commands"));
    WINCH_SERIAL.println(F("[WINCH] LED should be ON"));
}

void loop() {
    // Non-blocking winch state machine update
    Winch_Update();
    
    // Process incoming serial commands
    processSerial();
    
    // LED heartbeat (blink every 1 second)
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink >= 1000) {
        digitalWrite(PC13, !digitalRead(PC13));
        lastBlink = millis();
    }

    // Optional USB CDC heartbeat, useful for confirming the COM port is alive.
    static uint32_t lastCdcPrint = 0;
    if (millis() - lastCdcPrint >= 1000) {
        Serial.println("BLACKPILL USB CDC OK");
        lastCdcPrint = millis();
    }

    // Small yield for stability
    delay(1);
}