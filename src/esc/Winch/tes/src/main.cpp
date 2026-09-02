#include <Arduino.h>

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 5000) delay(10);
    delay(500);
    
    Serial.println("=== LED Blink Test - 10x ===");
    Serial.println("LED_BUILTIN = PC13 (Active LOW)");
    Serial.println("Starting blink sequence...");
}

void loop() {
    // Blink 10 times
    for (int i = 1; i <= 10; i++) {
        digitalWrite(LED_BUILTIN, LOW);   // LED ON (active LOW)
        Serial.print("Blink ");
        Serial.print(i);
        Serial.println(" - LED ON");
        delay(500);
        
        digitalWrite(LED_BUILTIN, HIGH);  // LED OFF
        Serial.print("Blink ");
        Serial.print(i);
        Serial.println(" - LED OFF");
        delay(500);
    }
    
    Serial.println("=== Test Complete - 10 blinks done ===");
    Serial.println("LED will stay OFF. Press RESET to run again.");
    
    // Keep LED off and stop
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) {
        delay(1000);
    }
}