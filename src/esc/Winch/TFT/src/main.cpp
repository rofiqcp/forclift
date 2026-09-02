#include <Arduino.h>
#include "tft_display.h"
#include "touchscreen.h"

// ============================================
// GLOBAL STATE
// ============================================
enum AppState {
    STATE_INIT,
    STATE_COLOR_TEST,
    STATE_MAIN_SCREEN,
    STATE_CALIBRATION,
    STATE_RUNNING
};

AppState currentState = STATE_INIT;

// Touch data
TouchData touchData;

// Button state
bool buttonPressed = false;
bool buttonPrevPressed = false;

// Timing
unsigned long lastTouchTime = 0;
unsigned long colorTestStartTime = 0;
int colorTestStep = 0;

// Calibration
bool calibrationMode = false;
bool showCalibrationHint = true;

// ============================================
// FORWARD DECLARATIONS
// ============================================
void HandleColorTest();
void HandleRunning();
void HandleCalibration();
void EnterCalibrationMode();
void PrintHelp();

// ============================================
// SETUP
// ============================================
void setup() {
    // Initialize Serial
    Serial.begin(115200);
    delay(1000);  // Wait for serial monitor
    
    Serial.println();
    Serial.println("================================");
    Serial.println("NADIA TFT TEST");
    Serial.println("================================");
    Serial.println();
    Serial.println("[INIT] STM32F411CEU6");
    Serial.println("[INIT] Black Pill V2.0");
    Serial.println();
    
    // Initialize TFT
    TFT_Init();
    
    // Initialize Touchscreen
    Touch_Init();
    
    Serial.println();
    Serial.println("System ready.");
    Serial.println();
    
    // Start color test
    currentState = STATE_COLOR_TEST;
    colorTestStartTime = millis();
    colorTestStep = 0;
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    switch (currentState) {
        case STATE_COLOR_TEST:
            HandleColorTest();
            break;
            
        case STATE_MAIN_SCREEN:
            TFT_DrawMainScreen();
            currentState = STATE_RUNNING;
            break;
            
        case STATE_CALIBRATION:
            HandleCalibration();
            break;
            
        case STATE_RUNNING:
            HandleRunning();
            break;
            
        default:
            break;
    }
    
    delay(10);  // Small delay for stability
}

// ============================================
// COLOR TEST HANDLER
// ============================================
void HandleColorTest() {
    static uint16_t colors[] = {COLOR_BLACK, COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE};
    static const char* colorNames[] = {"BLACK", "RED", "GREEN", "BLUE", "WHITE"};
    
    if (millis() - colorTestStartTime >= 500) {
        if (colorTestStep < 5) {
            Serial.printf("[TEST] Color: %s\n", colorNames[colorTestStep]);
            tft.fillScreen(colors[colorTestStep]);
            colorTestStep++;
            colorTestStartTime = millis();
        } else {
            Serial.println("[TEST] Color test complete");
            currentState = STATE_MAIN_SCREEN;
        }
    }
}

// ============================================
// RUNNING STATE HANDLER
// ============================================
void HandleRunning() {
    // Check for calibration mode (hold button for 3 seconds)
    if (buttonPressed && !calibrationMode) {
        if (millis() - lastTouchTime > 3000) {
            EnterCalibrationMode();
        }
    }
    
        // Update touch
        if (Touch_Update(&touchData)) {
            // Touch detected
            lastTouchTime = millis();
        
        // Print to Serial
        Serial.printf("[TOUCH] RAW X=%d, RAW Y=%d, Z=%d\n", 
                      touchData.rawX, touchData.rawY, touchData.rawZ);
        Serial.printf("[TOUCH] SCREEN X=%d, SCREEN Y=%d\n", 
                      touchData.screenX, touchData.screenY);
        
        // Update display
        TFT_UpdateTouchCoordinates(touchData.screenX, touchData.screenY, touchData.rawZ);
        
        // Check button
        bool inButton = Touch_IsInButton(touchData.screenX, touchData.screenY);
        
        if (inButton && !buttonPressed) {
            // Button pressed
            buttonPressed = true;
            TFT_DrawButton(true);
            Serial.println("[BUTTON] PRESSED");
            
            // Update status
            tft.setTextColor(COLOR_RED);
            tft.setTextSize(1);
            tft.setCursor(10, 290);
            tft.print("Status : PRESSED     ");
        }
        else if (!inButton && buttonPressed) {
            // Button released (was pressed, now outside)
            buttonPressed = false;
            TFT_DrawButton(false);
            Serial.println("[BUTTON] RELEASED");
            
            // Update status
            tft.setTextColor(COLOR_GREEN);
            tft.setTextSize(1);
            tft.setCursor(10, 290);
            tft.print("Status : RELEASED    ");
        }
        else if (inButton && buttonPressed) {
            // Still pressed, update status
            tft.setTextColor(COLOR_RED);
            tft.setTextSize(1);
            tft.setCursor(10, 290);
            tft.print("Status : PRESSED     ");
        }
    }
    else if (buttonPressed) {
        // Touch released
        buttonPressed = false;
        TFT_DrawButton(false);
        Serial.println("[BUTTON] RELEASED");
        
        // Update status
        tft.setTextColor(COLOR_GREEN);
        tft.setTextSize(1);
        tft.setCursor(10, 290);
        tft.print("Status : RELEASED    ");
    }
    
    // Check for calibration trigger (serial command)
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'c' || cmd == 'C') {
            EnterCalibrationMode();
        }
        else if (cmd == 'r' || cmd == 'R') {
            // Reset - re-run color test
            currentState = STATE_COLOR_TEST;
            colorTestStartTime = millis();
            colorTestStep = 0;
        }
        else if (cmd == 'h' || cmd == 'H') {
            PrintHelp();
        }
    }
}

// ============================================
// CALIBRATION MODE
// ============================================
void EnterCalibrationMode() {
    Serial.println();
    Serial.println("[CAL] Entering calibration mode...");
    Serial.println("[CAL] Touch each crosshair center precisely");
    Serial.println();
    
    calibrationMode = true;
    Touch_CalibrationStart();
    TFT_ShowCalibrationScreen();
    currentState = STATE_CALIBRATION;
}

void HandleCalibration() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        
        // Add calibration point
        Touch_CalibrationAddPoint(p.x, p.y);
        
        // Draw feedback on screen
        uint8_t step = Touch_CalibrationGetStep() - 1;  // Step just completed
        if (step < 5) {
            int16_t sx = Touch_CalibrationGetScreenX(step);
            int16_t sy = Touch_CalibrationGetScreenY(step);
            TFT_DrawCalibrationPoint(step, sx, sy);
            
            // Show next step
            if (Touch_CalibrationGetStep() < 5) {
                const char* stepNames[] = {"TOP RIGHT", "BOTTOM RIGHT", "BOTTOM LEFT", "CENTER"};
                tft.setTextColor(COLOR_YELLOW);
                tft.fillRect(10, 290, 220, 20, COLOR_BLACK);
                tft.setCursor(10, 290);
                tft.printf("Step %d/5: %s", Touch_CalibrationGetStep() + 1, stepNames[Touch_CalibrationGetStep() - 1]);
            }
        }
        
        // Check if complete
                if (Touch_CalibrationComplete()) {
                    Serial.println();
                    Serial.println("[CAL] Calibration values saved!");
            
                    uint16_t minX, maxX, minY, maxY;
                    Touch_CalibrationGetValues(&minX, &maxX, &minY, &maxY);
                    Serial.printf("[CAL] #define TS_MINX %d\n", minX);
                    Serial.printf("[CAL] #define TS_MAXX %d\n", maxX);
                    Serial.printf("[CAL] #define TS_MINY %d\n", minY);
                    Serial.printf("[CAL] #define TS_MAXY %d\n", maxY);
                    Serial.println();
                    Serial.println("[CAL] Update tft_display.h and touchscreen.h with these values");
                    Serial.println("[CAL] Press 'R' to restart test");
                    Serial.println();
            
            // Show completion
            tft.fillScreen(COLOR_GREEN);
            tft.setTextColor(COLOR_WHITE);
            tft.setTextSize(2);
            tft.setCursor(40, 140);
            tft.print("CALIBRATION");
            tft.setCursor(40, 170);
            tft.print("COMPLETE!");
            
            calibrationMode = false;
            // Stay in calibration state until reset
        }
        
        // Debounce
        delay(300);
    }
}

// ============================================
// SERIAL COMMAND HELP
// ============================================
void PrintHelp() {
    Serial.println();
    Serial.println("COMMANDS:");
    Serial.println("  C - Enter calibration mode");
    Serial.println("  R - Restart color test");
    Serial.println("  H - Show this help");
    Serial.println();
}