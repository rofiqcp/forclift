#include "tft_display.h"
#include "touchscreen.h"
#include <SPI.h>

// ============================================
// GLOBAL OBJECTS
// ============================================
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ============================================
// TFT INITIALIZATION
// ============================================
void TFT_Init() {
    Serial.println("[INIT] Initializing SPI...");
    
    // Configure SPI pins for TFT
    SPI.setSCLK(SPI_SCK);
    SPI.setMISO(SPI_MISO);
    SPI.setMOSI(SPI_MOSI);
    SPI.begin();
    
    Serial.println("[INIT] Initializing TFT...");
    
    // Initialize TFT with SPI frequency
    tft.begin(TFT_SPI_FREQ);
    
    // Set rotation for portrait mode (240x320)
    tft.setRotation(0);  // Portrait 240x320
    
    // Fill screen black
    tft.fillScreen(COLOR_BLACK);
    
    Serial.println("[OK] TFT initialized");
    Serial.printf("Resolution: %d x %d\n", tft.width(), tft.height());
    Serial.printf("Rotation: %d\n", tft.getRotation());
}

// ============================================
// COLOR TEST SEQUENCE
// ============================================
void TFT_ColorTest() {
    Serial.println("[TEST] Starting color test...");
    
    uint16_t colors[] = {
        COLOR_BLACK,
        COLOR_RED,
        COLOR_GREEN,
        COLOR_BLUE,
        COLOR_WHITE
    };
    
    const char* colorNames[] = {
        "BLACK",
        "RED",
        "GREEN",
        "BLUE",
        "WHITE"
    };
    
    for (int i = 0; i < 5; i++) {
        Serial.printf("[TEST] Color: %s\n", colorNames[i]);
        tft.fillScreen(colors[i]);
        delay(500);
    }
    
    Serial.println("[TEST] Color test complete");
}

// ============================================
// DRAW MAIN SCREEN
// ============================================
void TFT_DrawMainScreen() {
    // Clear screen
    tft.fillScreen(COLOR_BLACK);
    
    // Title bar
    tft.fillRect(0, 0, TFT_WIDTH, 30, COLOR_NAVY);
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 5);
    tft.print("NADIA AGV TFT");
    
    // Info section
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(10, 40);
    tft.print("TFT LCD 2.4\" TEST");
    
    tft.setCursor(10, 55);
    tft.print("Resolution : 240 x 320");
    
    tft.setCursor(10, 70);
    tft.print("MCU : STM32F411CEU6");
    
    // Touch status section
    tft.setTextColor(COLOR_YELLOW);
    tft.setTextSize(1);
    tft.setCursor(10, 95);
    tft.print("TOUCH STATUS");
    
    tft.setTextColor(COLOR_WHITE);
    tft.setCursor(10, 115);
    tft.print("X : ----");
    
    tft.setCursor(10, 130);
    tft.print("Y : ----");
    
    tft.setCursor(10, 145);
    tft.print("Z : ----");
    
    // Draw button
    TFT_DrawButton(false);
    
    // Status line
    tft.setTextColor(COLOR_GREEN);
    tft.setTextSize(1);
    tft.setCursor(10, 290);
    tft.print("Status : READY");
    
    Serial.println("[DRAW] Main screen drawn");
}

// ============================================
// DRAW BUTTON
// ============================================
void TFT_DrawButton(bool pressed) {
    uint16_t btnColor = pressed ? COLOR_RED : COLOR_GREEN;
    uint16_t textColor = COLOR_WHITE;
    uint16_t borderColor = pressed ? COLOR_YELLOW : COLOR_WHITE;
    
    // Button background
    tft.fillRoundRect(BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, BUTTON_RADIUS, btnColor);
    
    // Button border
    tft.drawRoundRect(BUTTON_X, BUTTON_Y, BUTTON_W, BUTTON_H, BUTTON_RADIUS, borderColor);
    
    // Button text
    tft.setTextColor(textColor);
    tft.setTextSize(2);
    
    int16_t x1, y1;
    uint16_t w, h;
    const char* btnText = pressed ? "PRESSED" : "PRESS ME";
    tft.getTextBounds(btnText, 0, 0, &x1, &y1, &w, &h);
    
    int16_t textX = BUTTON_X + (BUTTON_W - w) / 2;
    int16_t textY = BUTTON_Y + (BUTTON_H + h) / 2 - 2;
    
    tft.setCursor(textX, textY);
    tft.print(btnText);
}

// ============================================
// UPDATE TOUCH COORDINATES ON SCREEN
// ============================================
void TFT_UpdateTouchCoordinates(int16_t x, int16_t y, int16_t z) {
    // Clear previous values area
    tft.fillRect(40, 115, 100, 45, COLOR_BLACK);
    
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(40, 115);
    tft.printf("X : %4d", x);
    
    tft.setCursor(40, 130);
    tft.printf("Y : %4d", y);
    
    tft.setCursor(40, 145);
    tft.printf("Z : %4d", z);
}

// ============================================
// CLEAR TOUCH AREA
// ============================================
void TFT_ClearTouchArea() {
    tft.fillRect(10, 95, 220, 115, COLOR_BLACK);
    TFT_DrawButton(false);
}

// ============================================
// CALIBRATION SCREEN
// ============================================
void TFT_ShowCalibrationScreen() {
    tft.fillScreen(COLOR_BLACK);
    
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(2);
    tft.setCursor(30, 20);
    tft.print("TOUCH CALIBRATION");
    
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.print("Touch each crosshair");
    tft.setCursor(10, 65);
    tft.print("center precisely");
    
    tft.setTextColor(COLOR_YELLOW);
    tft.setCursor(10, 290);
    tft.print("Step 1/5: TOP LEFT");
}

// ============================================
// DRAW CALIBRATION POINT
// ============================================
void TFT_DrawCalibrationPoint(uint8_t index, int16_t x, int16_t y) {
    // Draw crosshair
    tft.drawLine(x - 10, y, x + 10, y, COLOR_RED);
    tft.drawLine(x, y - 10, x, y + 10, COLOR_RED);
    tft.drawCircle(x, y, 15, COLOR_RED);
    tft.drawCircle(x, y, 16, COLOR_RED);
    
    // Show index
    tft.setTextColor(COLOR_WHITE);
    tft.setTextSize(1);
    tft.setCursor(x - 5, y - 25);
    tft.printf("%d", index);
}