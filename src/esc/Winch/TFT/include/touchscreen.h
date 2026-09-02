#ifndef TOUCHSCREEN_H
#define TOUCHSCREEN_H

#include <Arduino.h>
#include <XPT2046_Touchscreen_TT.h>
#include <SPI.h>

// ============================================
// PIN DEFINITIONS (shared with tft_display.h)
// ============================================
// TOUCH_CS = PA4
// SPI_SCK  = PA5
// SPI_MISO = PA6
// SPI_MOSI = PA7

// ============================================
// TOUCH SETTINGS
// ============================================
#define TOUCH_SPI_FREQ  2000000  // 2 MHz for touch stability

// ============================================
// CALIBRATION VALUES (defaults - will be calibrated)
// ============================================
#define TS_MINX  150
#define TS_MAXX  3800
#define TS_MINY  150
#define TS_MAXY  3800

// ============================================
// GLOBAL OBJECTS
// ============================================
extern XPT2046_Touchscreen ts;
extern SPIClass spi;

// ============================================
// TOUCH DATA STRUCTURE
// ============================================
struct TouchData {
    bool pressed;
    int16_t rawX;
    int16_t rawY;
    int16_t rawZ;
    int16_t screenX;
    int16_t screenY;
    bool valid;
};

// ============================================
// FUNCTION DECLARATIONS
// ============================================
void Touch_Init();
bool Touch_Update(TouchData* data);
bool Touch_IsPressed();
int16_t Touch_GetRawX();
int16_t Touch_GetRawY();
int16_t Touch_GetRawZ();

// Calibration functions
void Touch_CalibrationStart();
void Touch_CalibrationAddPoint(int16_t rawX, int16_t rawY);
bool Touch_CalibrationComplete();
void Touch_CalibrationGetValues(uint16_t* minX, uint16_t* maxX, uint16_t* minY, uint16_t* maxY);
bool Touch_CalibrationIsActive();
uint8_t Touch_CalibrationGetStep();
int16_t Touch_CalibrationGetScreenX(uint8_t step);
int16_t Touch_CalibrationGetScreenY(uint8_t step);

// Mapping functions
int16_t Touch_MapX(int16_t rawX);
int16_t Touch_MapY(int16_t rawY);
bool Touch_IsInButton(int16_t x, int16_t y);

#endif // TOUCHSCREEN_H