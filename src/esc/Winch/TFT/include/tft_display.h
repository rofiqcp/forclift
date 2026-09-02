#ifndef TFT_DISPLAY_H
#define TFT_DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>

// ============================================
// PIN DEFINITIONS
// ============================================
#define TFT_CS      PB0
#define TFT_DC      PB1
#define TFT_RST     PB2

#define TOUCH_CS    PA4

#define SPI_SCK     PA5
#define SPI_MISO    PA6
#define SPI_MOSI    PA7

// ============================================
// DISPLAY SETTINGS
// ============================================
#define TFT_WIDTH   240
#define TFT_HEIGHT  320

// SPI Frequencies
#define TFT_SPI_FREQ    40000000  // 40 MHz for TFT
#define TOUCH_SPI_FREQ  2000000   // 2 MHz for Touch (slower for stability)

// ============================================
// COLORS (16-bit RGB565)
// ============================================
#define COLOR_BLACK       0x0000
#define COLOR_NAVY        0x000F
#define COLOR_DARKGREEN   0x03E0
#define COLOR_DARKCYAN    0x03EF
#define COLOR_MAROON      0x7800
#define COLOR_PURPLE      0x780F
#define COLOR_OLIVE       0x7BE0
#define COLOR_LIGHTGREY   0xC618
#define COLOR_DARKGREY    0x7BEF
#define COLOR_BLUE        0x001F
#define COLOR_GREEN       0x07E0
#define COLOR_CYAN        0x07FF
#define COLOR_RED         0xF800
#define COLOR_MAGENTA     0xF81F
#define COLOR_YELLOW      0xFFE0
#define COLOR_WHITE       0xFFFF
#define COLOR_ORANGE      0xFD20
#define COLOR_GREENYELLOW 0xAFE5
#define COLOR_PINK        0xF81F

// ============================================
// TOUCH CALIBRATION (will be updated after calibration)
// ============================================
#define TS_MINX  150
#define TS_MAXX  3800
#define TS_MINY  150
#define TS_MAXY  3800

// ============================================
// BUTTON DEFINITIONS
// ============================================
#define BUTTON_X      40
#define BUTTON_Y      220
#define BUTTON_W      160
#define BUTTON_H      50
#define BUTTON_RADIUS 8

// ============================================
// GLOBAL OBJECTS
// ============================================
extern Adafruit_ILI9341 tft;

// ============================================
// FUNCTION DECLARATIONS
// ============================================
void TFT_Init();
void TFT_ColorTest();
void TFT_DrawMainScreen();
void TFT_DrawButton(bool pressed);
void TFT_UpdateTouchCoordinates(int16_t x, int16_t y, int16_t z);
void TFT_ShowCalibrationScreen();
void TFT_DrawCalibrationPoint(uint8_t index, int16_t x, int16_t y);
void TFT_ClearTouchArea();

#endif // TFT_DISPLAY_H