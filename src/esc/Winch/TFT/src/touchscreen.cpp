#include "touchscreen.h"
#include "tft_display.h"
#include <SPI.h>

// ============================================
// GLOBAL OBJECTS
// ============================================
XPT2046_Touchscreen ts(TOUCH_CS);

// ============================================
// CALIBRATION STATE
// ============================================
static bool calibrationActive = false;
static uint8_t calibrationStep = 0;
static int16_t calRawX[5] = {0};
static int16_t calRawY[5] = {0};
static uint16_t calMinX = TS_MINX, calMaxX = TS_MAXX;
static uint16_t calMinY = TS_MINY, calMaxY = TS_MAXY;

// Calibration screen coordinates (in screen pixels)
static const int16_t calScreenX[5] = {20, 220, 220, 20, 120};
static const int16_t calScreenY[5] = {100, 100, 220, 220, 160};

// ============================================
// TOUCH INITIALIZATION
// ============================================
void Touch_Init() {
    Serial.println("[INIT] Initializing Touchscreen SPI...");
    
    // Configure SPI pins for touch (same as TFT)
    SPI.setSCLK(SPI_SCK);
    SPI.setMISO(SPI_MISO);
    SPI.setMOSI(SPI_MOSI);
    
    Serial.println("[INIT] Initializing Touchscreen...");
    
    // Initialize touchscreen with touch frequency
    ts.begin(SPI);
    ts.setRotation(0);  // Match TFT rotation
    
    Serial.println("[OK] Touchscreen initialized");
}

// ============================================
// UPDATE TOUCH DATA
// ============================================
bool Touch_Update(TouchData* data) {
    // Read touch (SPI handled by library)
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        
        data->pressed = true;
        data->rawX = p.x;
        data->rawY = p.y;
        data->rawZ = p.z;
        data->valid = true;
        
        // Map to screen coordinates
        data->screenX = Touch_MapX(p.x);
        data->screenY = Touch_MapY(p.y);
        
        // Constrain to screen bounds
        data->screenX = constrain(data->screenX, 0, TFT_WIDTH - 1);
        data->screenY = constrain(data->screenY, 0, TFT_HEIGHT - 1);
        
        return true;
    } else {
        data->pressed = false;
        data->rawX = 0;
        data->rawY = 0;
        data->rawZ = 0;
        data->screenX = -1;
        data->screenY = -1;
        data->valid = false;
        return false;
    }
}

// ============================================
// SIMPLE TOUCH CHECK
// ============================================
bool Touch_IsPressed() {
    return ts.touched();
}

// ============================================
// GET RAW VALUES
// ============================================
int16_t Touch_GetRawX() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        return p.x;
    }
    return 0;
}

int16_t Touch_GetRawY() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        return p.y;
    }
    return 0;
}

int16_t Touch_GetRawZ() {
    if (ts.touched()) {
        TS_Point p = ts.getPoint();
        return p.z;
    }
    return 0;
}

// ============================================
// MAPPING FUNCTIONS
// ============================================
int16_t Touch_MapX(int16_t rawX) {
    // Map raw X to screen X (0-239)
    // Handle reverse if needed
    int32_t mapped = map(rawX, calMinX, calMaxX, 0, TFT_WIDTH - 1);
    return (int16_t)constrain(mapped, 0, TFT_WIDTH - 1);
}

int16_t Touch_MapY(int16_t rawY) {
    // Map raw Y to screen Y (0-319)
    int32_t mapped = map(rawY, calMinY, calMaxY, 0, TFT_HEIGHT - 1);
    return (int16_t)constrain(mapped, 0, TFT_HEIGHT - 1);
}

// ============================================
// CHECK IF TOUCH IS IN BUTTON AREA
// ============================================
bool Touch_IsInButton(int16_t x, int16_t y) {
    return (x >= BUTTON_X && x <= BUTTON_X + BUTTON_W &&
            y >= BUTTON_Y && y <= BUTTON_Y + BUTTON_H);
}

// ============================================
// CALIBRATION FUNCTIONS
// ============================================
void Touch_CalibrationStart() {
    calibrationActive = true;
    calibrationStep = 0;
    memset(calRawX, 0, sizeof(calRawX));
    memset(calRawY, 0, sizeof(calRawY));
    
    Serial.println("[CAL] Calibration started");
    Serial.println("[CAL] Touch TOP LEFT crosshair");
}

void Touch_CalibrationAddPoint(int16_t rawX, int16_t rawY) {
    if (calibrationStep < 5) {
        calRawX[calibrationStep] = rawX;
        calRawY[calibrationStep] = rawY;
        
        Serial.printf("[CAL] Step %d: RAW X=%d, RAW Y=%d\n", 
                      calibrationStep + 1, rawX, rawY);
        
        calibrationStep++;
        
        if (calibrationStep < 5) {
            const char* stepNames[] = {"TOP RIGHT", "BOTTOM RIGHT", "BOTTOM LEFT", "CENTER"};
            Serial.printf("[CAL] Touch %s crosshair\n", stepNames[calibrationStep - 1]);
        }
    }
}

bool Touch_CalibrationComplete() {
    if (calibrationStep >= 5) {
        // Calculate calibration values
        // Top-left, Top-right, Bottom-right, Bottom-left, Center
        
        // Find min/max X from left and right points
        calMinX = min(calRawX[0], calRawX[3]);  // Left points
        calMaxX = max(calRawX[1], calRawX[2]);  // Right points
        
        // Find min/max Y from top and bottom points
        calMinY = min(calRawY[0], calRawY[1]);  // Top points
        calMaxY = max(calRawY[2], calRawY[3]);  // Bottom points
        
        // Add some margin
        int margin = 50;
        calMinX = max((int)calMinX - margin, 0);
        calMaxX = min((int)calMaxX + margin, 4095);
        calMinY = max((int)calMinY - margin, 0);
        calMaxY = min((int)calMaxY + margin, 4095);
        
        calibrationActive = false;
        
        Serial.println("[CAL] Calibration complete!");
        Serial.printf("[CAL] TS_MINX=%d, TS_MAXX=%d\n", calMinX, calMaxX);
        Serial.printf("[CAL] TS_MINY=%d, TS_MAXY=%d\n", calMinY, calMaxY);
        
        return true;
    }
    return false;
}

void Touch_CalibrationGetValues(uint16_t* minX, uint16_t* maxX, uint16_t* minY, uint16_t* maxY) {
    *minX = calMinX;
    *maxX = calMaxX;
    *minY = calMinY;
    *maxY = calMaxY;
}

bool Touch_CalibrationIsActive() {
    return calibrationActive;
}

uint8_t Touch_CalibrationGetStep() {
    return calibrationStep;
}

int16_t Touch_CalibrationGetScreenX(uint8_t step) {
    return calScreenX[step];
}

int16_t Touch_CalibrationGetScreenY(uint8_t step) {
    return calScreenY[step];
}