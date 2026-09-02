/*
 * TIPE DATA DAN ANTARMUKA CONTROLLER FOC MOTOR
 * =============================================
 *
 * Header ini mendefinisikan input motor, output FOC, parameter, lookup table,
 * serta state internal controller. Antarmuka publik menggunakan nama C biasa
 * seperti motorEnable, targetInput, currentD/currentQ, dan dutyPhaseA/B/C.
 *
 * State internal tetap bertipe integer fixed-point agar perilaku numeriknya
 * identik dengan controller sumber.
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

/* Deklarasi awal instance controller motor. */
typedef struct MotorController MotorController;

typedef struct {
  int16_t previousValue;
} CounterI16State;

typedef struct {
  int32_t previousValue1[2];
} LowPassFilterState;

typedef struct {
  uint16_t previousValue;
} CounterU16State;

typedef struct {
  bool previousValue;
} EdgeDetectorState;

typedef struct {
  EdgeDetectorState edgeDetector;
  CounterU16State transitionCounter;
  CounterU16State activationCounter;
  bool previousValue;
} DebounceFilterState;

typedef struct {
  int32_t previousValue;
  int32_t integratorAccumulator;
} BackCalculationState;

typedef struct {
  int32_t integratorState;
  int16_t previousError;
  int16_t lastP;
  int16_t lastI;
  int16_t lastD;
  int16_t lastOutput;
  uint8_t loadInitialState;
  bool previousValue1;
} PidCurrentState;

typedef struct {
  int32_t integratorState;
  int16_t previousError;
  int16_t lastP;
  int16_t lastI;
  int16_t lastD;
  int16_t lastOutput;
  uint8_t loadInitialState;
  bool previousValue1;
} PidSpeedState;

typedef struct {
  int16_t integratorState;
  int16_t previousError;
  int16_t lastP;
  int16_t lastI;
  int16_t lastD;
  int16_t lastOutput;
  uint8_t loadInitialState;
  bool previousValue1;
} PidTorqueState;

typedef struct {
  PidTorqueState torquePid;
  PidSpeedState speedPid;
  PidCurrentState currentPid;
  BackCalculationState voltageBackCalc;
  BackCalculationState speedBackCalc;
  BackCalculationState qCurrentBackCalc;
  EdgeDetectorState diagnosticErrorEdge;
  DebounceFilterState diagnosticDebounce;
  LowPassFilterState currentDqLowPass;
  CounterI16State transitionCounter;
  int32_t qCurrentLimitCorrection;
  int32_t previousValue;
  int16_t phaseModulation[3];
  int16_t filteredCurrentDQ[2];
  int16_t previousRawCounter;
  int16_t voltageQCommand;
  int16_t voltageDCommand;
  int16_t voltageDMax;
  int16_t voltageDMin;
  int16_t voltageQMaxLimit;
  int16_t voltageQMin;
  int16_t maxCurrent;
  int16_t currentQMax;
  int16_t currentQMin;
  int16_t motorCurrentMin;
  int16_t voltageBackCalcLimit;
  int16_t speedBackCalcLimit;
  int16_t qCurrentBackCalcLimit;
  int16_t fieldWeakeningCommand;
  int16_t controlTarget;
  int16_t absControlTarget;
  int16_t absCurrentD;
  int16_t signedSpeedEstimate;
  int16_t sinElectrical;
  int16_t cosElectrical;
  int16_t speedPeriodDelayCurrent;
  int16_t previousRawCounter2;
  int16_t speedPeriodDelay1;
  int16_t speedPeriodDelay2;
  int16_t speedPeriodDelay3;
  int16_t previousAbsSpeed;
  int16_t previousControlTarget;
  int8_t rotationDirection;
  int8_t previousHallPosition;
  int8_t activeBranch1;
  int8_t activeBranch2;
  int8_t activeBranch3;
  int8_t activeSwitchCase1;
  int8_t activeBranch4;
  int8_t activeBranch5;
  int8_t activeSwitchCase2;
  int8_t activeBranch6;
  int8_t activeBranch7;
  uint8_t activeControlMode;
  uint8_t previousHallA;
  uint8_t previousHallB;
  uint8_t previousHallC;
  uint8_t previousErrorCode;
  uint8_t controlModeMachineActive;
  uint8_t controlModeMachineState;
  uint8_t activeControlSubmode;
  bool diagnosticFaultStable;
  bool transitionDetected;
  bool controlDelayA;
  bool controlDelayB;
  bool controlDelayC;
  bool openLoopInitialize;
  bool previousDirectionChange;
  bool commutationAtSpeed;
  bool transitionDetectedState;
} MotorControlState;

/* Lookup table konstan controller */
typedef struct {

  int16_t sinTable[181];

  int16_t cosTable[181];

  int16_t phaseASineTable[181];

  int16_t phaseBSineTable[181];

  int16_t phaseCSineTable[181];

  uint16_t qCurrentScaleTable[50];

  int8_t commutationMapTable[18];

  int8_t hallToPosition[8];
} MotorLookupTables;

/* Input controller motor */
typedef struct {
  bool motorEnable;
  uint8_t controlModeRequest;
  int16_t targetInput;
  uint8_t hallA;
  uint8_t hallB;
  uint8_t hallC;
  int16_t phaseCurrentAB;
  int16_t phaseCurrentBC;
  int16_t dcLinkCurrent;
  int16_t mechanicalAngle;
} MotorInput;

/* Output controller motor */
typedef struct {
  int16_t dutyPhaseA;
  int16_t dutyPhaseB;
  int16_t dutyPhaseC;
  uint8_t errorCode;
  int16_t motorSpeed;
  int16_t electricalAngle;
  int16_t currentQ;
  int16_t currentD;
} MotorOutput;

/* Parameter controller motor */
typedef struct {
  int32_t openLoopVoltageRate;
  int16_t transitionDetectHigh;
  int16_t transitionDetectLow;
  int16_t cruiseSpeedTarget;
  int16_t counterResetMax;
  uint16_t speedCoefficient;
  uint16_t errorDequalifyTime;
  uint16_t errorQualifyTime;
  int16_t voltageDMax;
  int16_t voltageQMaxTable[46];
  int16_t voltageQAxisTable[46];
  int16_t phaseAdvanceMax;
  int16_t maxCurrent;
  int16_t fieldWeakeningCurrentMax;
  int16_t commutationActivateSpeedLow;
  int16_t commutationDeactivateSpeedHigh;
  int16_t fieldWeakeningSpeedHigh;
  int16_t fieldWeakeningSpeedLow;
  int16_t maxSpeed;
  int16_t standstillSpeedThreshold;
  int16_t inputTargetErrorThreshold;
  int16_t fieldWeakeningHigh;
  int16_t fieldWeakeningLow;
  uint16_t backCalculationGain;
  uint16_t currentDKp;
  uint16_t currentQKp;
  uint16_t speedKp;
  uint16_t currentDKd;
  uint16_t currentQKd;
  uint16_t speedKd;
  uint16_t currentFilterCoefficient;
  uint16_t currentDKi;
  uint16_t currentQKi;
  uint16_t currentQKiLimitProtection;
  uint16_t speedKi;
  uint16_t speedKiLimitProtection;
  uint8_t polePairs;
  uint8_t controlType;
  uint8_t phaseCurrentMeasurementMode;
  bool useMeasuredAngle;
  bool cruiseControlEnabled;
  bool diagnosticsEnabled;
  bool fieldWeakeningEnabled;
} MotorParameters;

/* Instance controller yang menghubungkan parameter, input, output, dan state. */
struct MotorController {
  MotorParameters *parameters;
  MotorInput *input;
  MotorOutput *output;
  MotorControlState *state;
};

/* Lookup table konstan controller */
extern const MotorLookupTables motorLookupTables;

/* Fungsi utama controller FOC. */
extern void MotorController_Init(MotorController *const controller);
extern void MotorController_Update(MotorController *const controller);

#endif /* FOC_MOTOR_H */
