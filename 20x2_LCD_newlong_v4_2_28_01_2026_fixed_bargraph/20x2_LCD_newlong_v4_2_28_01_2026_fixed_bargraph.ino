/*
 * WEB GUIDE CONTROLLER V4.1 - IMPROVED DUAL-LOOP ARCHITECTURE
 * 
 * Purpose: Web alignment controller for industrial machinery
 * Features: PID control, manual/auto modes, homing, calibration, centering
 * Architecture: Dual-loop with hardware timer for stepper control
 * 
 * KEY IMPROVEMENTS:
 * 1. Hardware Timer1 for stepper pulse generation (independent of main loop)
 * 2. Non-blocking LCD state machine with buffering
 * 3. True parallel execution of stepper and display
 * 4. Predictable timing for all critical operations
 * 5. Removed unused functions and variables
 * 6. Fixed display update conflicts
 * 7. Improved state machine management
 * 8. Enhanced error handling
 */

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <avr/wdt.h>
#include <TimerOne.h>  // Hardware timer for stepper control

// ====================================================================
// STRING CONSTANTS (PROGMEM - Saves RAM by storing in program memory)
// ====================================================================
const char STR_HOMING[] PROGMEM = "Homing...";
const char STR_CENTERING[] PROGMEM = "Centering...";
const char STR_TIMEOUT[] PROGMEM = "Timeout!";
const char STR_LEFT_LIMIT[] PROGMEM = "Left limit found";
const char STR_RIGHT_LIMIT[] PROGMEM = "Right limit found";
const char STR_HOME_FIRST[] PROGMEM = "Home first!";
const char STR_CALIBRATE_FIRST[] PROGMEM = "Calibrate first!";
const char STR_CENTER_TIMEOUT[] PROGMEM = "Center Timeout!";
const char STR_LIMIT_HIT[] PROGMEM = "Limit hit!";
const char STR_ADJUSTING[] PROGMEM = "Adjusting..";
const char STR_WEB_GUIDE[] PROGMEM = "WEB GUIDE V4.1";
const char STR_INITIALIZING[] PROGMEM = "Initializing...";
const char STR_NEEDS_CALIBRATION[] PROGMEM = "[NEEDS CALIBRATION!]";
const char STR_LEFT_LIMIT_ROMANIAN[] PROGMEM = "<<< LIMITA STANGA";
const char STR_RIGHT_LIMIT_ROMANIAN[] PROGMEM = "LIMITA DREAPTA >>>";

// ====================================================================
// DUAL-LOOP ARCHITECTURE SETTINGS
// ====================================================================

// Hardware Timer Settings (for stepper control)
const unsigned long STEPPER_TIMER_INTERVAL = 50;  // 50us = 20kHz update rate
volatile bool stepperTimerFlag = false;
volatile bool runStepperInInterrupt = false;
volatile unsigned long stepperInterruptCounter = 0;

// LCD State Machine (Non-blocking)
enum LcdUpdateState {
  LCD_IDLE,
  LCD_WRITING_CHAR,
  LCD_WAITING,
  LCD_CLEARING,
  LCD_SETTING_CURSOR
};

LcdUpdateState lcdState = LCD_IDLE;
unsigned long lcdStateTimer = 0;
const unsigned long LCD_CHAR_DELAY = 40;  // 40us between characters
int lcdCurrentRow = 0;
int lcdCurrentCol = 0;
char lcdLine0[21] = "                    ";
char lcdLine1[21] = "                    ";
char lcdNewLine0[21] = "                    ";
char lcdNewLine1[21] = "                    ";
bool lcdNeedsUpdate = false;
bool lcdUpdateInProgress = false;

// Main Loop Timing
unsigned long lastMainLoopTime = 0;
const unsigned long MAIN_LOOP_INTERVAL = 1000;  // Track main loop frequency

// ====================================================================
// USER-ADJUSTABLE PARAMETERS (SAFE TO MODIFY)
// ====================================================================

// PID CONTROL PARAMETERS
const double KP_DEFAULT = 20.0;      // Proportional gain (adjust for responsiveness)
const double KP_MIN = 0.0;           // Minimum Kp value
const double KP_MAX = 1000.0;        // Maximum Kp value
double Kp = KP_DEFAULT;              // Current Kp value

const double KI_DEFAULT = 0.0;       // Integral gain (adjust for steady-state error)
const double KI_MIN = 0.0;           // Minimum Ki value
const double KI_MAX = 1.0;           // Maximum Ki value
double Ki = KI_DEFAULT;              // Current Ki value

const double KD_DEFAULT = 0.5;       // Derivative gain (adjust for overshoot)
const double KD_MIN = 0.0;           // Minimum Kd value
const double KD_MAX = 1000.0;        // Maximum Kd value
double Kd = KD_DEFAULT;              // Current Kd value

const int PID_DIRECTION_DEFAULT = DIRECT;  // PID direction: DIRECT or REVERSE
int pidDirection = PID_DIRECTION_DEFAULT;  // Current PID direction

// SENSOR SETTINGS
const double SETPOINT_DEFAULT = 4.25;  // Target voltage for sensor (V)
const double SETPOINT_MIN = 0.0;       // Minimum setpoint
const double SETPOINT_MAX = 6.0;       // Maximum setpoint
double setPoint = SETPOINT_DEFAULT;    // Current setpoint

const double DEADBAND_DEFAULT = 0.05;  // Deadband around setpoint (V)
const double DEADBAND_MIN = 0.0;       // Minimum deadband
const double DEADBAND_MAX = 0.5;       // Maximum deadband
double deadband = DEADBAND_DEFAULT;    // Current deadband

// VOLTAGE DIVIDER SETTINGS (for sensor input)
const double R1_DEFAULT = 1000.0;      // Voltage divider resistor 1 (Ω)
double R1 = R1_DEFAULT;                // Current R1 value
const double R2_DEFAULT = 5100.0;      // Voltage divider resistor 2 (Ω)
double R2 = R2_DEFAULT;                // Current R2 value
const double ADC_REF_VOLTAGE = 5.0;    // Arduino reference voltage
const int ADC_RESOLUTION = 1023;       // ADC resolution (10-bit)

// STEPPER MOTOR SPEED SETTINGS
const int STEPPER_MIN_SPEED_DEFAULT = 50;  // Minimum stepper speed (steps/sec)
int stepperMinSpeed = STEPPER_MIN_SPEED_DEFAULT;  // Current min speed

const int STEPPER_MAX_SPEED_DEFAULT = 1000;      // Maximum stepper speed (steps/sec)
const int STEPPER_MAX_SPEED_MAX = 10000;         // Absolute maximum speed
int stepperMaxSpeed = STEPPER_MAX_SPEED_DEFAULT; // Current max speed

const int HOMING_SPEED_DEFAULT = 500;   // Speed during homing (steps/sec)
const int HOMING_SPEED_MIN = 50;        // Minimum homing speed
const int HOMING_SPEED_MAX = 10000;     // Maximum homing speed
int homingSpeed = HOMING_SPEED_DEFAULT; // Current homing speed

const int AUTO_MODE_SPEED_DEFAULT = 800;   // Speed in auto mode (steps/sec)
const int AUTO_MODE_SPEED_MIN = 50;        // Minimum auto speed
const int AUTO_MODE_SPEED_MAX = 10000;     // Maximum auto speed
int autoModeSpeed = AUTO_MODE_SPEED_DEFAULT; // Current auto speed

const int MANUAL_MODE_SPEED_DEFAULT = 2000;  // Speed in manual mode (steps/sec)
const int MANUAL_MODE_SPEED_MIN = 100;       // Minimum manual speed
const int MANUAL_MODE_SPEED_MAX = 5000;      // Maximum manual speed
int manualModeSpeed = MANUAL_MODE_SPEED_DEFAULT; // Current manual speed

// DYNAMIC SPEED ADJUSTMENT (based on error)
const double LARGE_ERROR_THRESHOLD = 0.5;    // Voltage error for fast speed
const double MEDIUM_ERROR_THRESHOLD = 0.2;   // Voltage error for medium speed
const int FAST_SPEED_MULTIPLIER = 3;         // Multiplier for large errors
const int MEDIUM_SPEED_MULTIPLIER = 2;       // Multiplier for medium errors

long fullTravelSteps = 0;  // Total steps from left to right limit (calibrated)

// MANUAL CONTROL SETTINGS
const int STEP_MANUAL_FACTOR_DEFAULT = 50;   // Steps per encoder click
const int STEP_MANUAL_FACTOR_MIN = 1;        // Minimum steps/click
const int STEP_MANUAL_FACTOR_MAX = 1000;     // Maximum steps/click
int stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT; // Current value

const int STEP_MANUAL_FACTOR_SW_DEFAULT = 10;  // Steps per button press
const int STEP_MANUAL_FACTOR_SW_MIN = 1;       // Minimum steps/button
const int STEP_MANUAL_FACTOR_SW_MAX = 1000;    // Maximum steps/button
int stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT; // Current value

// STEPPER ACCELERATION
const int STEPPER_ACCELERATION_DEFAULT = 1000;  // Acceleration (steps/sec²)
const int STEPPER_ACCELERATION_MIN = 1;         // Minimum acceleration
const int STEPPER_ACCELERATION_MAX = 10000;     // Maximum acceleration
int stepperAcceleration = STEPPER_ACCELERATION_DEFAULT; // Current acceleration

// BUTTON SETTINGS
const int BUTTON_HOLD_INTERVAL_DEFAULT = 500;  // Button hold time for action (ms)
const int BUTTON_HOLD_INTERVAL_MIN = 50;       // Minimum hold time
const int BUTTON_HOLD_INTERVAL_MAX = 2000;     // Maximum hold time
int buttonHoldInterval = BUTTON_HOLD_INTERVAL_DEFAULT; // Current hold time

// DYNAMIC SPEED FACTOR (scales all speeds)
const float DYNAMIC_SPEED_FACTOR_DEFAULT = 1.0;  // Global speed multiplier
const float DYNAMIC_SPEED_FACTOR_MIN = 0.1;      // Minimum speed factor
const float DYNAMIC_SPEED_FACTOR_MAX = 1.0;      // Maximum speed factor
float dynamicSpeedFactor = DYNAMIC_SPEED_FACTOR_DEFAULT; // Current factor

// CENTERING SPEEDS
const int CENTER_MOVE_SPEED_DEFAULT = 800;   // Initial centering speed
const int CENTER_MOVE_SPEED_MIN = 50;        // Minimum center speed
const int CENTER_MOVE_SPEED_MAX = 5000;      // Maximum center speed
int centerMoveSpeed = CENTER_MOVE_SPEED_DEFAULT; // Current center speed

const int CENTER_COARSE_FINE_SPEED_DEFAULT = 200;  // Coarse centering speed
const int CENTER_COARSE_FINE_SPEED_MIN = 50;       // Minimum coarse speed
const int CENTER_COARSE_FINE_SPEED_MAX = 1000;     // Maximum coarse speed
int centerCoarseFineSpeed = CENTER_COARSE_FINE_SPEED_DEFAULT; // Current coarse speed

const int CENTER_FINE_SPEED_DEFAULT = 50;    // Fine centering speed
const int CENTER_FINE_SPEED_MIN = 10;        // Minimum fine speed
const int CENTER_FINE_SPEED_MAX = 1000;      // Maximum fine speed
int centerFineSpeed = CENTER_FINE_SPEED_DEFAULT; // Current fine speed

// CENTERING SWEEP DISTANCES (For 5-7 step backlash systems)
const long CENTERING_BACK_OFF_STEPS = 15;     // Steps to move back after detection
const long CENTERING_FORWARD_SWEEP_STEPS = 20; // Steps to sweep forward to verify

unsigned long startupCompletionTimer = 0;
bool startupShowingMessage = false;

unsigned long centeringCompletionTimer = 0;
bool centeringShowingMessage = false;

const long MAX_MOVE_WITHOUT_CALIB = 5000;  // Max steps to move without calibration

// ====================================================================
// SYSTEM STATE VARIABLES (DO NOT MODIFY)
// ====================================================================
bool isManualMode = false;                    // Current operation mode
const bool INVERT_LED_LOGIC = false;          // LED polarity

unsigned long lastFullDisplayRefresh = 0;     // Timer for display updates
const unsigned long FULL_DISPLAY_REFRESH_INTERVAL = 500; // Full update interval (ms)

bool startupHomingComplete = false;           // Startup homing flag
bool quickHomingRequired = true;              // Quick homing needed
bool showVoltageBarGraph = false;             // Display mode flag

unsigned long centeringStartTime = 0;         // Centering timer
bool limitHitDuringCentering = false;         // Centering safety flag
bool justFinishedCentering = false;           // Centering completion flag
unsigned long centeringFinishTime = 0;        // Centering finish timestamp
bool justFinishedHoming = false;              // Homing completion flag

char lastVoltageStr[7] = "0.00";  // Track last displayed voltage
long lastDisplayedSteps = -999999; // Track last displayed steps

// ====================================================================
// HARDWARE PIN DEFINITIONS (MATCH YOUR WIRING)
// ====================================================================
LiquidCrystal lcd(33, 35, 37, 39, 41, 43);   // LCD pins
Encoder myEnc(2, 3);                          // Rotary encoder pins
#define ENCODER_SW 4                          // Encoder switch pin
#define STEP_PIN 12                           // Stepper step pin
#define DIRECTION_PIN 11                      // Stepper direction pin
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);
#define C0_PIN 13                             // Motor current control 0
#define C1_PIN 10                             // Motor current control 1
#define LEFT_LIMIT_PIN 45                     // Left limit switch
#define RIGHT_LIMIT_PIN 47                    // Right limit switch
#define CENTER_LIMIT_PIN 49                   // Center sensor
#define LIMIT_ACTIVE_HIGH true                // Limit switch logic
#define LEFT_BUTTON_PIN 29                    // Left manual button
#define RIGHT_BUTTON_PIN 31                   // Right manual button
#define AUTO_BUTTON_PIN 27                    // Auto mode button
#define SET_BUTTON_PIN 26                     // Set button
#define LEFT_LED_PIN 48                       // Left LED
#define RIGHT_LED_PIN 50                      // Right LED
#define MODE_LED_PIN 46                       // Mode LED
#define SET_LED_PIN 44                        // Set LED
#define CENTER_LED_PIN 52                     // Center LED

/// ====================================================================
// STATE MACHINE ENUMS (MOVED UP - Must be declared before function prototypes)
// ====================================================================
enum CenteringState {
  CENTERING_IDLE,
  CENTERING_MOVE_TO_CENTER,
  CENTERING_VERIFY_TRANSITION,
  CENTERING_FINAL_STOP,
  CENTERING_COMPLETE
};

enum HomingState { 
  HOMING_IDLE, 
  HOMING_INIT, 
  HOMING_LEFT, 
  HOMING_LEFT_WAIT,          // Wait after left limit found
  HOMING_RIGHT, 
  HOMING_RIGHT_WAIT,         // Wait after right limit found
  HOMING_CENTER, 
  HOMING_COMPLETE_WAIT,      // Wait before finishing
  HOMING_COMPLETE_FINAL,     // Final cleanup
  HOMING_COMPLETE 
};

enum CalibState { 
  CALIB_IDLE, 
  CALIB_INIT, 
  CALIB_MOVE_LEFT, 
  CALIB_LEFT_CONFIRM, 
  CALIB_MOVE_RIGHT, 
  CALIB_RIGHT_CONFIRM 
};

enum MenuState { 
  NORMAL, 
  MENU_BROWSE, 
  MENU_EDIT, 
  MENU_CALIB, 
  MENU_DIGIT_EDIT, 
  MENU_HOMING_SUB, 
  MENU_MOTOR_CURRENT_SUB, 
  MENU_EXIT_CONFIRM 
};

enum MotorCurrentLevel {
  CURRENT_0_PERCENT, 
  CURRENT_50_80_PERCENT, 
  CURRENT_100_PERCENT, 
  CURRENT_120_150_PERCENT 
};

// ====================================================================
//  STARTUP STATE MACHINE - HANDLES LIMIT SWITCHES
// ====================================================================

enum StartupState {
  STARTUP_IDLE,
  STARTUP_CHECK_LIMITS,        // Check if at limit switch
  STARTUP_ESCAPE_LIMIT,        // Move away from limit
  STARTUP_SEEK_TRANSITION,
  STARTUP_BACK_OFF,
  STARTUP_VERIFY_TRANSITION,
  STARTUP_SET_CENTER,
  STARTUP_HOMING_FALLBACK,
  STARTUP_COMPLETE
};

StartupState startupState = STARTUP_IDLE;
bool startupSearchDone = false;
bool startupCenterFound = false;
long startupSearchStartPos = 0;
int startupSearchDirection = 1;  // 1 = right, -1 = left
bool startupEscapingLimit = false;

enum StartupMode {
  STARTUP_AUTO,     // Always start in Automatic mode
  STARTUP_MANUAL,   // Always start in Manual mode
  STARTUP_LAST      // Remember last state before power loss
};

StartupMode startupMode = STARTUP_MANUAL;  // Default startup mode
bool lastPowerStateManual = false;       // Remember last mode before power loss

// ====================================================================
// SETTINGS STRUCTURE (EEPROM storage)
// ====================================================================
struct Settings {
  float Kp;
  float Ki;
  float Kd;
  int pidDirection;
  float setPoint;
  float deadband;
  bool isManualMode;
  int stepperMinSpeed;
  int stepperMaxSpeed;
  int homingSpeed;
  int autoModeSpeed;
  long fullTravelSteps;
  int stepManualFactor;
  int stepManualFactorSW;
  int stepperAcceleration;
  int buttonHoldInterval;
  float dynamicSpeedFactor;
  MotorCurrentLevel motorCurrentLevel;
  int centerMoveSpeed;
  int centerFineSpeed;
  int centerCoarseFineSpeed;
  StartupMode startupMode;
  bool lastPowerStateManual;
  uint16_t checksum;
};
const int EEPROM_ADDRESS = 0;

// ====================================================================
// FUNCTION PROTOTYPES
// ====================================================================

// Dual-loop display functions
void buildVoltageBarGraph(char* buffer);
void buildPositionGraph(char* buffer);
void updateLcdNonBlocking();
void setLcdContent(const char* line0, const char* line1);
void quickUpdateDisplayBuffered();

// Main system functions
void handleMenuSystem();
void runCalibration();
void handleSetButton();
void handleAutoButton();
void handleCenterButton();
void handleManualButtons();
void runHomingSequence();
void startSmartCentering();
void runSmartCentering();
void updateVoltageReading();
void setStepperSpeedsForMode();

// Display functions
void drawMenu();
void drawCalibMenu();
void drawMotorCurrentSubmenu();
void drawHomingSubmenu();
void showMessageTimed(const char* line1, const char* line2, unsigned long duration);
void lcdPrint_P(const char* str);

// Configuration functions
void saveSettings();
void loadSettings();
void initMenuValues();
void setMotorCurrentLevel(MotorCurrentLevel level);
void createBarGraphChars();

// Menu helper functions
long getMenuValueInt(int idx);
void adjustParameter(int delta);
void getMenuValueString(int idx, char* buffer);
bool isNumericItem(int idx);
bool isToggleItem(int idx);
void parseEditedValue(int idx);

// Sensor and limit functions
bool updateCenterSensor();
bool updateLimitSwitch(int pin, unsigned long& debounceTime, bool& lastState);
void updateDirectionLEDsFromVelocity(float speed);
void updateSetLedBlink();

// Utility functions
bool isValidFloat(float val, float minVal, float maxVal);
bool isValidInt(long val, long minVal, long maxVal);
bool isSafeToMove(long targetPos);
uint16_t calculateChecksum(Settings& settings);

// ====================================================================
// CONTROL SYSTEM VARIABLES
// ====================================================================
double input = 0;              // PID input (sensor voltage)
double displayInput = 4.25;    // Smoothed sensor voltage for display
double output = 0;             // PID output (stepper speed)
bool sensorFault = false;      // Sensor fault flag
int sensorFaultCounter = 0;    // Filter for transient faults
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, pidDirection);

long currentPosition = 0;       // Current stepper position
unsigned long lastUpdateTime = 0; // Last update timestamp

// State machine variables
HomingState homingState = HOMING_IDLE;
bool homingDone = false;
bool homeToLeft = true;
unsigned long homingStartTime = 0;
const unsigned long HOMING_TIMEOUT_MS = 60000;

CalibState calibState = CALIB_IDLE;
unsigned long calibTimer = 0;
const int calibSpeed = 200;
const unsigned long CALIB_TIMEOUT_MS = 60000;

CenteringState centeringState = CENTERING_IDLE;
int lastCenterSensorState = HIGH;
int currentCenterSensorState = HIGH;
unsigned long centerSensorDebounceTime = 0;
const unsigned long CENTER_SENSOR_DEBOUNCE_MS = 10;
bool centeringActive = false;

MenuState menuState = NORMAL;

// Menu system variables
int currentMenuItem = 0;
int menuTopItem = 0;
const int LCD_MENU_ROWS = 4;
const char* menuItems[] = {
  "PID Kp", "PID Ki", "PID Kd", "PID Direction", "Sensor SetPoint",
  "Deadband", "Operation Mode", "Min Speed", "Max Speed",
  "Homing Speed", "Auto Speed", "Calib Steps", "Manual Factor",
  "Manual Fac SW", "Manual Speed", "Acceleration", "Manual Btn Hold tm", "Dyn Speed %","Center Speed",
  "Mot. Current", "Startup Mode", "Home...", "EXIT MENU"
};
const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);

long lastMenuValues[numMenuItems];
int lastMenuTopItem = -1;
int lastCurrentMenuItem = -1;
int lastCalibMenuItem = -1;

// Calibration menu
int currentCalibMenuItem = 0;
const char* calibMenuItems[] = {"Continue", "Cancel"};
const int numCalibMenuItems = sizeof(calibMenuItems) / sizeof(calibMenuItems[0]);

// Motor current submenu
const char* motorCurrentSubmenuItems[] = {"0%", "50-80%", "100%", "120-150%", "Back"};
const int numMotorCurrentSubmenuItems = sizeof(motorCurrentSubmenuItems) / sizeof(motorCurrentSubmenuItems[0]);
int currentMotorCurrentSubMenuItem = 0;
int lastMotorCurrentSubMenuItem = -1;
MotorCurrentLevel currentMotorLevel = CURRENT_100_PERCENT;

// Homing submenu
const char* homingSubmenuItems[] = {"Home Left", "Home Right", "Back"};
const int numHomingSubmenuItems = sizeof(homingSubmenuItems) / sizeof(homingSubmenuItems[0]);
int currentHomingSubMenuItem = 0;
int lastHomingSubMenuItem = -1;

// Digit editing variables
char editValueStr[24];
int currentDigitPos = 0;
unsigned long blinkTimer = 0;
bool blinkState = true;
const unsigned long BLINK_INTERVAL = 500;

// Button timing variables
unsigned long setButtonDownTime = 0;
bool setButtonPrevState = HIGH;
const unsigned long SET_BUTTON_LONG_PRESS_MS = 3000;

unsigned long autoButtonDownTime = 0;
bool autoButtonPrevState = HIGH;
const unsigned long AUTO_BUTTON_DEBOUNCE_MS = 100;

unsigned long centerButtonDownTime = 0;
bool centerButtonPrevState = HIGH;
const unsigned long CENTER_BUTTON_DEBOUNCE_MS = 100;

// LED blinking
bool setLedBlinkError = false;
unsigned long setLedBlinkTimer = 0;

// Encoder and display timing
long lastHandledEncoderPos = 0;
unsigned long previousMillis = 0;
const long interval = 20;

// Voltage reading timing
unsigned long voltageReadTimer = 0;
const unsigned long VOLTAGE_READ_INTERVAL_AUTO = 5;
const unsigned long VOLTAGE_READ_INTERVAL_MANUAL = 50;

// Display update timing
long lastDisplayedPosition = -1;
bool lastPosGraphDirectionLeft = true; // Tracks last direction used in bargraph
bool lastVoltageGraphDirectionLeft = true; // Tracks if last voltage was < center
unsigned long lastPositionUpdate = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 100;

// Encoder button timing
unsigned long encoderButtonDownTime = 0;
bool encoderButtonPrevState = HIGH;
const unsigned long LONG_PRESS_MS = 1000;
const unsigned long SHORT_PRESS_MS = 50;

// Limit switch debouncing
unsigned long leftLimitDebounceTime = 0;
unsigned long rightLimitDebounceTime = 0;
bool leftLimitState = LOW;
bool rightLimitState = LOW;

// Display lockout
unsigned long displayLockoutUntil = 0;
const unsigned long DISPLAY_LOCKOUT_MS = 500;

// Message display
unsigned long messageDisplayStartTime = 0;
bool messageDisplayActive = false;

// Centering variables
long verifiedTransitionPos = -1;
bool returnedToCenter = false;
bool approachingFromLeft = false;

// Fast display mode
const unsigned long BARGRAPH_UPDATE_INTERVAL = 100;
unsigned long lastBargraphUpdate = 0;

// Add to user-adjustable parameters section
const int STARTUP_CENTERING_SPEED = 1000;  // Fast speed for startup search
const int STARTUP_SEARCH_TIMEOUT = 10000;  // 10 seconds max for startup search
const long STARTUP_SEARCH_RANGE = 5000;    // Max steps to search before giving up

// ====================================================================
// CUSTOM LCD BAR GRAPH CHARACTERS
// ====================================================================

// Patterns for filling from RIGHT to LEFT (used for left side of screen)
const uint8_t fillPatternsRTL[5] = {
  0b00000, // 0 pixels
  0b00001, // 1 pixel
  0b00011, // 2 pixels
  0b00111, // 3 pixels
  0b01111  // 4 pixels
};

// Patterns for filling from LEFT to RIGHT (used for right side of screen)
const uint8_t fillPatternsLTR[5] = {
  0b00000, // 0 pixels
  0b10000, // 1 pixel
  0b11000, // 2 pixels
  0b11100, // 3 pixels
  0b11110  // 4 pixels
};

// Character slot assignments
#define CHAR_VOLT_PARTIAL 1
#define CHAR_POS_PARTIAL  2
#define CHAR_FULL_BLOCK   3
#define CHAR_CENTER_LEFT  4
#define CHAR_CENTER_RIGHT 5

// Static bitmaps
const uint8_t bitmapFullBlock[8]   = {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111};
const uint8_t bitmapCenterLeft[8]  = {0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011};
const uint8_t bitmapCenterRight[8] = {0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000};


// ====================================================================
// DUAL-LOOP IMPLEMENTATION FUNCTIONS
// ====================================================================

// Hardware Timer Interrupt for Stepper Control
void stepperTimerISR() {
  stepperTimerFlag = true;
  stepperInterruptCounter++;
  
  // Run stepper motor in interrupt context
  // This ensures stepper pulses happen at exact intervals
  if (runStepperInInterrupt && menuState == NORMAL && !centeringActive) {
    // INSTANT LIMIT PROTECTION:
    // Check limit switches at 20kHz for sub-millisecond response.
    // digitalRead is fast enough for 50us interval on Mega2560.
    bool leftLimit = (digitalRead(LEFT_LIMIT_PIN) == (LIMIT_ACTIVE_HIGH ? HIGH : LOW));
    bool rightLimit = (digitalRead(RIGHT_LIMIT_PIN) == (LIMIT_ACTIVE_HIGH ? HIGH : LOW));

    float currentVel = stepper.speed();

    // Only allow pulse generation if NOT hitting a limit in the direction of travel
    if ((leftLimit && currentVel < -0.1) || (rightLimit && currentVel > 0.1)) {
        // Block stepper.run() to stop pulses instantly
    } else {
        stepper.run();
    }
  }
}

// Non-blocking LCD State Machine
void updateLcdNonBlocking() {
  if (!lcdNeedsUpdate && !lcdUpdateInProgress) return;
  
  unsigned long now = micros();
  
  switch(lcdState) {
    case LCD_IDLE:
      if (lcdNeedsUpdate) {
        // Check what needs to be updated using memcmp to support character code 0
        bool line0Changed = (memcmp(lcdLine0, lcdNewLine0, 20) != 0);
        bool line1Changed = (memcmp(lcdLine1, lcdNewLine1, 20) != 0);
        
        if (line0Changed || line1Changed) {
          lcdUpdateInProgress = true;
          lcdCurrentRow = 0;
          lcdCurrentCol = 0;
          lcdState = LCD_SETTING_CURSOR;
          lcdStateTimer = now;
        } else {
          lcdNeedsUpdate = false;
        }
      }
      break;
      
    case LCD_SETTING_CURSOR:
      if (now - lcdStateTimer >= LCD_CHAR_DELAY) {
        lcd.setCursor(lcdCurrentCol, lcdCurrentRow);
        lcdState = LCD_WRITING_CHAR;
        lcdStateTimer = now;
      }
      break;
      
    case LCD_WRITING_CHAR: {
      if (now - lcdStateTimer >= LCD_CHAR_DELAY) {
        char* currentLine = (lcdCurrentRow == 0) ? lcdLine0 : lcdLine1;
        char* newLine = (lcdCurrentRow == 0) ? lcdNewLine0 : lcdNewLine1;
        
        // Write character if changed
        if (currentLine[lcdCurrentCol] != newLine[lcdCurrentCol]) {
          lcd.write(newLine[lcdCurrentCol]);
          currentLine[lcdCurrentCol] = newLine[lcdCurrentCol];
        }
        
        // Move to next position
        lcdCurrentCol++;
        if (lcdCurrentCol >= 20) {
          lcdCurrentCol = 0;
          lcdCurrentRow++;
          if (lcdCurrentRow >= 2) {
            // Update complete
            lcdUpdateInProgress = false;
            lcdNeedsUpdate = false;
            lcdState = LCD_IDLE;
          } else {
            lcdState = LCD_SETTING_CURSOR;
          }
        } else {
          lcdState = LCD_SETTING_CURSOR;
        }
        lcdStateTimer = now;
      }
      break;
    }
  }
}

// Set LCD content (buffered, non-blocking)
void setLcdContent(const char* line0, const char* line1) {
  // Safe copy with space padding to support character code 0 and prevent reading past short strings
  const char* p0 = line0;
  const char* p1 = line1;

  for (int i = 0; i < 20; i++) {
    if (p0 && *p0 != '\0') lcdNewLine0[i] = *p0++;
    else lcdNewLine0[i] = ' ';

    if (p1 && *p1 != '\0') lcdNewLine1[i] = *p1++;
    else lcdNewLine1[i] = ' ';
  }

  lcdNewLine0[20] = '\0';
  lcdNewLine1[20] = '\0';
  
  // Mark as needing update
  lcdNeedsUpdate = true;
}

// ====================================================================
//  quickUpdateDisplayBuffered()
// ====================================================================

// Fast display update using buffers (no direct LCD calls)
void quickUpdateDisplayBuffered() {
    static unsigned long lastUpdate = 0;
    unsigned long now = millis();
    
    // Update at 10Hz in auto mode, 5Hz in manual
    unsigned long updateInterval = isManualMode ? 200 : 100;
    if (now - lastUpdate < updateInterval) return;
    lastUpdate = now;

    // Build line 0 based on display mode
    char line0[21];
    char line1[21];
    
    // Clear buffers completely
    memset(line0, ' ', 20);
    memset(line1, ' ', 20);
    line0[20] = '\0';
    line1[20] = '\0';

    if (showVoltageBarGraph) {
        // === VOLTAGE BARGRAPH ===
        const float VOLTAGE_CENTER = 4.25;
        const float VOLTAGE_HYSTERESIS = 0.02; // 20mV hysteresis

        static bool isVoltageLeftOfCenter = true;
        if (isVoltageLeftOfCenter) {
            if (displayInput > VOLTAGE_CENTER + VOLTAGE_HYSTERESIS) isVoltageLeftOfCenter = false;
        } else {
            if (displayInput < VOLTAGE_CENTER - VOLTAGE_HYSTERESIS) isVoltageLeftOfCenter = true;
        }

        if (isVoltageLeftOfCenter != lastVoltageGraphDirectionLeft) {
            lastVoltageGraphDirectionLeft = isVoltageLeftOfCenter;
            lastDisplayedPosition = -1;
        }

        buildVoltageBarGraph(line0);
    } else {
        // === TEXT DISPLAY ===
        char voltageStr[7];
        if (displayInput < 0.0 || displayInput > 6.0) {
            strncpy(voltageStr, "x.xx", 7);
        } else {
            dtostrf(displayInput, 4, 2, voltageStr);
        }
        voltageStr[6] = '\0';
        
        const char* modeStr = isManualMode ? "Man" : "Auto";
        char posStr[9];
        snprintf(posStr, sizeof(posStr), "P:%ld", currentPosition);
        posStr[8] = '\0';
        
        // Build string with proper spacing
        int baseLen = snprintf(line0, 21, "S:%sV %s", voltageStr, modeStr);
        int spaceAvailable = 20 - baseLen;
        
        if (spaceAvailable >= strlen(posStr)) {
            // Add spacing
            for (int i = baseLen; i < 20 - strlen(posStr); i++) {
                line0[i] = ' ';
            }
            // Add position at end
            strncpy(line0 + (20 - strlen(posStr)), posStr, strlen(posStr));
        }
        line0[20] = '\0';
    }

    // === POSITION BARGRAPH ===
    static bool isLeftOfCenter = true;
    if (fullTravelSteps > 0) {
        long centerPos = fullTravelSteps / 2;
        long posHysteresis = 10; // 10 steps hysteresis

        if (isLeftOfCenter) {
            if (currentPosition > centerPos + posHysteresis) isLeftOfCenter = false;
        } else {
            if (currentPosition < centerPos - posHysteresis) isLeftOfCenter = true;
        }
    }

    if (isLeftOfCenter != lastPosGraphDirectionLeft) {
        lastPosGraphDirectionLeft = isLeftOfCenter;
        lastDisplayedPosition = -1;
    }

    buildPositionGraph(line1);
    line1[20] = '\0';

    setLcdContent(line0, line1);
}

// ====================================================================
// MODIFIED BARGRAPH FUNCTIONS - FILL FROM CENTER MARKERS
// ====================================================================

void buildVoltageBarGraph(char* buffer) {
    memset(buffer, ' ', 20);
    buffer[20] = '\0';

    const float VOLTAGE_MIN = 2.48;
    const float VOLTAGE_MAX = 6.0;
    const float VOLTAGE_CENTER = 4.25;

    if (displayInput < VOLTAGE_MIN || displayInput > VOLTAGE_MAX) {
        const char* msg = "Sensor Out of Range";
        for (int i = 0; i < 20 && msg[i]; i++) buffer[i] = msg[i];
        return;
    }

    // Calculate total pixels from center (Max 48 pixels growth per side)
    float distanceFromCenter = fabs(displayInput - VOLTAGE_CENTER);
    float halfRange = (VOLTAGE_MAX - VOLTAGE_MIN) / 2.0;
    int totalPixels = (int)map(distanceFromCenter * 1000, 0, halfRange * 1000, 0, 48);
    totalPixels = constrain(totalPixels, 0, 48);

    bool isLeftOfCenter_disp = lastVoltageGraphDirectionLeft;

    if (isLeftOfCenter_disp) {
        // FILL LEFT: Chars 9 down to 0
        int growPixels = totalPixels;
        
        // Char 9 (Base 2 pixels)
        int c9 = growPixels > 3 ? 3 : growPixels;
        if (c9 == 0) buffer[9] = CHAR_CENTER_LEFT;
        else if (c9 == 3) buffer[9] = CHAR_FULL_BLOCK;
        else {
            updateDynamicChar(CHAR_VOLT_PARTIAL, 2 + c9, false);
            buffer[9] = CHAR_VOLT_PARTIAL;
        }
        
        growPixels -= c9;
        for (int c = 8; c >= 0; c--) {
            if (growPixels <= 0) break;
            if (growPixels >= 5) {
                buffer[c] = CHAR_FULL_BLOCK;
                growPixels -= 5;
            } else {
                updateDynamicChar(CHAR_VOLT_PARTIAL, growPixels, false);
                buffer[c] = CHAR_VOLT_PARTIAL;
                growPixels = 0;
            }
        }
        buffer[10] = CHAR_CENTER_RIGHT;
    } else {
        // FILL RIGHT: Chars 10 up to 19
        int growPixels = totalPixels;
        
        // Char 10 (Base 2 pixels)
        int c10 = growPixels > 3 ? 3 : growPixels;
        if (c10 == 0) buffer[10] = CHAR_CENTER_RIGHT;
        else if (c10 == 3) buffer[10] = CHAR_FULL_BLOCK;
        else {
            updateDynamicChar(CHAR_VOLT_PARTIAL, 2 + c10, true);
            buffer[10] = CHAR_VOLT_PARTIAL;
        }
        
        growPixels -= c10;
        for (int c = 11; c < 20; c++) {
            if (growPixels <= 0) break;
            if (growPixels >= 5) {
                buffer[c] = CHAR_FULL_BLOCK;
                growPixels -= 5;
            } else {
                updateDynamicChar(CHAR_VOLT_PARTIAL, growPixels, true);
                buffer[c] = CHAR_VOLT_PARTIAL;
                growPixels = 0;
            }
        }
        buffer[9] = CHAR_CENTER_LEFT;
    }
}

// ====================================================================
// MODIFIED buildPositionGraph() - WITH 2-PIXEL CENTER LINES
// ====================================================================

void buildPositionGraph(char* buffer) {
    memset(buffer, ' ', 20);
    buffer[20] = '\0';
    
    if (fullTravelSteps <= 0) {
        const char* msg = "No Calibration";
        for (int i = 0; i < 20 && msg[i]; i++) buffer[i] = msg[i];
        return;
    }
    
    if (leftLimitState) {
        const char* msg = "<<< LIMITA STANGA";
        for (int i = 0; i < 20 && msg[i]; i++) buffer[i] = msg[i];
        return;
    }
    
    if (rightLimitState) {
        const char* msg = "LIMITA DREAPTA >>>";
        for (int i = 0; i < 20 && msg[i]; i++) buffer[i] = msg[i];
        return;
    }
    
    long centerPos = fullTravelSteps / 2;
    if (centerPos <= 0) {
        const char* msg = "Center:0";
        for (int i = 0; i < 20 && msg[i]; i++) buffer[i] = msg[i];
        return;
    }

    // Calculate total pixels from center (Max 48 pixels growth per side)
    long distanceFromCenter = abs(currentPosition - centerPos);
    int totalPixels = (int)map(distanceFromCenter, 0, centerPos, 0, 48);
    totalPixels = constrain(totalPixels, 0, 48);

    bool isLeftOfCenter = currentPosition < centerPos;

    if (isLeftOfCenter) {
        // FILL LEFT: Chars 9 down to 0
        int growPixels = totalPixels;
        
        // Char 9 (Base 2 pixels)
        int c9 = growPixels > 3 ? 3 : growPixels;
        if (c9 == 0) buffer[9] = CHAR_CENTER_LEFT;
        else if (c9 == 3) buffer[9] = CHAR_FULL_BLOCK;
        else {
            updateDynamicChar(CHAR_POS_PARTIAL, 2 + c9, false);
            buffer[9] = CHAR_POS_PARTIAL;
        }
        
        growPixels -= c9;
        for (int c = 8; c >= 0; c--) {
            if (growPixels <= 0) break;
            if (growPixels >= 5) {
                buffer[c] = CHAR_FULL_BLOCK;
                growPixels -= 5;
            } else {
                updateDynamicChar(CHAR_POS_PARTIAL, growPixels, false);
                buffer[c] = CHAR_POS_PARTIAL;
                growPixels = 0;
            }
        }
        buffer[10] = CHAR_CENTER_RIGHT;
    } else {
        // FILL RIGHT: Chars 10 up to 19
        int growPixels = totalPixels;
        
        // Char 10 (Base 2 pixels)
        int c10 = growPixels > 3 ? 3 : growPixels;
        if (c10 == 0) buffer[10] = CHAR_CENTER_RIGHT;
        else if (c10 == 3) buffer[10] = CHAR_FULL_BLOCK;
        else {
            updateDynamicChar(CHAR_POS_PARTIAL, 2 + c10, true);
            buffer[10] = CHAR_POS_PARTIAL;
        }
        
        growPixels -= c10;
        for (int c = 11; c < 20; c++) {
            if (growPixels <= 0) break;
            if (growPixels >= 5) {
                buffer[c] = CHAR_FULL_BLOCK;
                growPixels -= 5;
            } else {
                updateDynamicChar(CHAR_POS_PARTIAL, growPixels, true);
                buffer[c] = CHAR_POS_PARTIAL;
                growPixels = 0;
            }
        }
        buffer[9] = CHAR_CENTER_LEFT;
    }
}



// ====================================================================
//  DISPLAY BUFFER CLEAR FUNCTION
// ====================================================================
void clearAllDisplayBuffers() {
    // Clear physical LCD
    lcd.clear();
    
    // Reset all LCD buffers
    memset(lcdLine0, ' ', 20);
    memset(lcdLine1, ' ', 20);
    memset(lcdNewLine0, ' ', 20);
    memset(lcdNewLine1, ' ', 20);
    lcdLine0[20] = '\0';
    lcdLine1[20] = '\0';
    lcdNewLine0[20] = '\0';
    lcdNewLine1[20] = '\0';
    
    // Reset LCD state machine
    lcdNeedsUpdate = false;
    lcdUpdateInProgress = false;
    lcdState = LCD_IDLE;
    
    // Reset all tracking variables
    lastDisplayedPosition = -1;
    lastDisplayedSteps = -999999;
    strcpy(lastVoltageStr, "");
    lastFullDisplayRefresh = 0;
    lastBargraphUpdate = 0;
    lastPositionUpdate = 0;
    
    // Reset direction tracking for bargraphs
    lastPosGraphDirectionLeft = true;
    lastVoltageGraphDirectionLeft = true;
}

// ====================================================================
// NON-BLOCKING MESSAGE DISPLAY
// ====================================================================

/**
 * Displays a timed message on the LCD without blocking execution
 * @param line1 First line of message (or NULL)
 * @param line2 Second line of message (or NULL)
 * @param duration How long to display message (ms)
 */
void showMessageTimed(const char* line1, const char* line2, unsigned long duration) {
  // Use direct LCD for messages (they're infrequent)
  lcd.clear();
  if (line1) {
    lcd.setCursor(0, 0);
    lcdPrint_P(line1);
  }
  if (line2) {
    lcd.setCursor(0, 1);
    lcdPrint_P(line2);
  }
  messageDisplayStartTime = millis();
  messageDisplayActive = true;
  displayLockoutUntil = millis() + duration;
  
  // Reset display tracking to force update after message
  lastDisplayedPosition = -1;
  strcpy(lastVoltageStr, "");
  lastDisplayedSteps = -999999;
  lastFullDisplayRefresh = 0;
  lastBargraphUpdate = 0;
}

// ====================================================================
// HELPER FUNCTIONS
// ====================================================================

/**
 * Prints a string from program memory to LCD
 * @param str String stored in PROGMEM
 */
void lcdPrint_P(const char* str) {
  char buf[21];
  strcpy_P(buf, str);
  lcd.print(buf);
}

/**
 * Validates a float value is within range and not NaN/infinite
 * @param val Value to check
 * @param minVal Minimum allowed value
 * @param maxVal Maximum allowed value
 * @return True if valid
 */
bool isValidFloat(float val, float minVal, float maxVal) {
  if (val != val) return false; // Check for NaN
  if (val == INFINITY || val == -INFINITY) return false;
  return (val >= minVal && val <= maxVal);
}

/**
 * Validates an integer value is within range
 * @param val Value to check
 * @param minVal Minimum allowed value
 * @param maxVal Maximum allowed value
 * @return True if valid
 */
bool isValidInt(long val, long minVal, long maxVal) {
  return (val >= minVal && val <= maxVal);
}

/**
 * Checks if a target position is safe to move to
 * @param targetPos Target position in steps
 * @return True if safe to move
 */
bool isSafeToMove(long targetPos) {
  // Get current position
  long currentPos = stepper.currentPosition();
  
  // Update limit switch states
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
  
  // If no calibration exists, allow limited movement for manual adjustments
  if (fullTravelSteps <= 0) {
    // Without calibration, only allow moving within reasonable bounds
    if (leftLimitState && targetPos < currentPos) return false;
    if (rightLimitState && targetPos > currentPos) return false;
    
    // Check absolute position bounds
    if (abs(targetPos) > MAX_MOVE_WITHOUT_CALIB) return false;
    
    if (abs(targetPos - currentPos) > MAX_MOVE_WITHOUT_CALIB) return false;
    return true;
  }
  
  // With calibration, use normal checks
  if (targetPos < 0) return false;
  if (targetPos > fullTravelSteps) return false;
  
  // Check if we're at a limit and trying to move into it
  if (leftLimitState && targetPos < currentPos) {
    // Trying to move left into left limit - NOT SAFE
    return false;
  }
  if (rightLimitState && targetPos > currentPos) {
    // Trying to move right into right limit - NOT SAFE
    return false;
  }
  
  // If moving away from a limit, it's safe
  if (leftLimitState && targetPos > currentPos) {
    // Moving right away from left limit - SAFE
    return true;
  }
  if (rightLimitState && targetPos < currentPos) {
    // Moving left away from right limit - SAFE
    return true;
  }
  
  // Normal movement within bounds - SAFE
  return true;
}

// ====================================================================
// EEPROM FUNCTIONS
// ====================================================================

/**
 * Calculates checksum for settings validation
 * @param settings Settings structure
 * @return Calculated checksum
 */
uint16_t calculateChecksum(Settings& settings) {
  uint16_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&settings;
  for (size_t i = 0; i < sizeof(Settings) - sizeof(uint16_t); i++) {
    checksum += ptr[i];
  }
  return checksum;
}

/**
 * Saves all settings to EEPROM
 */
void saveSettings() {
  Settings settings = {
    (float)Kp, (float)Ki, (float)Kd, pidDirection, (float)setPoint,
    (float)deadband, isManualMode, stepperMinSpeed, stepperMaxSpeed,
    homingSpeed, autoModeSpeed, fullTravelSteps, stepManualFactor,
    stepManualFactorSW, stepperAcceleration, buttonHoldInterval,
    dynamicSpeedFactor, currentMotorLevel, centerMoveSpeed, centerFineSpeed, 
    centerCoarseFineSpeed, startupMode, isManualMode, 0
  };
  settings.checksum = calculateChecksum(settings);
  wdt_reset();
  EEPROM.put(EEPROM_ADDRESS, settings);
  wdt_reset();
}

/**
 * Loads settings from EEPROM, uses defaults if checksum fails
 */
void loadSettings() {
  Settings settings;
  EEPROM.get(EEPROM_ADDRESS, settings);
  uint16_t storedChecksum = settings.checksum;
  settings.checksum = 0;
  uint16_t calculatedChecksum = calculateChecksum(settings);
  
  if (storedChecksum != calculatedChecksum) {
    // Checksum failed, use defaults
    wdt_reset();
    Kp = KP_DEFAULT;
    Ki = KI_DEFAULT;
    Kd = KD_DEFAULT;
    pidDirection = PID_DIRECTION_DEFAULT;
    setPoint = SETPOINT_DEFAULT;
    deadband = DEADBAND_DEFAULT;
    isManualMode = false;
    stepperMinSpeed = STEPPER_MIN_SPEED_DEFAULT;
    stepperMaxSpeed = STEPPER_MAX_SPEED_DEFAULT;
    homingSpeed = HOMING_SPEED_DEFAULT;
    autoModeSpeed = AUTO_MODE_SPEED_DEFAULT;
    fullTravelSteps = 0;
    stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT;
    stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT;
    stepperAcceleration = STEPPER_ACCELERATION_DEFAULT;
    buttonHoldInterval = BUTTON_HOLD_INTERVAL_DEFAULT;
    dynamicSpeedFactor = DYNAMIC_SPEED_FACTOR_DEFAULT;
    currentMotorLevel = CURRENT_100_PERCENT;
    centerMoveSpeed = CENTER_MOVE_SPEED_DEFAULT;
    centerFineSpeed = CENTER_FINE_SPEED_DEFAULT;
    centerCoarseFineSpeed = CENTER_COARSE_FINE_SPEED_DEFAULT;
    startupMode = STARTUP_AUTO;
    lastPowerStateManual = false;
  } else {
    // Checksum OK, load settings
    wdt_reset();
   
    Kp = isValidFloat(settings.Kp, KP_MIN, KP_MAX) ? settings.Kp : KP_DEFAULT;
    Ki = isValidFloat(settings.Ki, KI_MIN, KI_MAX) ? settings.Ki : KI_DEFAULT;
    Kd = isValidFloat(settings.Kd, KD_MIN, KD_MAX) ? settings.Kd : KD_DEFAULT;
    pidDirection = (settings.pidDirection == DIRECT || settings.pidDirection == REVERSE) ? settings.pidDirection : PID_DIRECTION_DEFAULT;
    setPoint = isValidFloat(settings.setPoint, SETPOINT_MIN, SETPOINT_MAX) ? settings.setPoint : SETPOINT_DEFAULT;
    deadband = isValidFloat(settings.deadband, DEADBAND_MIN, DEADBAND_MAX) ? settings.deadband : DEADBAND_DEFAULT;
    isManualMode = settings.isManualMode;
   
    wdt_reset();
   
    int loadedMinSpeed = settings.stepperMinSpeed;
    int loadedMaxSpeed = settings.stepperMaxSpeed;
    if (loadedMinSpeed > loadedMaxSpeed) {
      int temp = loadedMinSpeed;
      loadedMinSpeed = loadedMaxSpeed;
      loadedMaxSpeed = temp;
    }
   
    stepperMinSpeed = isValidInt(loadedMinSpeed, 0, STEPPER_MAX_SPEED_MAX) ? loadedMinSpeed : STEPPER_MIN_SPEED_DEFAULT;
    stepperMaxSpeed = isValidInt(loadedMaxSpeed, 0, STEPPER_MAX_SPEED_MAX) ? loadedMaxSpeed : STEPPER_MAX_SPEED_DEFAULT;
    homingSpeed = isValidInt(settings.homingSpeed, HOMING_SPEED_MIN, HOMING_SPEED_MAX) ? settings.homingSpeed : HOMING_SPEED_DEFAULT;
    autoModeSpeed = isValidInt(settings.autoModeSpeed, AUTO_MODE_SPEED_MIN, AUTO_MODE_SPEED_MAX) ? settings.autoModeSpeed : AUTO_MODE_SPEED_DEFAULT;
   
    wdt_reset();
   
    fullTravelSteps = settings.fullTravelSteps >= 0 ? settings.fullTravelSteps : 0;
    stepManualFactor = isValidInt(settings.stepManualFactor, STEP_MANUAL_FACTOR_MIN, STEP_MANUAL_FACTOR_MAX) ? settings.stepManualFactor : STEP_MANUAL_FACTOR_DEFAULT;
    stepManualFactorSW = isValidInt(settings.stepManualFactorSW, STEP_MANUAL_FACTOR_SW_MIN, STEP_MANUAL_FACTOR_SW_MAX) ? settings.stepManualFactorSW : STEP_MANUAL_FACTOR_SW_DEFAULT;
    stepperAcceleration = isValidInt(settings.stepperAcceleration, STEPPER_ACCELERATION_MIN, STEPPER_ACCELERATION_MAX) ? settings.stepperAcceleration : STEPPER_ACCELERATION_DEFAULT;
    buttonHoldInterval = isValidInt(settings.buttonHoldInterval, BUTTON_HOLD_INTERVAL_MIN, BUTTON_HOLD_INTERVAL_MAX) ? settings.buttonHoldInterval : BUTTON_HOLD_INTERVAL_DEFAULT;
   
    wdt_reset();
   
    dynamicSpeedFactor = isValidFloat(settings.dynamicSpeedFactor, DYNAMIC_SPEED_FACTOR_MIN, DYNAMIC_SPEED_FACTOR_MAX) ? settings.dynamicSpeedFactor : DYNAMIC_SPEED_FACTOR_DEFAULT;
    currentMotorLevel = settings.motorCurrentLevel;
    centerMoveSpeed = isValidInt(settings.centerMoveSpeed, CENTER_MOVE_SPEED_MIN, CENTER_MOVE_SPEED_MAX) ? settings.centerMoveSpeed : CENTER_MOVE_SPEED_DEFAULT;
    centerFineSpeed = isValidInt(settings.centerFineSpeed, CENTER_FINE_SPEED_MIN, CENTER_FINE_SPEED_MAX) ? settings.centerFineSpeed : CENTER_FINE_SPEED_DEFAULT;
    centerCoarseFineSpeed = isValidInt(settings.centerCoarseFineSpeed, CENTER_COARSE_FINE_SPEED_MIN, CENTER_COARSE_FINE_SPEED_MAX) ? settings.centerCoarseFineSpeed : CENTER_COARSE_FINE_SPEED_DEFAULT;
    startupMode = settings.startupMode;
    lastPowerStateManual = settings.lastPowerStateManual;
  }
  wdt_reset();
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetControllerDirection(pidDirection);
  digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);
  setMotorCurrentLevel(currentMotorLevel);
  wdt_reset();
}

// ====================================================================
// MOTOR CURRENT CONTROL
// ====================================================================

/**
 * Sets motor current level via C0/C1 pins
 * @param level Current level enum
 */
void setMotorCurrentLevel(MotorCurrentLevel level) {
  switch(level) {
    case CURRENT_0_PERCENT:
      digitalWrite(C0_PIN, LOW);
      digitalWrite(C1_PIN, LOW);
      break;
    case CURRENT_50_80_PERCENT:
      digitalWrite(C0_PIN, HIGH);
      digitalWrite(C1_PIN, LOW);
      break;
    case CURRENT_100_PERCENT:
      digitalWrite(C0_PIN, LOW);
      digitalWrite(C1_PIN, HIGH);
      break;
    case CURRENT_120_150_PERCENT:
      digitalWrite(C0_PIN, HIGH);
      digitalWrite(C1_PIN, HIGH);
      break;
  }
  currentMotorLevel = level;
}

/**
 * Sets stepper speeds based on current operation mode
 * PROTECTED: Uses critical sections to prevent ISR interference
 */
void setStepperSpeedsForMode() {
  noInterrupts();
  if (isManualMode) {
    stepper.setMaxSpeed(manualModeSpeed);
  } else {
    stepper.setMaxSpeed(autoModeSpeed);
  }
  stepper.setAcceleration(stepperAcceleration);
  interrupts();
}

// ====================================================================
// OPTIMIZED VOLTAGE READING - FASTER FOR DISPLAY
// ====================================================================

/**
 * Reads and filters sensor voltage with different strategies for each mode
 * Auto mode: Single fast read for quick PID response
 * Manual mode: Average 3 samples for stable display
 */
void updateVoltageReading() {
  unsigned long now = millis();
  unsigned long readInterval = isManualMode ? VOLTAGE_READ_INTERVAL_MANUAL : VOLTAGE_READ_INTERVAL_AUTO;
 
  if (now - voltageReadTimer >= readInterval) {
    voltageReadTimer = now;
   
    float Vin;
   
    if (isManualMode) {
      // Manual mode: Average 3 samples for stable display
      long sum = 0;
      for (int i = 0; i < 3; i++) {
        sum += analogRead(A0);
        delayMicroseconds(20);
      }
      float adcValue = sum / 3.0;
      if (ADC_RESOLUTION <= 0) return;  //  SAFETY CHECK
      float Vout = (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
      Vin = Vout * (R1 + R2) / R2;
    } else {
      // Auto mode: Single fast read for quick PID response
      if (ADC_RESOLUTION <= 0) return;  // SAFETY CHECK
      float adcValue = analogRead(A0);
      float Vout = (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
      Vin = Vout * (R1 + R2) / R2;
    }
   
    // Validate and update
    if (Vin >= 0.0 && Vin <= 6.0) {
      input = Vin;

      // SENSOR FAULT PROTECTION:
      // If sensor reading is at absolute extreme rails, it likely indicates a broken wire.
      // Thresholds: < 0.1V (Ground short/cut) or > 5.8V (VCC short/cut)
      if (Vin < 0.1 || Vin > 5.8) {
        if (sensorFaultCounter < 100) sensorFaultCounter++;
        if (sensorFaultCounter >= 50) sensorFault = true; // Sustained for ~250ms
      } else {
        sensorFaultCounter = 0;
        sensorFault = false;
      }

      // Update display averaged value (EMA filter for stability)
      // Time constant of ~1 second
      static unsigned long lastAvgUpdate = 0;
      unsigned long nowAvg = millis();
      if (lastAvgUpdate == 0) lastAvgUpdate = nowAvg;
      float dt = (nowAvg - lastAvgUpdate) / 1000.0;
      lastAvgUpdate = nowAvg;

      float timeConstant = 1.0; // 1 second averaging
      float alpha = dt / (timeConstant + dt);
      if (alpha > 1.0) alpha = 1.0;
      if (alpha < 0.0) alpha = 0.0;

      displayInput = (displayInput * (1.0 - alpha)) + (Vin * alpha);
    }
  }
}

// ====================================================================
// HOMING SEQUENCE
// ====================================================================

/**
 * Executes homing sequence to find limits and center
 * Non-blocking state machine implementation
 */
void runHomingSequence() {
  // CRITICAL: Disable interrupt control while homing to prevent clashes
  runStepperInInterrupt = false;

  static unsigned long messageTimer = 0;
  wdt_reset();
  unsigned long now = millis();
  bool timeout = (now - homingStartTime > HOMING_TIMEOUT_MS);
  
  switch(homingState) {
    case HOMING_INIT:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcdPrint_P(STR_HOMING);
      homingStartTime = now;
     
      if (homeToLeft) {
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to left limit"));
        stepper.setSpeed(-homingSpeed);
        homingState = HOMING_LEFT;
      } else {
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to right limit"));
        stepper.setSpeed(homingSpeed);
        homingState = HOMING_RIGHT;
      }
      break;
      
    case HOMING_LEFT:
      if (timeout) {
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_TIMEOUT);
        messageTimer = millis();
        homingState = HOMING_COMPLETE;
        break;
      }
      
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      
      if (leftLimitState) {
        // Found left limit!
        stepper.stop();
        stepper.setCurrentPosition(0);
        currentPosition = 0;
        lastDisplayedPosition = 0;
        lastDisplayedSteps = 0;
        
        // Turn off all LEDs
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        
        // Display message
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_LEFT_LIMIT);
        lcd.print(F(" "));
        
        // Start timer for non-blocking wait
        messageTimer = millis();
        homingState = HOMING_LEFT_WAIT;
      } else {
        // Still moving to left
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        stepper.runSpeed();
      }
      break;
      
    // Wait after finding left limit
    case HOMING_LEFT_WAIT:
      wdt_reset();
      
      // Non-blocking wait for 200ms
      if (millis() - messageTimer >= 200) {
        // Wait complete, now decide next action
        if (fullTravelSteps > 0) {
          // We have calibration data, move to center
          long centerPos = fullTravelSteps / 2;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcdPrint_P(STR_CENTERING);
          
          stepper.setMaxSpeed(homingSpeed);
          stepper.setAcceleration(stepperAcceleration);
          stepper.moveTo(centerPos);
          homingState = HOMING_CENTER;
        } else {
          // No calibration, just finish
          homingState = HOMING_COMPLETE;
        }
      }
      break;
      
    case HOMING_RIGHT:
      if (timeout) {
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_TIMEOUT);
        messageTimer = millis();
        homingState = HOMING_COMPLETE;
        break;
      }
      
      rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
      
      if (rightLimitState) {
        // Found right limit!
        stepper.stop();
        stepper.setCurrentPosition(fullTravelSteps);
        currentPosition = fullTravelSteps;
        lastDisplayedPosition = fullTravelSteps;
        lastDisplayedSteps = fullTravelSteps;
        
        // Turn off all LEDs
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        
        // Display message
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_RIGHT_LIMIT);
        lcd.print(F(" "));
        
        // Start timer for non-blocking wait
        messageTimer = millis();
        homingState = HOMING_RIGHT_WAIT;
      } else {
        // Still moving to right
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(CENTER_LED_PIN, LOW);
        stepper.runSpeed();
      }
      break;
      
    // Wait after finding right limit
    case HOMING_RIGHT_WAIT:
      wdt_reset();
      
      // Non-blocking wait for 200ms
      if (millis() - messageTimer >= 200) {
        // Wait complete, now decide next action
        if (fullTravelSteps > 0) {
          // We have calibration data, move to center
          long centerPos = fullTravelSteps / 2;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcdPrint_P(STR_CENTERING);
          
          stepper.setMaxSpeed(homingSpeed);
          stepper.setAcceleration(stepperAcceleration);
          stepper.moveTo(centerPos);
          homingState = HOMING_CENTER;
        } else {
          // No calibration, just finish
          homingState = HOMING_COMPLETE;
        }
      }
      break;
      
    case HOMING_CENTER:
      if (timeout) {
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_TIMEOUT);
        messageTimer = millis();
        homingState = HOMING_COMPLETE;
        break;
      }
      
      stepper.run();
      
      if (stepper.distanceToGo() == 0) {
        // Reached center position
        currentPosition = stepper.currentPosition();
        lastDisplayedPosition = currentPosition;
        lastDisplayedSteps = currentPosition;
        
        // Turn off direction LEDs, turn on center LED
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, HIGH);
        
        // Display center position
        lcd.setCursor(0, 1);
        char buffer[21];
        snprintf(buffer, sizeof(buffer), "Center: %ld", currentPosition);
        buffer[20] = '\0';
        lcd.print(buffer);
        
        // Start timer for message display
        messageTimer = millis();
        homingState = HOMING_COMPLETE_WAIT;
      }
      break;
      
    // Wait before showing completion message
    case HOMING_COMPLETE_WAIT:
      wdt_reset();
      
      // Non-blocking wait for 500ms
      if (millis() - messageTimer >= 500) {
        // Mark homing as complete
        homingDone = true;
        
        // Show completion message
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Homing Complete"));
        
        // Start timer for final message
        messageTimer = millis();
        homingState = HOMING_COMPLETE_FINAL;
      }
      break;
      
    // Final message display before returning to normal
    case HOMING_COMPLETE_FINAL:
      wdt_reset();
      
      // Non-blocking wait for 500ms
      if (millis() - messageTimer >= 500) {
        // ============================================================
        // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
        // ============================================================
        clearAllDisplayBuffers();
        
        // Mark as finished and return to idle
        homingState = HOMING_IDLE;
        justFinishedHoming = true;
        
        // Re-enable interrupt control
        runStepperInInterrupt = true;

        // Immediately update display
        quickUpdateDisplayBuffered();
      }
      break;
      
    case HOMING_COMPLETE:
      // This state is now only used for timeout/error cases
      // Just wait briefly then go to idle
      if (millis() - messageTimer >= 500) {
        homingState = HOMING_IDLE;
        runStepperInInterrupt = true;
      }
      break;
      
    default:
      // Safety fallback
      homingState = HOMING_IDLE;
      break;
  }
}

// ====================================================================
// CENTERING FUNCTIONS
// ====================================================================

/**
 * Reads center sensor with debouncing
 * @return True if sensor state changed
 */
bool updateCenterSensor() {
  unsigned long now = millis();
  int rawState = digitalRead(CENTER_LIMIT_PIN);
  
  if (rawState != lastCenterSensorState) {
    if ((now - centerSensorDebounceTime) >= CENTER_SENSOR_DEBOUNCE_MS) {
      lastCenterSensorState = rawState;
      currentCenterSensorState = rawState;
      centerSensorDebounceTime = now;
      return true; // State changed
    }
  }
  return false; // State unchanged
}

/**
 * Starts smart centering sequence
 * Verifies center position using sensor and compensates for backlash
 */
void startSmartCentering() {
  if (!homingDone) {
    setLcdContent("Home first!", "");
    delay(1000);
    return;
  }
  if (fullTravelSteps <= 0) {
    setLcdContent("Calibrate first!", "");
    delay(1000);
    return;
  }
  if (centeringActive) {
    return; // Prevent re-entry
  }

  // Reset all centering state variables
  centeringActive = true;
  centeringState = CENTERING_MOVE_TO_CENTER;
  centeringStartTime = millis();
  verifiedTransitionPos = -1;
  returnedToCenter = false;
  
  // Calculate ideal center position
  long idealCenter = fullTravelSteps / 2;
  long currentPos = stepper.currentPosition();
  
  // Determine direction of movement
  approachingFromLeft = (currentPos < idealCenter);
  
  // Set initial display
  if (approachingFromLeft) {
    setLcdContent("Centering...", "Moving Right ->");
  } else {
    setLcdContent("Centering...", "<- Moving Left");
  }

  // Move to calculated center position with HIGH speed
  stepper.enableOutputs();
  stepper.setMaxSpeed(centerMoveSpeed);
  stepper.setAcceleration(5000);  // Fast acceleration
  stepper.moveTo(idealCenter);
}

/**
* Runs smart centering state machine
* Non-blocking implementation with timeout and direction-aware limit safety
*/
void runSmartCentering() {
    if (!centeringActive) return;
    
    // Update sensors and limits EVERY TIME
    leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
    rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
    updateCenterSensor();
    
    // Timeout check
    if (millis() - centeringStartTime > 10000) {
        stepper.stop();
        centeringActive = false;
        centeringState = CENTERING_IDLE;
        setLcdContent("Center Timeout!", "");
        runStepperInInterrupt = true;
        return;
    }
    
    stepper.setAcceleration(5000);
    
    switch (centeringState) {
        case CENTERING_MOVE_TO_CENTER: {
            stepper.setMaxSpeed(centerMoveSpeed);
            stepper.run();
            
            bool sensorTransition = false;
            if (approachingFromLeft) {
                sensorTransition = (currentCenterSensorState == LOW);
            } else {
                sensorTransition = (currentCenterSensorState == HIGH);
            }
            
            if (sensorTransition || stepper.distanceToGo() == 0) {
                if (sensorTransition) {
                    verifiedTransitionPos = stepper.currentPosition();
                    
                    lcd.setCursor(0, 1);
                    lcd.print("Transition found!");
                    
                    stepper.stop();
                    delayMicroseconds(100);
                    
                    long backTarget = verifiedTransitionPos;
                    if (approachingFromLeft) {
                        backTarget -= 10;
                    } else {
                        backTarget += 10;
                    }
                    backTarget = constrain(backTarget, 0, fullTravelSteps);
                    
                    stepper.setMaxSpeed(centerCoarseFineSpeed);
                    stepper.moveTo(backTarget);
                    centeringState = CENTERING_VERIFY_TRANSITION;
                } else {
                    if (approachingFromLeft) {
                        stepper.moveTo(fullTravelSteps);
                        lcd.setCursor(0, 1);
                        lcd.print("Searching right...");
                    } else {
                        stepper.moveTo(0);
                        lcd.setCursor(0, 1);
                        lcd.print("Searching left...");
                    }
                }
            }
            break;
        }
        
        case CENTERING_VERIFY_TRANSITION: {
            stepper.run();
            
            if (stepper.distanceToGo() == 0) {
                long forwardTarget = verifiedTransitionPos;
                if (approachingFromLeft) {
                    forwardTarget += 20;
                } else {
                    forwardTarget -= 20;
                }
                forwardTarget = constrain(forwardTarget, 0, fullTravelSteps);
                
                stepper.setMaxSpeed(centerFineSpeed);
                stepper.moveTo(forwardTarget);
                centeringState = CENTERING_FINAL_STOP;
            }
            break;
        }
        
        case CENTERING_FINAL_STOP: {
            stepper.run();
            
            bool finalTransition = false;
            if (approachingFromLeft) {
                finalTransition = (currentCenterSensorState == LOW);
            } else {
                finalTransition = (currentCenterSensorState == HIGH);
            }
            
            if (finalTransition || stepper.distanceToGo() == 0) {
                if (finalTransition) {
                    verifiedTransitionPos = stepper.currentPosition();
                }
                
                stepper.stop();
                delayMicroseconds(100);
                stepper.setMaxSpeed(centerMoveSpeed);
                stepper.moveTo(verifiedTransitionPos);
                returnedToCenter = true;
                centeringState = CENTERING_COMPLETE;
            }
            break;
        }
        
        case CENTERING_COMPLETE: {
            stepper.run();
            
            if (stepper.distanceToGo() == 0 && returnedToCenter && !centeringShowingMessage) {
                myPID.SetMode(MANUAL);
                output = 0.0;
                
                long centerPos = fullTravelSteps / 2;
                
                // ============================================================
                // CRITICAL: UPDATE ALL POSITION TRACKING VARIABLES
                // ============================================================
                stepper.setCurrentPosition(centerPos);
                currentPosition = centerPos;
                lastDisplayedPosition = centerPos;
                lastDisplayedSteps = centerPos;
                // ============================================================
                
                if (!isManualMode) {
                    myPID.SetMode(AUTOMATIC);
                }
                
                // Show completion message
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Center OK!");
                lcd.setCursor(0, 1);
                lcd.print("Pos:");
                lcd.print(currentPosition);
                
                digitalWrite(CENTER_LED_PIN, HIGH);
                
                // Start non-blocking timer
                centeringCompletionTimer = millis();
                centeringShowingMessage = true;
            }
            
            // Non-blocking delay
            if (centeringShowingMessage && (millis() - centeringCompletionTimer >= 500)) {
                // ============================================================
                // CRITICAL: CLEAR DISPLAY COMPLETELY BEFORE TRANSITION
                // ============================================================
                clearAllDisplayBuffers();
                
                // Reset centering state
                centeringActive = false;
                centeringState = CENTERING_IDLE;
                verifiedTransitionPos = -1;
                returnedToCenter = false;
                justFinishedCentering = true;
                centeringFinishTime = millis();
                centeringShowingMessage = false;
                
                runStepperInInterrupt = true;
                
                // Immediately update display
                quickUpdateDisplayBuffered();
            }
            break;
        }
        
        default:
            centeringActive = false;
            centeringState = CENTERING_IDLE;
            break;
    }
}

// ====================================================================
// LED CONTROL
// ====================================================================

/**
 * Updates direction LEDs based on stepper velocity
 * @param speed Current stepper speed (positive = right, negative = left)
 */
void updateDirectionLEDsFromVelocity(float speed) {
  if (abs(speed) < 1.0) {
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
    digitalWrite(CENTER_LED_PIN, currentPosition == fullTravelSteps / 2 ? HIGH : LOW);
  } else {
    if (INVERT_LED_LOGIC) {
      digitalWrite(LEFT_LED_PIN, speed < 0 ? HIGH : LOW);
      digitalWrite(RIGHT_LED_PIN, speed > 0 ? HIGH : LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
    } else {
      digitalWrite(LEFT_LED_PIN, speed < 0 ? LOW : HIGH);
      digitalWrite(RIGHT_LED_PIN, speed > 0 ? LOW : HIGH);
      digitalWrite(CENTER_LED_PIN, LOW);
    }
  }
}

/**
 * Updates set LED blinking pattern based on button press
 */
void updateSetLedBlink() {
  unsigned long now = millis();
 
  if (setButtonPrevState == LOW && menuState == NORMAL) {
    unsigned long held = now - setButtonDownTime;
   
    if (held < SET_BUTTON_LONG_PRESS_MS) {
      unsigned long timeRemaining = SET_BUTTON_LONG_PRESS_MS - held;
      unsigned long blinkSpeed;
     
      if (timeRemaining > 2000) {
        blinkSpeed = 500;
      } else if (timeRemaining > 1000) {
        blinkSpeed = 250;
      } else {
        blinkSpeed = 100;
      }
     
      if (now - setLedBlinkTimer >= blinkSpeed) {
        blinkState = !blinkState;
        setLedBlinkTimer = now;
      }
      digitalWrite(SET_LED_PIN, blinkState ? HIGH : LOW);
    } else {
      digitalWrite(SET_LED_PIN, HIGH);
    }
  } else if (setLedBlinkError) {
    if (now - setLedBlinkTimer >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      setLedBlinkTimer = now;
    }
    digitalWrite(SET_LED_PIN, blinkState ? HIGH : LOW);
  } else {
    digitalWrite(SET_LED_PIN, menuState != NORMAL ? HIGH : LOW);
  }
}

// ====================================================================
// MENU FUNCTIONS
// ====================================================================

/**
 * Initializes menu values array
 */
void initMenuValues() {
  for (int i = 0; i < numMenuItems; i++) {
    lastMenuValues[i] = 0;
  }
}

/**
 * Checks if menu item is numeric (editable with digits)
 * @param idx Menu item index
 * @return True if numeric item
 */
bool isNumericItem(int idx) {
   switch(idx) {
    case 0: case 1: case 2: case 4: case 5: case 7: case 8: case 9: case 10:
    case 12: case 13: case 14: case 15: case 16: case 17: case 18: return true;
    default: return false;
  }
}

/**
 * Checks if menu item is toggle (ON/OFF or DIRECT/REVERSE)
 * @param idx Menu item index
 * @return True if toggle item
 */
bool isToggleItem(int idx) {
  switch(idx) {
    case 3: case 6: case 20: return true;  // Startup Mode is at index 20
    default: return false;
  }
}

/**
 * Draws main menu on LCD with current selection
 */
void drawMenu() {
  wdt_reset();
  static unsigned long lastMenuUpdate = 0;
  static bool lastBlinkState = true;
  if (millis() - lastMenuUpdate < 50 && blinkState == lastBlinkState) return;
  lastMenuUpdate = millis();
  lastBlinkState = blinkState;
  
  bool forceRedraw = (lastMenuTopItem == -1) ||
                     (lastCurrentMenuItem != currentMenuItem) ||
                     (lastMenuTopItem != menuTopItem) ||
                     (menuState == MENU_EDIT);  // Force redraw when in edit mode
  
  for (int i = 0; i < 2; i++) {
    int idx = menuTopItem + i;
    
    if (idx < 0 || idx >= numMenuItems) {
      // Clear the entire line if no valid menu item
      lcd.setCursor(0, i);
      for (int j = 0; j < 20; j++) {
        lcd.write(' ');
      }
      continue;
    }
    
    long currentValue = getMenuValueInt(idx);
    if (forceRedraw || lastMenuValues[idx] != currentValue || 
        (menuState == MENU_DIGIT_EDIT && idx == currentMenuItem)) {
      
      // Clear the entire line first
      lcd.setCursor(0, i);
      for (int j = 0; j < 20; j++) {
        lcd.write(' ');
      }
      
      // Now draw the menu item
      lcd.setCursor(0, i);
      lcd.print(idx == currentMenuItem ? ">" : " ");
      
      // Print menu item name
      lcd.print(menuItems[idx]);
      
      // Prepare value string
      char valueStr[24];
      memset(valueStr, 0, sizeof(valueStr));
      
      if (menuState == MENU_DIGIT_EDIT && idx == currentMenuItem) {
        strncpy(valueStr, editValueStr, 23);
        if (!blinkState && (isdigit(valueStr[currentDigitPos]) || valueStr[currentDigitPos] == '_')) {
          valueStr[currentDigitPos] = ' ';
        }
      } else {
        getMenuValueString(idx, valueStr);
      }
      
      // Right-align the value
      int valueLen = strlen(valueStr);
      if (valueLen <= 20) {
        // Calculate starting position to right-align
        int startPos = 20 - valueLen;
        
        // Clear any existing text in the value area
        for (int j = startPos; j < 20; j++) {
          lcd.setCursor(j, i);
          lcd.write(' ');
        }
        
        // Print the value
        lcd.setCursor(startPos, i);
        lcd.print(valueStr);
      }
      
      lastMenuValues[idx] = currentValue;
    }
  }
  
  lastMenuTopItem = menuTopItem;
  lastCurrentMenuItem = currentMenuItem;
}

/**
 * Draws calibration confirmation menu
 */
void drawCalibMenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;
  if (lastCalibMenuItem != currentCalibMenuItem || millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    // Clear both lines
    lcd.setCursor(0, 0);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    lcd.setCursor(0, 1);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    
    lcd.setCursor(0, 0);
    
    if (calibState == CALIB_LEFT_CONFIRM) {
      lcd.print(F("Left Limit Reached"));
    } else if (calibState == CALIB_RIGHT_CONFIRM) {
      char buffer[21];
      snprintf(buffer, sizeof(buffer), "Calib Done: %ld", fullTravelSteps);
      buffer[20] = '\0';
      lcd.print(buffer);
    } else if (menuState == MENU_EXIT_CONFIRM) {
      lcd.print(F("Exit & Save?"));
    }
    
    char lineBuffer[21];
    memset(lineBuffer, 0, sizeof(lineBuffer));
    snprintf(lineBuffer, sizeof(lineBuffer), "%c%-9s %c%s",
            (currentCalibMenuItem == 0 ? '>' : ' '),
            calibMenuItems[0],
            (currentCalibMenuItem == 1 ? '>' : ' '),
            calibMenuItems[1]);
    lineBuffer[20] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(lineBuffer);
    lastCalibMenuItem = currentCalibMenuItem;
  }
}

/**
 * Gets menu value as integer for comparison
 * @param idx Menu item index
 * @return Value as integer (scaled appropriately)
 */
long getMenuValueInt(int idx) {
  if (idx < 0 || idx >= numMenuItems) return 0;
 
  switch(idx) {
    case 0: return (long)(Kp * 100);
    case 1: return (long)(Ki * 1000);
    case 2: return (long)(Kd * 1000);
    case 3: return pidDirection == DIRECT ? 0 : 1;
    case 4: return (long)(setPoint * 100);
    case 5: return (long)(deadband * 1000);
    case 6: return isManualMode ? 1 : 0;
    case 7: return stepperMinSpeed;
    case 8: return stepperMaxSpeed;
    case 9: return homingSpeed;
    case 10: return autoModeSpeed;
    case 11: return fullTravelSteps;
    case 12: return stepManualFactor;
    case 13: return stepManualFactorSW;
    case 14: return manualModeSpeed;
    case 15: return stepperAcceleration;
    case 16: return buttonHoldInterval;
    case 17: return (long)(dynamicSpeedFactor * 100);
    case 18: return centerMoveSpeed;
    case 19: return (int)currentMotorLevel;  // Motor Current
    case 20: return (int)startupMode;        // Startup Mode
    case 21: return 0;                       // Home...
    case 22: return 0;                       // EXIT MENU
    default: return 0;
  }
}

/**
 * Adjusts parameter value for toggle items
 * @param delta Direction of adjustment (+1 or -1)
 */
void adjustParameter(int delta) {
  switch(currentMenuItem) {
    case 3:
      pidDirection = (pidDirection == DIRECT ? REVERSE : DIRECT);
      myPID.SetControllerDirection(pidDirection);
      break;
      
    case 6:
      stepper.stop();
      long pos = stepper.currentPosition();
      stepper.setCurrentPosition(pos);
      stepper.moveTo(pos);
     
      isManualMode = !isManualMode;
      digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);
     
      if (isManualMode) {
        myPID.SetMode(MANUAL);
        output = 0.0;
      } else {
        myPID.SetMode(AUTOMATIC);
      }
      break;
      
    case 20:  // Startup Mode
      // Properly cycle through the 3 startup modes
      int newMode = (int)startupMode + delta;
      if (newMode < 0) newMode = 2;  // Wrap from AUTO to LAST
      if (newMode > 2) newMode = 0;  // Wrap from LAST to AUTO
      startupMode = (StartupMode)newMode;
      break;
  }
}

/**
 * Parses edited string value and applies it to the parameter
 * @param idx Menu item index
 */
void parseEditedValue(int idx) {
  wdt_reset();
 
  if (idx < 0 || idx >= numMenuItems) return;
 
  char tempStr[24];
  memset(tempStr, 0, sizeof(tempStr));
  strncpy(tempStr, editValueStr, 23);
  tempStr[23] = '\0';
 
  if (strlen(tempStr) == 0) return;
 
  for (int i = 0; tempStr[i]; i++) {
    if (tempStr[i] == '_') tempStr[i] = '0';
  }
 
  switch(idx) {
    case 0: Kp = constrain(atof(tempStr), KP_MIN, KP_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 1: Ki = constrain(atof(tempStr), KI_MIN, KI_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 2: Kd = constrain(atof(tempStr), KD_MIN, KD_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 4: setPoint = constrain(atof(tempStr), SETPOINT_MIN, SETPOINT_MAX); break;
    case 5: deadband = constrain(atof(tempStr), DEADBAND_MIN, DEADBAND_MAX); break;
    case 7: stepperMinSpeed = constrain((int)atol(tempStr), 0, stepperMaxSpeed); break;
    case 8: stepperMaxSpeed = constrain((int)atol(tempStr), stepperMinSpeed, STEPPER_MAX_SPEED_MAX); break;
    case 9: homingSpeed = constrain((int)atol(tempStr), HOMING_SPEED_MIN, HOMING_SPEED_MAX); break;
    case 10: autoModeSpeed = constrain((int)atol(tempStr), AUTO_MODE_SPEED_MIN, AUTO_MODE_SPEED_MAX); break;
    case 12: stepManualFactor = constrain((int)atol(tempStr), STEP_MANUAL_FACTOR_MIN, STEP_MANUAL_FACTOR_MAX); break;
    case 13: stepManualFactorSW = constrain((int)atol(tempStr), STEP_MANUAL_FACTOR_SW_MIN, STEP_MANUAL_FACTOR_SW_MAX); break;
    case 14: manualModeSpeed = constrain((int)atol(tempStr), MANUAL_MODE_SPEED_MIN, MANUAL_MODE_SPEED_MAX); break;
    case 15: stepperAcceleration = constrain((int)atol(tempStr), STEPPER_ACCELERATION_MIN, STEPPER_ACCELERATION_MAX); break;
    case 16: buttonHoldInterval = constrain((int)atol(tempStr), BUTTON_HOLD_INTERVAL_MIN, BUTTON_HOLD_INTERVAL_MAX); break;
    case 17: dynamicSpeedFactor = constrain(atof(tempStr) / 100.0, DYNAMIC_SPEED_FACTOR_MIN, DYNAMIC_SPEED_FACTOR_MAX); break;
    case 18: centerMoveSpeed = constrain((int)atol(tempStr), CENTER_MOVE_SPEED_MIN, CENTER_MOVE_SPEED_MAX); break;
  }
 
  wdt_reset();
}

/**
 * Gets menu value as formatted string for display
 * @param idx Menu item index
 * @param buffer Output buffer for string
 */
void getMenuValueString(int idx, char* buffer) {
  if (!buffer) return;
  if (idx < 0 || idx >= numMenuItems) {
    strcpy(buffer, "ERROR");
    return;
  }
 
  char temp[24];
  memset(temp, 0, sizeof(temp));
 
  switch(idx) {
    case 0:
      dtostrf(Kp, 7, 2, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 1:
      dtostrf(Ki, 7, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 2:
      dtostrf(Kd, 7, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 3: strcpy(buffer, pidDirection == DIRECT ? "DIR" : "REV"); break;
      
    case 4:
      dtostrf(setPoint, 4, 2, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 5:
      dtostrf(deadband, 4, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 6: strcpy(buffer, isManualMode ? "MAN" : "AUTO"); break;
      
    case 7:
      snprintf(temp, 24, "%5d", stepperMinSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 8:
      snprintf(temp, 24, "%5d", stepperMaxSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 9:
      snprintf(temp, 24, "%4d", homingSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 10:
      snprintf(temp, 24, "%5d", autoModeSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 11: snprintf(buffer, 24, "%ld", fullTravelSteps); break;
      
    case 12:
      snprintf(temp, 24, "%3d", stepManualFactor);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 13:
      snprintf(temp, 24, "%3d", stepManualFactorSW);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 14:
      snprintf(temp, 24, "%4d", manualModeSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 15:
      snprintf(temp, 24, "%4d", stepperAcceleration);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 16:
      snprintf(temp, 24, "%4d", buttonHoldInterval);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 17:
      snprintf(temp, 24, "%3d", (int)(dynamicSpeedFactor * 100));
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 18:
      snprintf(temp, 24, "%4d", centerMoveSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
      
    case 19:  // Motor Current
      switch(currentMotorLevel) {
        case CURRENT_0_PERCENT: strcpy(buffer, "0%"); break;
        case CURRENT_50_80_PERCENT: strcpy(buffer, "50-80%"); break;
        case CURRENT_100_PERCENT: strcpy(buffer, "100%"); break;
        case CURRENT_120_150_PERCENT: strcpy(buffer, "120-150%"); break;
        default: strcpy(buffer, "100%");
      }
      break;
      
    case 20:  // Startup Mode
      switch(startupMode) {
        case STARTUP_AUTO: strcpy(buffer, "AUTO"); break;
        case STARTUP_MANUAL: strcpy(buffer, "MAN"); break;
        case STARTUP_LAST: strcpy(buffer, "LAST"); break;
        default: strcpy(buffer, "AUTO");
      }
      break;
      
    case 21: strcpy(buffer, ""); break;  // Home...
    case 22: strcpy(buffer, ""); break;  // EXIT MENU
    default: strcpy(buffer, ""); break;
  }
  buffer[23] = '\0';
}

// ====================================================================
// OPTIMIZED DISPLAY FUNCTIONS
// ====================================================================


// ====================================================================
// LIMIT SWITCH FUNCTION
// ====================================================================

/**
 * Reads limit switch with debouncing
 * @param pin Limit switch pin
 * @param debounceTime Reference to debounce timer
 * @param lastState Reference to last switch state
 * @return True if limit is active
 */
bool updateLimitSwitch(int pin, unsigned long& debounceTime, bool& lastState) {
  unsigned long now = millis();
  bool rawState = digitalRead(pin);
  
  if (rawState != lastState && (now - debounceTime) >= 50) {
    lastState = rawState;
    debounceTime = now;
    return LIMIT_ACTIVE_HIGH ? (rawState == HIGH) : (rawState == LOW);
  }
  
  return LIMIT_ACTIVE_HIGH ? (lastState == HIGH) : (lastState == LOW);
}

// ====================================================================
// CALIBRATION - ALL DELAYS REMOVED
// ====================================================================

/**
 * Runs calibration sequence to measure full travel distance
 * Non-blocking state machine implementation
 */
void runCalibration() {
  // Disable interrupt control during calibration
  runStepperInInterrupt = false;

  static unsigned long delayTimer = 0;
  static bool delayWaiting = false;
 
  unsigned long currentMillis = millis();
  bool setButtonState = digitalRead(SET_BUTTON_PIN);
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
 
  int delta = 0;
  static long lastEncoderPos_calib = 0;
  long newEncoderPos = myEnc.read();
  
  if (calibState != CALIB_IDLE) {
    long encoderDiff = (newEncoderPos - lastEncoderPos_calib) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos_calib = newEncoderPos;
      delta = encoderDiff > 0 ? 1 : -1;
    }
  }
  
  unsigned long dt = currentMillis - lastUpdateTime;
  lastUpdateTime = currentMillis;
  
  switch(calibState) {
    case CALIB_INIT:
      wdt_disable();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Calibration Mode"));
      lcd.setCursor(0, 1);
      lcd.print(F("Finding Left Limit"));
      calibTimer = currentMillis;
      stepper.setSpeed(-calibSpeed);
      calibState = CALIB_MOVE_LEFT;
      delayWaiting = false;
      break;
      
    case CALIB_MOVE_LEFT:
      wdt_reset();
      if (!leftLimitState) {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        stepper.runSpeed();
      } else {
        stepper.stop();
        stepper.setCurrentPosition(0);
        currentPosition = 0;
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        calibTimer = currentMillis;
        delayTimer = currentMillis;
        delayWaiting = true;
        calibState = CALIB_LEFT_CONFIRM;
      }
      break;
      
    case CALIB_LEFT_CONFIRM:
      wdt_reset();
     
      // Wait 50ms after limit hit
      if (delayWaiting && (currentMillis - delayTimer >= 50)) {
        delayWaiting = false;
      }
     
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
      digitalWrite(SET_LED_PIN, LOW);
     
      if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
        calibState = CALIB_IDLE;
        menuState = NORMAL;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calib Timed Out"));
        delayTimer = currentMillis;
        delayWaiting = true;
        wdt_enable(WDTO_8S);
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
        runStepperInInterrupt = true;
        break;
      }
     
      if (delta != 0) {
        currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
        lastCalibMenuItem = -1;
      }
     
      if (setButtonState == LOW && setButtonPrevState == HIGH) {
        setButtonDownTime = currentMillis;
      } else if (setButtonState == HIGH && setButtonPrevState == LOW) {
        unsigned long held = currentMillis - setButtonDownTime;
        if (held >= AUTO_BUTTON_DEBOUNCE_MS && held < SET_BUTTON_LONG_PRESS_MS) {
          if (currentCalibMenuItem == 0) {
            calibTimer = currentMillis;
            stepper.setSpeed(calibSpeed);
            digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
            digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
            digitalWrite(CENTER_LED_PIN, LOW);
            digitalWrite(SET_LED_PIN, LOW);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("Finding Right Limit"));
            calibState = CALIB_MOVE_RIGHT;
          } else {
            calibState = CALIB_IDLE;
            menuState = NORMAL;
            lcd.clear();
            lcd.print(F("Calib Cancelled"));
            delayTimer = currentMillis;
            delayWaiting = true;
            wdt_enable(WDTO_8S);
            displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
            runStepperInInterrupt = true;
          }
        }
      }
      setButtonPrevState = setButtonState;
      drawCalibMenu();
      break;
      
    case CALIB_MOVE_RIGHT:
      wdt_reset();
      if (!rightLimitState) {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        lcd.setCursor(0, 1);
        char buffer[21];
        snprintf(buffer, sizeof(buffer), "Steps: %ld", stepper.currentPosition());
        buffer[20] = '\0';
        lcd.print(buffer);
        stepper.runSpeed();
      } else {
        stepper.stop();
        fullTravelSteps = stepper.currentPosition();
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        calibTimer = currentMillis;
        delayTimer = currentMillis;
        delayWaiting = true;
        calibState = CALIB_RIGHT_CONFIRM;
      }
      break;
      
    case CALIB_RIGHT_CONFIRM:
      wdt_reset();
     
      // Wait 50ms after limit hit
      if (delayWaiting && (currentMillis - delayTimer >= 50)) {
        delayWaiting = false;
      }
     
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
      digitalWrite(SET_LED_PIN, LOW);
     
      if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
        calibState = CALIB_IDLE;
        menuState = NORMAL;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calib Timed Out"));
        delayTimer = currentMillis;
        delayWaiting = true;
        wdt_enable(WDTO_8S);
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
        runStepperInInterrupt = true;
        break;
      }
     
      if (delta != 0) {
        currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
        lastCalibMenuItem = -1;
      }
     
      if (setButtonState == LOW && setButtonPrevState == HIGH) {
        setButtonDownTime = currentMillis;
      } else if (setButtonState == HIGH && setButtonPrevState == LOW) {
        unsigned long held = currentMillis - setButtonDownTime;
        if (held >= AUTO_BUTTON_DEBOUNCE_MS && held < SET_BUTTON_LONG_PRESS_MS) {
          if (currentCalibMenuItem == 0) {
            saveSettings(); // Save calibration to EEPROM
            
            // CRITICAL FIX: Move away from the right limit switch before centering
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("Releasing limit..."));
            lcd.setCursor(0, 1);
            lcd.print(F("Moving left..."));
            
            // Enable stepper
            stepper.enableOutputs();
            
            // Move left 200 steps to release from limit switch
            stepper.setMaxSpeed(500);
            stepper.setAcceleration(stepperAcceleration);
            stepper.move(-200);
            
            // Run until movement complete
            unsigned long moveStartTime = millis();
            while (stepper.distanceToGo() != 0) {
              stepper.run();
              wdt_reset();
              
              // Safety timeout
              if (millis() - moveStartTime > 5000) {
                break;
              }
            }
            
            // Update current position
            currentPosition = stepper.currentPosition();
            
            // Wait a moment
            delay(200);
            
            // Now start centering
            calibState = CALIB_IDLE;
            menuState = NORMAL;
            
            // Mark homing as done so centering can work
            homingDone = true;
            
            // Start centering
            startSmartCentering();
            
            wdt_enable(WDTO_8S);
            displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
            
            // Note: startSmartCentering handles runStepperInInterrupt
          } else {
            calibState = CALIB_IDLE;
            menuState = NORMAL;
            lcd.clear();
            lcd.print(F("Calib Cancelled"));
            delayTimer = currentMillis;
            delayWaiting = true;
            wdt_enable(WDTO_8S);
            displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
            runStepperInInterrupt = true;
          }
        }
      }
      setButtonPrevState = setButtonState;
      drawCalibMenu();
      break;
      
    case CALIB_IDLE:
    default:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
      digitalWrite(SET_LED_PIN, LOW);
      break;
  }
 
  // Handle timed messages for calib cancel/timeout
  if (delayWaiting && (menuState == NORMAL) && (millis() - delayTimer >= 1000)) {
    delayWaiting = false;

    // ============================================================
    // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
    // ============================================================
    clearAllDisplayBuffers();

    quickUpdateDisplayBuffered();
  }
}

// ====================================================================
// BUTTON HANDLERS - FIXED VERSION
// ====================================================================

/**
 * Handles set button presses with short/long press detection
 * FIXED: Short press toggles display, long press enters menu
 */
void handleSetButton() {
  bool setButtonState = digitalRead(SET_BUTTON_PIN);
  unsigned long now = millis();
 
  if (setButtonState == LOW && setButtonPrevState == HIGH) {
    setButtonDownTime = now;
    setLedBlinkError = true;
    setLedBlinkTimer = now;
    blinkState = true;
  } else if (setButtonState == HIGH && setButtonPrevState == LOW) {
    unsigned long held = now - setButtonDownTime;
    setLedBlinkError = false;
   
    if (held >= SET_BUTTON_LONG_PRESS_MS) {
      // LONG PRESS: Enter/Exit menu
      if (menuState == NORMAL) {
        menuState = MENU_BROWSE;
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
        for (int i = 0; i < numMenuItems; i++) {
          lastMenuValues[i] = 0;
        }
        lcd.clear();
        drawMenu();
      } else {
        // EXITING MENU - Save settings before exiting
        saveSettings(); // SAVE SETTINGS TO EEPROM
        menuState = NORMAL;
        
        // ============================================================
        // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
        // ============================================================
        clearAllDisplayBuffers();
        
        // Update display immediately without waiting for interval
        quickUpdateDisplayBuffered();
        
        displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
      }
    } else if (held >= AUTO_BUTTON_DEBOUNCE_MS && menuState == NORMAL && held < SET_BUTTON_LONG_PRESS_MS) {
      // SHORT PRESS: Toggle between voltage bar graph and normal display
      showVoltageBarGraph = !showVoltageBarGraph;
      
      // Reset tracking to force full update
      lastDisplayedPosition = -1;
      lastFullDisplayRefresh = 0; // Force refresh
      quickUpdateDisplayBuffered();
      
      displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
    } else if (held >= AUTO_BUTTON_DEBOUNCE_MS && menuState != NORMAL) {
      // Handle menu navigation
      if (menuState == MENU_BROWSE) {
        if (currentMenuItem == 11) {
          menuState = MENU_CALIB;
          calibState = CALIB_INIT;
          currentCalibMenuItem = 0;
          lastCalibMenuItem = -1;
          lcd.clear();
        }
        else if (currentMenuItem == 19) {      // Motor Current
          menuState = MENU_MOTOR_CURRENT_SUB;
          currentMotorCurrentSubMenuItem = (int)currentMotorLevel;
          lastMotorCurrentSubMenuItem = -1;
          lcd.clear();
          drawMotorCurrentSubmenu();
        }
        else if (currentMenuItem == 20) {      // Startup Mode
          // This is a toggle item, handle it in MENU_EDIT state
          menuState = MENU_EDIT;
        }
        else if (currentMenuItem == 21) {      // Home...
          menuState = MENU_HOMING_SUB;
          currentHomingSubMenuItem = 0;
          lastHomingSubMenuItem = -1;
          lcd.clear();
          drawHomingSubmenu();
        }
        else if (currentMenuItem == 22) {      // EXIT MENU
          menuState = MENU_EXIT_CONFIRM;
          currentCalibMenuItem = 0;
          lastCalibMenuItem = -1;
          lcd.clear();
        }
        else if (isNumericItem(currentMenuItem)) {
          menuState = MENU_DIGIT_EDIT;
          getMenuValueString(currentMenuItem, editValueStr);
          currentDigitPos = 0;
          blinkTimer = millis();
          blinkState = true;
        }
        else if (isToggleItem(currentMenuItem)) {
          menuState = MENU_EDIT;
        }
      }
      else if (menuState == MENU_EDIT) {
        menuState = MENU_BROWSE;
        saveSettings(); // Save toggle changes to EEPROM
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
      }
      else if (menuState == MENU_DIGIT_EDIT) {
        do {
          currentDigitPos++;
          if (editValueStr[currentDigitPos] == '\0') {
            parseEditedValue(currentMenuItem);
            menuState = MENU_BROWSE;
            saveSettings(); // Save digit edits to EEPROM
            lastMenuTopItem = -1;
            lastCurrentMenuItem = -1;
            break;
          }
        } while (!(isdigit(editValueStr[currentDigitPos]) || editValueStr[currentDigitPos] == '_'));
      }
      else if (menuState == MENU_MOTOR_CURRENT_SUB) {
        if (currentMotorCurrentSubMenuItem == numMotorCurrentSubmenuItems - 1) {
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          drawMenu();
        } else {
          setMotorCurrentLevel((MotorCurrentLevel)currentMotorCurrentSubMenuItem);
          saveSettings(); // Save motor current to EEPROM
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          drawMenu();
        }
      }
      else if (menuState == MENU_HOMING_SUB) {
        if (currentHomingSubMenuItem == 0) {
          homeToLeft = true;
          menuState = NORMAL;
          
          // ============================================================
          // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
          // ============================================================
          clearAllDisplayBuffers();
          
          homingState = HOMING_INIT;
          homingDone = false;
          justFinishedHoming = false;
          
          // Update display immediately
          quickUpdateDisplayBuffered();
          
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        }
        else if (currentHomingSubMenuItem == 1) {
          homeToLeft = false;
          menuState = NORMAL;
          
          // ============================================================
          // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
          // ============================================================
          clearAllDisplayBuffers();
          
          homingState = HOMING_INIT;
          homingDone = false;
          justFinishedHoming = false;
          
          // Update display immediately
          quickUpdateDisplayBuffered();
          
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        }
        else if (currentHomingSubMenuItem == 2) {
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          drawMenu();
        }
      }
      else if (menuState == MENU_EXIT_CONFIRM) {
        if (currentCalibMenuItem == 0) {
          saveSettings(); // SAVE ALL SETTINGS TO EEPROM
          menuState = NORMAL;
          
          // ============================================================
          // CRITICAL: CLEAR ALL DISPLAY BUFFERS BEFORE RETURNING
          // ============================================================
          clearAllDisplayBuffers();
          
          // Update display immediately
          quickUpdateDisplayBuffered();
          
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        } else {
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          drawMenu();
        }
      }
    }
  } else if (setButtonState == LOW && setButtonPrevState == LOW && menuState == NORMAL) {
    unsigned long held = now - setButtonDownTime;
    if (held < SET_BUTTON_LONG_PRESS_MS) {
      setLedBlinkError = true;
    }
  }
 
  setButtonPrevState = setButtonState;
}

/**
 * Draws motor current selection submenu
 */
void drawMotorCurrentSubmenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;
  if (lastMotorCurrentSubMenuItem != currentMotorCurrentSubMenuItem || millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    // Clear both lines
    lcd.setCursor(0, 0);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    lcd.setCursor(0, 1);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    
    lcd.setCursor(0, 0);
    lcd.print(F("Motor Current"));
    
    char lineBuffer[21];
    memset(lineBuffer, 0, sizeof(lineBuffer));
    snprintf(lineBuffer, sizeof(lineBuffer), ">%-17s", motorCurrentSubmenuItems[currentMotorCurrentSubMenuItem]);
    lineBuffer[20] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(lineBuffer);
    lastMotorCurrentSubMenuItem = currentMotorCurrentSubMenuItem;
  }
}

/**
 * Draws homing selection submenu
 */
void drawHomingSubmenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;
  if (lastHomingSubMenuItem != currentHomingSubMenuItem || millis() - lastUpdate > 100) {
    lastUpdate = millis();
    
    // Clear both lines
    lcd.setCursor(0, 0);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    lcd.setCursor(0, 1);
    for (int j = 0; j < 20; j++) lcd.write(' ');
    
    lcd.setCursor(0, 0);
    lcd.print(F("Homing Select"));
    
    char lineBuffer[21];
    memset(lineBuffer, 0, sizeof(lineBuffer));
    snprintf(lineBuffer, sizeof(lineBuffer), ">%-17s", homingSubmenuItems[currentHomingSubMenuItem]);
    lineBuffer[20] = '\0';
    lcd.setCursor(0, 1);
    lcd.print(lineBuffer);
    lastHomingSubMenuItem = currentHomingSubMenuItem;
  }
}

/**
 * Handles auto button to toggle between manual and auto modes
 */
 void handleAutoButton() {
  // Don't allow mode switching when in menu
  if (menuState != NORMAL) return;
  
  bool autoButtonState = digitalRead(AUTO_BUTTON_PIN);
  unsigned long now = millis();
  
  if (autoButtonState == LOW && autoButtonPrevState == HIGH) {
    autoButtonDownTime = now;
  }
  else if (autoButtonState == HIGH && autoButtonPrevState == LOW) {
    unsigned long held = now - autoButtonDownTime;
    if (held >= AUTO_BUTTON_DEBOUNCE_MS) {
      noInterrupts();
      stepper.stop();
      long pos = stepper.currentPosition();
      stepper.setCurrentPosition(pos);
      stepper.moveTo(pos);
      interrupts();

      isManualMode = !isManualMode;
      digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);
      if (isManualMode) {
        myPID.SetMode(MANUAL);
        output = 0.0;
      } else {
        myPID.SetMode(AUTOMATIC);
      }
      saveSettings(); // Save mode change to EEPROM
      
      // Reset display tracking to force immediate update
      lastDisplayedPosition = -1;
      strcpy(lastVoltageStr, "");
      lastDisplayedSteps = -999999;
      
      // Update display immediately
      if (menuState == NORMAL) {
        quickUpdateDisplayBuffered();
        lastFullDisplayRefresh = now;
        lastBargraphUpdate = now;
        displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
      }
    }
  }
  autoButtonPrevState = autoButtonState;
}

/**
 * Handles center button to start centering sequence
 * FIXED: Only works when NOT in menu
 */
void handleCenterButton() {
  // Don't allow centering when in menu
  if (menuState != NORMAL) return;
  
  bool centerButtonState = digitalRead(ENCODER_SW);
  unsigned long now = millis();
 
  if (centerButtonState == LOW && centerButtonPrevState == HIGH) {
    centerButtonDownTime = now;
  } else if (centerButtonState == HIGH && centerButtonPrevState == LOW) {
    unsigned long held = now - centerButtonDownTime;
    if (held >= CENTER_BUTTON_DEBOUNCE_MS && homingDone && !centeringActive) {
      if (fullTravelSteps > 0) {
        startSmartCentering();
        
        // Reset display tracking to force update after centering
        lastDisplayedPosition = -1;
        strcpy(lastVoltageStr, "");
        lastDisplayedSteps = -999999;
        lastFullDisplayRefresh = 0;
      }
    }
  }
  centerButtonPrevState = centerButtonState;
}

/**
 * Handles manual control buttons and encoder
 * FIXED: Only works when NOT in menu
 */
void handleManualButtons() {
  // Don't allow manual control when in menu
  if (menuState != NORMAL) return;
  
  unsigned long now = millis();
  wdt_reset();
  long newEncoderPos = myEnc.read();
  long encoderDiff = (newEncoderPos - lastHandledEncoderPos) / 4;
 
  long targetPos = stepper.currentPosition();
  if (encoderDiff != 0) {
    noInterrupts();
    stepper.setMaxSpeed(stepperMaxSpeed);
    stepper.setAcceleration(stepperAcceleration);
    interrupts();
   
    long movementSteps = encoderDiff * stepManualFactor;
    targetPos = stepper.currentPosition() + movementSteps;
    lastHandledEncoderPos = newEncoderPos;
    
    // Force display update on encoder movement
    lastDisplayedPosition = -1;
    lastDisplayedSteps = -999999;
  }
  else {
    setStepperSpeedsForMode();
   
    bool leftBtn = (digitalRead(LEFT_BUTTON_PIN) == LOW);
    bool rightBtn = (digitalRead(RIGHT_BUTTON_PIN) == LOW);
   
    if (leftBtn && !leftLimitState) {
      targetPos = stepper.currentPosition() - stepManualFactorSW;
      // Force display update on button press
      lastDisplayedPosition = -1;
      lastDisplayedSteps = -999999;
    }
    else if (rightBtn && !rightLimitState) {
      targetPos = stepper.currentPosition() + stepManualFactorSW;
      // Force display update on button press
      lastDisplayedPosition = -1;
      lastDisplayedSteps = -999999;
    }
  }
  
  if (isSafeToMove(targetPos)) {
    noInterrupts();
    stepper.moveTo(targetPos);
    interrupts();
  }
  
  noInterrupts();
  currentPosition = stepper.currentPosition();
  float currentSpeed = stepper.speed();
  interrupts();
  updateDirectionLEDsFromVelocity(currentSpeed);
  
  // Fast display update for manual mode
  if (now - lastBargraphUpdate >= BARGRAPH_UPDATE_INTERVAL) {
    lastBargraphUpdate = now;
    if (menuState == NORMAL && !centeringActive && !messageDisplayActive) {
      quickUpdateDisplayBuffered();
    }
  }
  
  if (now - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
    lastPositionUpdate = now;
  }
}

/**
 * Handles menu system navigation and editing
 * Manages encoder input for menu browsing and value adjustment
 */
void handleMenuSystem() {
  wdt_reset();
  if (calibState != CALIB_IDLE || homingState != HOMING_IDLE || centeringActive) return;
 
  static MenuState lastMenuState = NORMAL;
  static long lastEncoderPos_menu = 0;
  long newEncoderPos = myEnc.read();
  int delta = 0;
 
  if (menuState == MENU_BROWSE || menuState == MENU_EDIT || menuState == MENU_DIGIT_EDIT ||
      menuState == MENU_HOMING_SUB || menuState == MENU_MOTOR_CURRENT_SUB) {
    long encoderDiff = (newEncoderPos - lastEncoderPos_menu) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos_menu = newEncoderPos;
      delta = encoderDiff > 0 ? 1 : -1;

      // Interrupt-safe speed setting for menu adjustments
      noInterrupts();
      stepper.setMaxSpeed(stepperMaxSpeed);
      stepper.setAcceleration(stepperAcceleration);
      interrupts();
     
      if (menuState == MENU_BROWSE) {
        currentMenuItem = (currentMenuItem + delta + numMenuItems) % numMenuItems;
        if (currentMenuItem < menuTopItem) {
          menuTopItem = currentMenuItem;
        } else if (currentMenuItem > menuTopItem + 1) {
          menuTopItem = currentMenuItem - 1;
        }
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
        drawMenu();
      }
      else if (menuState == MENU_EDIT) {
        // Special handling for Startup Mode (index 20)
        if (currentMenuItem == 20) {
          // Directly adjust the startup mode with encoder
          int newMode = (int)startupMode + delta;
          if (newMode < 0) newMode = 2;  // Wrap from AUTO to LAST
          if (newMode > 2) newMode = 0;  // Wrap from LAST to AUTO
          startupMode = (StartupMode)newMode;
          
          // Force immediate display update - ALWAYS redraw when in edit mode
          lastMenuValues[20] = -1;  // Force redraw by setting to different value
          drawMenu();
        } else {
          // Handle other toggle items normally
          adjustParameter(delta);
          // Force redraw for all toggle items in edit mode
          lastMenuValues[currentMenuItem] = -1;
          drawMenu();
        }
      } 
      else if (menuState == MENU_DIGIT_EDIT) {
        if (isdigit(editValueStr[currentDigitPos]) || editValueStr[currentDigitPos] == '_') {
          int digitVal = (editValueStr[currentDigitPos] == '_') ? 0 : (editValueStr[currentDigitPos] - '0');
          digitVal = (digitVal + delta + 10) % 10;
          editValueStr[currentDigitPos] = (digitVal == 0 && isNumericItem(currentMenuItem)) ? '_' : ('0' + digitVal);
        }
        drawMenu();
      }
      else if (menuState == MENU_HOMING_SUB) {
        currentHomingSubMenuItem = (currentHomingSubMenuItem + delta + numHomingSubmenuItems) % numHomingSubmenuItems;
        lastHomingSubMenuItem = -1;
        drawHomingSubmenu();
      }
      else if (menuState == MENU_MOTOR_CURRENT_SUB) {
        currentMotorCurrentSubMenuItem = (currentMotorCurrentSubMenuItem + delta + numMotorCurrentSubmenuItems) % numMotorCurrentSubmenuItems;
        lastMotorCurrentSubMenuItem = -1;
        drawMotorCurrentSubmenu();
      }
    }
  }
  else if (menuState == MENU_EXIT_CONFIRM) {
    long encoderDiff = (newEncoderPos - lastEncoderPos_menu) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos_menu = newEncoderPos;
      delta = encoderDiff > 0 ? 1 : -1;
      currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
      lastCalibMenuItem = -1;
      drawCalibMenu();
    }
  }
  
  if (menuState == MENU_DIGIT_EDIT) {
    unsigned long now = millis();
    if (now - blinkTimer >= BLINK_INTERVAL) {
      blinkState = !blinkState;
      blinkTimer = now;
      drawMenu();
    }
  }
  
  // Handle encoder button - only for centering when NOT in menu
  bool encBtnState = digitalRead(ENCODER_SW);
  unsigned long now_enc = millis();
  
  // Don't handle centering when in menu
  if (menuState != NORMAL) {
    // Just update the state to avoid false triggers
    encoderButtonPrevState = encBtnState;
  } else {
    // Only handle centering when NOT in menu
    if (encBtnState == LOW && encoderButtonPrevState == HIGH) {
      encoderButtonDownTime = now_enc;
    } else if (encBtnState == HIGH && encoderButtonPrevState == LOW) {
      unsigned long held = now_enc - encoderButtonDownTime;
      if (held >= SHORT_PRESS_MS && fullTravelSteps > 0 && !centeringActive && startupSearchDone) {
        startSmartCentering();
      }
    }
    encoderButtonPrevState = encBtnState;
  }
  
  // Force redraw if menu state changed
  if (menuState != lastMenuState) {
    lastMenuState = menuState;
    if (menuState == MENU_EXIT_CONFIRM) {
      drawCalibMenu();
    } else if (menuState == MENU_HOMING_SUB) {
      drawHomingSubmenu();
    } else if (menuState != NORMAL) {
      drawMenu();
    }
  }
}

// ====================================================================
// BAR GRAPH CREATION
// ====================================================================

/**
 * Creates custom bar graph characters in LCD memory
 */
void createBarGraphChars() {
  lcd.createChar(CHAR_FULL_BLOCK, (uint8_t*)bitmapFullBlock);
  lcd.createChar(CHAR_CENTER_LEFT, (uint8_t*)bitmapCenterLeft);
  lcd.createChar(CHAR_CENTER_RIGHT, (uint8_t*)bitmapCenterRight);

  // Initialize partial slots with empty
  uint8_t empty[8] = {0};
  lcd.createChar(CHAR_VOLT_PARTIAL, empty);
  lcd.createChar(CHAR_POS_PARTIAL, empty);
}

/**
 * Updates a dynamic custom character slot only if the pattern changed
 * @param slot LCD custom character slot (1 or 2)
 * @param fillLevel 1-4 pixels
 * @param isLTR True for Left-to-Right fill, False for Right-to-Left
 */
void updateDynamicChar(int slot, int fillLevel, bool isLTR) {
  static int lastVoltLevel = -1;
  static bool lastVoltLTR = false;
  static int lastPosLevel = -1;
  static bool lastPosLTR = false;

  bool changed = false;
  if (slot == CHAR_VOLT_PARTIAL) {
    if (fillLevel != lastVoltLevel || isLTR != lastVoltLTR) {
      lastVoltLevel = fillLevel;
      lastVoltLTR = isLTR;
      changed = true;
    }
  } else if (slot == CHAR_POS_PARTIAL) {
    if (fillLevel != lastPosLevel || isLTR != lastPosLTR) {
      lastPosLevel = fillLevel;
      lastPosLTR = isLTR;
      changed = true;
    }
  }

  if (changed) {
    uint8_t bitmap[8];
    uint8_t pattern = isLTR ? fillPatternsLTR[constrain(fillLevel, 0, 4)] : fillPatternsRTL[constrain(fillLevel, 0, 4)];
    for (int i = 0; i < 8; i++) bitmap[i] = pattern;
    lcd.createChar(slot, bitmap);
  }
}

// ====================================================================
// SETUP FUNCTION - DUAL-LOOP INITIALIZATION
// ====================================================================

/**
 * Initializes hardware, loads settings, and performs startup sequence
 */
void setup() {
  // Configure pins
  pinMode(LEFT_LIMIT_PIN, INPUT);
  pinMode(RIGHT_LIMIT_PIN, INPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
 
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUTO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SET_BUTTON_PIN, INPUT_PULLUP);
 
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  pinMode(CENTER_LED_PIN, OUTPUT);
  pinMode(MODE_LED_PIN, OUTPUT);
  pinMode(SET_LED_PIN, OUTPUT);
  pinMode(C0_PIN, OUTPUT);
  pinMode(C1_PIN, OUTPUT);
  
  // Initialize outputs
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(SET_LED_PIN, LOW);
  digitalWrite(MODE_LED_PIN, HIGH);
  
  // Initialize LCD
  lcd.begin(20, 2);
  createBarGraphChars();
  lcd.setCursor(0, 0);
  lcdPrint_P(STR_WEB_GUIDE);
  lcd.setCursor(0, 1);
  lcdPrint_P(STR_INITIALIZING);

  stepper.setMinPulseWidth(500); // Minimum pulse width in microseconds
 
  // Wait 2 seconds non-blocking
  unsigned long startWait = millis();
  while (millis() - startWait < 2000) {
    wdt_reset();
  }
 
  lcd.clear();
 
  // Load settings from EEPROM
  loadSettings();

  // Apply startup mode
  switch(startupMode) {
    case STARTUP_AUTO:
      isManualMode = false;
      break;
    case STARTUP_MANUAL:
      isManualMode = true;
      break;
    case STARTUP_LAST:
      // isManualMode is already loaded from EEPROM
      break;
  }
 
  // Set mode LED based on startup mode
  digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);

  // Read initial limit switch states
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
 
  // Configure stepper and PID
  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-stepperMaxSpeed, stepperMaxSpeed);
  myPID.SetSampleTime(20);
  
  // Initialize menu system
  initMenuValues();
  lastHandledEncoderPos = myEnc.read();
  lastUpdateTime = millis();
  voltageReadTimer = millis();

  // Initialize voltage readings to avoid "glide" on startup
  float startAdc = analogRead(A0);
  float startVout = (startAdc / 1023.0) * 5.0;
  input = startVout * (R1 + R2) / R2;
  displayInput = input;

  wdt_enable(WDTO_8S);
 
  // Reset position tracking
  currentPosition = 0;
  stepper.setCurrentPosition(0);
 
  // ====================================================================
  // DUAL-LOOP INITIALIZATION
  // ====================================================================
  
  // Initialize LCD buffers
  memset(lcdLine0, ' ', 20);
  memset(lcdLine1, ' ', 20);
  memset(lcdNewLine0, ' ', 20);
  memset(lcdNewLine1, ' ', 20);
  lcdLine0[20] = '\0';
  lcdLine1[20] = '\0';
  lcdNewLine0[20] = '\0';
  lcdNewLine1[20] = '\0';
  
  // Setup hardware timer for stepper control
  Timer1.initialize(STEPPER_TIMER_INTERVAL);  // 50 microseconds
  Timer1.attachInterrupt(stepperTimerISR);
  
  // Enable stepper control in interrupt
  runStepperInInterrupt = true;
  
  // ====================================================================
  // END DUAL-LOOP INITIALIZATION
  // ====================================================================
  
  // Check if calibration exists
  if (fullTravelSteps <= 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("No Calibration"));
    lcd.setCursor(0, 1);
    lcd.print(F("Run calibration"));
   
    // Wait 2 seconds
    startWait = millis();
    while (millis() - startWait < 2000) {
      wdt_reset();
    }
   
    startupHomingComplete = true;  // Skip startup sequence
    startupSearchDone = true;      // Mark startup as done
    homingDone = true;             // Mark homing as done (no homing needed)
    
    // Set mode based on startup setting
    if (isManualMode) {
      myPID.SetMode(MANUAL);
    } else {
      myPID.SetMode(AUTOMATIC);
    }
    
    // Update display using buffered method
    quickUpdateDisplayBuffered();
    
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Smart startup..."));
   
    // Wait 1 second
    startWait = millis();
    while (millis() - startWait < 1000) {
      wdt_reset();
    }
   
    // Start smart startup sequence
    startupHomingComplete = false;
    startupSearchDone = false;
    startupState = STARTUP_IDLE;
    startupSearchStartPos = 0;
  }
}

// ====================================================================
// SIMPLE SMART STARTUP - SEEK TO TRANSITION
// ====================================================================

/**
 * Simple startup sequence that moves to find the sensor transition
 * FIXED: Does not change setpoint, keeps user's setpoint
 */
void runSmartStartup() {
  // CRITICAL: Disable interrupt control while starting up
  runStepperInInterrupt = false;

  static unsigned long startupTimeout = 0;
  static bool firstMove = true;
  static long transitionPos1 = -1;
  static long transitionPos2 = -1;
  static bool foundTransition1 = false;
  static int targetTransitionState = LOW;
  
  // Always update sensor and limit switches
  updateCenterSensor();
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
  
  switch(startupState) {
    case STARTUP_IDLE:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Smart startup..."));
      
      // Reset variables
      firstMove = true;
      transitionPos1 = -1;
      transitionPos2 = -1;
      foundTransition1 = false;
      startupTimeout = millis();
      startupEscapingLimit = false;
      
      // Move to limit check
      startupState = STARTUP_CHECK_LIMITS;
      break;
      
    case STARTUP_CHECK_LIMITS:
      // Check if we're stuck at a limit switch
      if (leftLimitState) {
        // Stuck at LEFT limit - need to move RIGHT
        lcd.setCursor(0, 1);
        lcd.print(F("At LEFT limit   "));
        delay(500);
        
        lcd.setCursor(0, 1);
        lcd.print(F("Moving RIGHT... "));
        
        // Move away from left limit
        stepper.enableOutputs();
        stepper.setMaxSpeed(homingSpeed);
        stepper.setAcceleration(stepperAcceleration);
        stepper.move(500);  // Move 500 steps right
        
        startupEscapingLimit = true;
        startupState = STARTUP_ESCAPE_LIMIT;
        
      } else if (rightLimitState) {
        // Stuck at RIGHT limit - need to move LEFT
        lcd.setCursor(0, 1);
        lcd.print(F("At RIGHT limit  "));
        delay(500);
        
        lcd.setCursor(0, 1);
        lcd.print(F("Moving LEFT...  "));
        
        // Move away from right limit
        stepper.enableOutputs();
        stepper.setMaxSpeed(homingSpeed);
        stepper.setAcceleration(stepperAcceleration);
        stepper.move(-500);  // Move 500 steps left
        
        startupEscapingLimit = true;
        startupState = STARTUP_ESCAPE_LIMIT;
        
      } else {
        // NOT at a limit - proceed with normal startup
        startupState = STARTUP_SEEK_TRANSITION;
        
        // Determine search direction based on sensor state
        if (currentCenterSensorState == LOW) {
          lcd.setCursor(0, 1);
          lcd.print(F("R->L: LOW->HIGH"));
          startupSearchDirection = -1;
          targetTransitionState = HIGH;
        } else {
          lcd.setCursor(0, 1);
          lcd.print(F("L->R: HIGH->LOW"));
          startupSearchDirection = 1;
          targetTransitionState = LOW;
        }
        
        stepper.enableOutputs();
        stepper.setMaxSpeed(STARTUP_CENTERING_SPEED);
        stepper.setAcceleration(stepperAcceleration);
        stepper.move(startupSearchDirection * 5000);
      }
      break;
      
    case STARTUP_ESCAPE_LIMIT:
      stepper.run();
      currentPosition = stepper.currentPosition();
      lastDisplayedSteps = currentPosition;         
      lastDisplayedPosition = currentPosition;      
      
      // Update limits while moving
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
      
      // Check if we've escaped the limit
      if (stepper.distanceToGo() == 0 || (!leftLimitState && !rightLimitState)) {
        stepper.stop();
        
        // Verify we're clear of limits
        if (!leftLimitState && !rightLimitState) {
          lcd.setCursor(0, 1);
          lcd.print(F("Limit cleared!  "));
          delay(500);
          
          // Now determine search direction
          if (currentCenterSensorState == LOW) {
            lcd.setCursor(0, 1);
            lcd.print(F("R->L: LOW->HIGH"));
            startupSearchDirection = -1;
            targetTransitionState = HIGH;
          } else {
            lcd.setCursor(0, 1);
            lcd.print(F("L->R: HIGH->LOW"));
            startupSearchDirection = 1;
            targetTransitionState = LOW;
          }
          
          stepper.setMaxSpeed(STARTUP_CENTERING_SPEED);
          stepper.setAcceleration(stepperAcceleration);
          stepper.move(startupSearchDirection * 5000);
          
          startupState = STARTUP_SEEK_TRANSITION;
        } else {
          // Still at limit after move - fall back to homing
          lcd.setCursor(0, 1);
          lcd.print(F("Stuck at limit! "));
          delay(1000);
          startupState = STARTUP_HOMING_FALLBACK;
        }
      }
      break;
      
    case STARTUP_SEEK_TRANSITION:
      stepper.run();
      currentPosition = stepper.currentPosition();
      
      updateCenterSensor();
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
      
      // Check for transition
      if (currentCenterSensorState == targetTransitionState) {
        transitionPos1 = stepper.currentPosition();
        foundTransition1 = true;
        lcd.setCursor(0, 1);
        lcd.print(F("Transition found!"));
      }
      
      // Stop conditions
      if (foundTransition1 || leftLimitState || rightLimitState || 
          stepper.distanceToGo() == 0 || (millis() - startupTimeout > 10000)) {
        
        stepper.stop();
        
        if (foundTransition1) {
          lcd.setCursor(0, 1);
          lcd.print(F("Back off...     "));
          
          currentPosition = stepper.currentPosition();
          stepper.move(-startupSearchDirection * 30);
          startupState = STARTUP_BACK_OFF;
        } else {
          lcd.setCursor(0, 1);
          if (leftLimitState || rightLimitState) {
            lcd.print(F("Limit - homing  "));
          } else {
            lcd.print(F("No trans - home "));
          }
          delay(500);
          startupState = STARTUP_HOMING_FALLBACK;
        }
      }
      break;
      
    case STARTUP_BACK_OFF:
      stepper.run();
      currentPosition = stepper.currentPosition();
      
      if (stepper.distanceToGo() == 0) {
        lcd.setCursor(0, 1);
        lcd.print(F("Verifying...    "));
        
        currentPosition = stepper.currentPosition();
        stepper.setMaxSpeed(centerFineSpeed);
        stepper.move(startupSearchDirection * 60);
        
        startupState = STARTUP_VERIFY_TRANSITION;
      }
      break;
      
    case STARTUP_VERIFY_TRANSITION:
      stepper.run();
      currentPosition = stepper.currentPosition();
      
      updateCenterSensor();
      
      if (currentCenterSensorState == targetTransitionState) {
        transitionPos2 = stepper.currentPosition();
        
        long centerTransitionPos = 0;
        if (transitionPos2 != -1) {
          centerTransitionPos = (transitionPos1 + transitionPos2) / 2;
        } else {
          centerTransitionPos = transitionPos1;
        }
        
        stepper.moveTo(centerTransitionPos);
        startupState = STARTUP_SET_CENTER;
        
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to center"));
      }
      
      if (stepper.distanceToGo() == 0) {
        stepper.moveTo(transitionPos1);
        startupState = STARTUP_SET_CENTER;
        
        lcd.setCursor(0, 1);
        lcd.print(F("Using 1st detect"));
      }
      break;
      
    case STARTUP_SET_CENTER:
      stepper.run();
      currentPosition = stepper.currentPosition();
      
      if (stepper.distanceToGo() == 0 && !startupShowingMessage) {
        // CRITICAL: Calculate and set center position
        long centerPos = 0;
        if (fullTravelSteps > 0) {
          centerPos = fullTravelSteps / 2;
        } else {
          centerPos = currentPosition;
        }
        
        // ============================================================
        // CRITICAL POSITION TRACKING UPDATE - ALL VARIABLES
        // ============================================================
        stepper.setCurrentPosition(centerPos);
        currentPosition = centerPos;
        lastDisplayedPosition = centerPos;
        lastDisplayedSteps = centerPos;
        // ============================================================
        
        // Enable homing flag
        homingDone = true;
        
        // Re-enable PID if in auto mode
        if (!isManualMode) {
          myPID.SetMode(AUTOMATIC);
        }
        
        // Display success message
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Startup Complete"));
        lcd.setCursor(0, 1);
        char msg[21];
        snprintf(msg, sizeof(msg), "Pos:%ld SP:%.2fV", currentPosition, setPoint);
        lcd.print(msg);
        
        digitalWrite(CENTER_LED_PIN, HIGH);
        
        // Start non-blocking timer
        startupCompletionTimer = millis();
        startupShowingMessage = true;
      }
      
      // Non-blocking delay for message display
      if (startupShowingMessage && (millis() - startupCompletionTimer >= 1000)) {
        // ============================================================
        // CRITICAL: CLEAR DISPLAY COMPLETELY BEFORE TRANSITION
        // ============================================================
        clearAllDisplayBuffers();
        
        // Mark startup complete
        startupState = STARTUP_COMPLETE;
        startupSearchDone = true;
        startupHomingComplete = true;
        startupCenterFound = true;
        startupShowingMessage = false;

        // Re-enable interrupt control
        runStepperInInterrupt = true;
        
        displayLockoutUntil = millis() + 100;
        
        // Immediately update display with fresh data
        quickUpdateDisplayBuffered();
      }
      break;
      
    case STARTUP_HOMING_FALLBACK:
      // Note: Homing will handle runStepperInInterrupt
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Startup Failed"));
      lcd.setCursor(0, 1);
      lcd.print(F("Using Homing..."));
      delay(500);
      
      stepper.stop();
      homingState = HOMING_INIT;
      homingDone = false;
      justFinishedHoming = false;
      homeToLeft = true;
      startupState = STARTUP_COMPLETE;
      startupSearchDone = true;
      break;
      
    case STARTUP_COMPLETE:
      // Nothing to do - startup complete
      break;
  }
}

// ====================================================================
// MAIN LOOP - DUAL-LOOP ARCHITECTURE WITH FAST CENTERING
// ====================================================================

/**
 * Main program loop - DUAL-LOOP ARCHITECTURE
 * Loop 1: Hardware Timer Interrupt - Stepper control (20kHz) 
 * Loop 2: Main Loop - Everything else (100Hz-1kHz)
 */
void loop() {
  wdt_reset();
  unsigned long currentMillis = millis();
  
  // ====================================================================
  // TASK 1: Non-blocking LCD update (highest priority in main loop)
  // ====================================================================
  updateLcdNonBlocking();
  
  // ====================================================================
  // TASK 2: Read sensor voltage (fast)
  // ====================================================================
  updateVoltageReading();
  
  // ====================================================================
  // TASK 3: Handle startup sequence if needed
  // ====================================================================
  if (!startupSearchDone && !startupHomingComplete) {
    runSmartStartup();
    return;  // Don't process other things during startup
  }
  
  // ====================================================================
  // TASK 4: Update limit switches (fast)
  // ====================================================================
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
  
  // ====================================================================
  // TASK 5: Handle state machines (moderate speed)
  // ====================================================================
  static unsigned long stateMachineTimer = 0;
  if (currentMillis - stateMachineTimer >= 10) {  // 100Hz state machine update
    stateMachineTimer = currentMillis;
    
    // Handle homing if active
    if (!homingDone && homingState != HOMING_IDLE) {
      runHomingSequence();
      return;
    }
    
    // Handle calibration if active
    if (calibState != CALIB_IDLE) {
      runCalibration();
      return;
    }
  }
  
  // ====================================================================
  // TASK 6: Handle FAST CENTERING (if active) - RUNS AT FULL SPEED
  // ====================================================================
  if (centeringActive) {
    // CRITICAL: During centering, we run stepper in MAIN LOOP for precise control
    runStepperInInterrupt = false;  // Disable interrupt stepper control
    
    // Run the fast centering state machine (runs stepper.run() internally)
    runSmartCentering();
    
    // If centering just completed, re-enable interrupt stepper control
    if (!centeringActive) {
      runStepperInInterrupt = true;  // Re-enable interrupt stepper control
      
      // Force immediate display update after centering
      lastDisplayedPosition = -1;
      strcpy(lastVoltageStr, "");
      lastDisplayedSteps = -999999;
      lastFullDisplayRefresh = 0;
      lastBargraphUpdate = 0;
    }
  }
  
  // ====================================================================
  // TASK 7: Handle button inputs (moderate speed)
  // ====================================================================
  static unsigned long buttonTimer = 0;
  if (currentMillis - buttonTimer >= 20) {  // 50Hz button polling
    buttonTimer = currentMillis;
    
    handleSetButton();
    handleAutoButton();
    handleCenterButton();
    updateSetLedBlink();
  }
  
  // ====================================================================
  // TASK 8: Handle normal operation (moderate speed) - 50Hz
  // ====================================================================
  static unsigned long controlTimer = 0;
  if (currentMillis - controlTimer >= 20 && menuState == NORMAL && !centeringActive) {
    controlTimer = currentMillis;
    
    if (isManualMode) {
      // Manual mode - handle manual control
      handleManualButtons();
    } else if (sensorFault) {
      // SENSOR FAULT SAFETY: Stop all movement in Auto mode if sensor fails
      noInterrupts();
      stepper.stop();
      interrupts();
      updateDirectionLEDsFromVelocity(0);
    } else {
      // Auto mode - PID control
      setStepperSpeedsForMode();
      
      double error = setPoint - input;
      bool inDeadband = abs(error) < deadband;
      
      if (!inDeadband) {
        myPID.Compute();
      } else {
        output = 0;
      }

      static long targetPosition = stepper.currentPosition();

      if (!inDeadband) {
        const float POSITION_SCALE = 0.8;
        long positionDelta = (long)(output * POSITION_SCALE);
        
        int maxMoveSteps = autoModeSpeed / 50;
        
        if (abs(error) > LARGE_ERROR_THRESHOLD) {
          maxMoveSteps = (autoModeSpeed * FAST_SPEED_MULTIPLIER) / 50;
        } else if (abs(error) > MEDIUM_ERROR_THRESHOLD) {
          maxMoveSteps = (autoModeSpeed * MEDIUM_SPEED_MULTIPLIER) / 50;
        }
        
        maxMoveSteps = constrain(maxMoveSteps, 1, 300);
        positionDelta = constrain(positionDelta, -maxMoveSteps, maxMoveSteps);
        
        targetPosition += positionDelta;
      }

      // Keep target within safe bounds
      if (fullTravelSteps > 0) {
        targetPosition = constrain(targetPosition, 0, fullTravelSteps);
      } else {
        // If no calibration yet, limit movement to reasonable range
        long cur = stepper.currentPosition();
        targetPosition = constrain(targetPosition, cur - 1000, cur + 1000);
      }

      if (isSafeToMove(targetPosition)) {
        noInterrupts();
        stepper.moveTo(targetPosition);
        interrupts();
      }
      
      // Update current position from stepper
      noInterrupts();
      currentPosition = stepper.currentPosition();
      interrupts();
      
      // Update direction LEDs (Interrupt-safe reading)
      noInterrupts();
      float currentSpeed = stepper.speed();
      interrupts();
      updateDirectionLEDsFromVelocity(currentSpeed);
    }
  }
  
  // ====================================================================
  // TASK 9: Handle menu system (lowest priority) - 20Hz
  // ====================================================================
  static unsigned long menuTimer = 0;
  if (currentMillis - menuTimer >= 50 && !centeringActive) {
    menuTimer = currentMillis;
    
    if (menuState != NORMAL && calibState == CALIB_IDLE && !centeringActive) {
      handleMenuSystem();
    }
  }
  
  // ====================================================================
  // TASK 10: Update display (slowest - 5-10Hz) - Buffered/non-blocking
  // ====================================================================
  static unsigned long displayTimer = 0;
  unsigned long displayInterval = isManualMode ? 200 : 100;  // 5Hz manual, 10Hz auto
  
  if (currentMillis - displayTimer >= displayInterval) {
    displayTimer = currentMillis;
    
    // Check if centering message display time has elapsed
    if (messageDisplayActive && currentMillis - messageDisplayStartTime >= 1000) {
      messageDisplayActive = false;
    }
    
    // Force display update after centering completes
    if (justFinishedCentering) {
      if (currentMillis - centeringFinishTime > 1000) {
        justFinishedCentering = false;
        
        // Force display update
        lastDisplayedPosition = -1;
        strcpy(lastVoltageStr, "");
        lastDisplayedSteps = -999999;
        lastFullDisplayRefresh = 0;
      }
    }
    
    // Update display after homing
    if (justFinishedHoming) {
      justFinishedHoming = false;
      
      // Force display update
      lastDisplayedPosition = -1;
      strcpy(lastVoltageStr, "");
      lastDisplayedSteps = -999999;
      lastFullDisplayRefresh = 0;
    }
    
    // Only update display in normal mode, not during centering or messages
    if (menuState == NORMAL && !centeringActive && !messageDisplayActive) {
      if (sensorFault && !isManualMode) {
          // Display critical error message
          setLcdContent("! SENSOR FAULT !", "Check Cable / VCC ");
      } else {
          quickUpdateDisplayBuffered();
      }
    }
  }
  
  // ====================================================================
  // TASK 11: Track main loop performance (diagnostic)
  // ====================================================================
  static unsigned long loopCounter = 0;
  static unsigned long lastLoopTime = 0;
  loopCounter++;
  
  if (currentMillis - lastLoopTime >= 1000) {
    lastLoopTime = currentMillis;
    // Optional: Display loop frequency for debugging
    // Serial.print("Loop frequency: ");
    // Serial.print(loopCounter);
    // Serial.println(" Hz");
    loopCounter = 0;
  }
  
  // ====================================================================
  // END OF MAIN LOOP
  // ====================================================================
}