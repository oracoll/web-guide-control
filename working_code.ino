/*
 * WEB GUIDE CONTROLLER V3.1
 * PID-based web guider with stepper motor, optical sensor, LCD menu, homing, calibration
 * and smart optical centering
 *
 * Hardware:
 *   - LCD 20x2 (parallel)
 *   - Rotary encoder + push button
 *   - AccelStepper controlled bipolar stepper
 *   - Analog voltage sensor (voltage divider)
 *   - 3× limit switches (left, right, center/optical)
 *   - Several buttons & status LEDs
 *   - Motor current control via two logic pins
 *
 * Created: 2024-2025
 * Cleaned & restructured: January 2026
 * Improved: February 2026
 */

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <avr/wdt.h>

// ====================================================================
// STRING CONSTANTS (PROGMEM) - Store strings in flash memory to save RAM
// ====================================================================
const char STR_HOMING[] PROGMEM = "Homing...";
const char STR_CALIBRATION[] PROGMEM = "Calibration";
const char STR_CENTERING[] PROGMEM = "Centering...";
const char STR_ERROR[] PROGMEM = "Error!";
const char STR_TIMEOUT[] PROGMEM = "Timeout!";
const char STR_COMPLETE[] PROGMEM = "Complete";
const char STR_LEFT_LIMIT[] PROGMEM = "Left limit found";
const char STR_RIGHT_LIMIT[] PROGMEM = "Right limit found";
const char STR_AUTO_HOMING[] PROGMEM = "Auto-homing...";
const char STR_NO_CALIBRATION[] PROGMEM = "No calibration";
const char STR_RUN_CALIBRATION[] PROGMEM = "Run calibration";
const char STR_HOMING_ERROR[] PROGMEM = "Homing error!";
const char STR_LEFT_NOT_DETECTED[] PROGMEM = "Left not detected";
const char STR_RIGHT_NOT_DETECTED[] PROGMEM = "Right not detected";
const char STR_CENTER_OFFSET[] PROGMEM = "Center offset";
const char STR_RECALIBRATING[] PROGMEM = "Recalibrating...";
const char STR_CENTER_OK[] PROGMEM = "Center OK";
const char STR_HOME_FIRST[] PROGMEM = "Home first!";
const char STR_CALIBRATE_FIRST[] PROGMEM = "Calibrate first!";
const char STR_CENTER_TIMEOUT[] PROGMEM = "Center Timeout!";
const char STR_LIMIT_HIT[] PROGMEM = "Limit hit!";
const char STR_ADJUSTING[] PROGMEM = "Adjusting..";
const char STR_WEB_GUIDE[] PROGMEM = "WEB GUIDE V3.1";
const char STR_INITIALIZING[] PROGMEM = "Initializing...";
const char STR_MANUAL[] PROGMEM = "Man";
const char STR_AUTO[] PROGMEM = "Auto";
const char STR_NEEDS_CALIBRATION[] PROGMEM = "[NEEDS CALIBRATION!]";
const char STR_LEFT_LIMIT_ROMANIAN[] PROGMEM = "<<< LIMITA STANGA";
const char STR_RIGHT_LIMIT_ROMANIAN[] PROGMEM = "LIMITA DREAPTA >>>";
const char STR_PRESS_SET_MENU[] PROGMEM = "Press SET for menu";
const char STR_CENTER_NOT_FOUND[] PROGMEM = "Center not found";
const char STR_HOMING_TO_LEFT[] PROGMEM = "Homing to left...";
const char STR_EDGE_NOT_FOUND[] PROGMEM = "Edge not found!";


// ====================================================================
// MOTOR CONFIGURATION - Grouped related constants
// ====================================================================
const int STEPPER_MIN_SPEED_DEFAULT = 50;
const int STEPPER_MIN_SPEED_MIN = 0;
int stepperMinSpeed = STEPPER_MIN_SPEED_DEFAULT;

const int STEPPER_MAX_SPEED_DEFAULT = 1000;
const int STEPPER_MAX_SPEED_MAX = 10000;
int stepperMaxSpeed = STEPPER_MAX_SPEED_DEFAULT;

const int HOMING_SPEED_DEFAULT = 500;
const int HOMING_SPEED_MIN = 50;
const int HOMING_SPEED_MAX = 10000;
int homingSpeed = HOMING_SPEED_DEFAULT;

const int AUTO_MODE_SPEED_DEFAULT = 800;
const int AUTO_MODE_SPEED_MIN = 50;
const int AUTO_MODE_SPEED_MAX = 10000;
int autoModeSpeed = AUTO_MODE_SPEED_DEFAULT;

const int MANUAL_MODE_SPEED_DEFAULT = 2000;
const int MANUAL_MODE_SPEED_MIN = 100;
const int MANUAL_MODE_SPEED_MAX = 5000;
int manualModeSpeed = MANUAL_MODE_SPEED_DEFAULT;

const int STEPPER_ACCELERATION_DEFAULT = 1000;
const int STEPPER_ACCELERATION_MIN = 1;
const int STEPPER_ACCELERATION_MAX = 10000;
int stepperAcceleration = STEPPER_ACCELERATION_DEFAULT;

// ====================================================================
// PID CONFIGURATION
// ====================================================================
const double KP_DEFAULT = 20.0;
const double KP_MIN = 0.0;
const double KP_MAX = 1000.0;
double Kp = KP_DEFAULT;

const double KI_DEFAULT = 0.0;
const double KI_MIN = 0.0;
const double KI_MAX = 1.0;
double Ki = KI_DEFAULT;

const double KD_DEFAULT = 0.5;
const double KD_MIN = 0.0;
const double KD_MAX = 1000.0;
double Kd = KD_DEFAULT;

const int PID_DIRECTION_DEFAULT = DIRECT;
int pidDirection = PID_DIRECTION_DEFAULT;

const double SETPOINT_DEFAULT = 2.5;
const double SETPOINT_MIN = 0.0;
const double SETPOINT_MAX = 5.0;
double setPoint = SETPOINT_DEFAULT;

const double DEADBAND_DEFAULT = 0.05;
const double DEADBAND_MIN = 0.0;
const double DEADBAND_MAX = 0.2;
double deadband = DEADBAND_DEFAULT;

// ====================================================================
// VOLTAGE SENSOR CALIBRATION (Voltage Divider)
// ====================================================================
const double R1_DEFAULT = 1000.0;
double R1 = R1_DEFAULT;

const double R2_DEFAULT = 5100.0;
double R2 = R2_DEFAULT;

const double ADC_REF_VOLTAGE = 5.0;
const int ADC_RESOLUTION = 1023;

// ====================================================================
// PID ERROR THRESHOLDS
// ====================================================================
const double LARGE_ERROR_THRESHOLD = 0.5;
const double MEDIUM_ERROR_THRESHOLD = 0.2;
const int FAST_SPEED_MULTIPLIER = 3;
const int MEDIUM_SPEED_MULTIPLIER = 2;

// ====================================================================
// TRAVEL AND MOVEMENT PARAMETERS
// ====================================================================
const long FULL_TRAVEL_STEPS_DEFAULT = 0;
const long FULL_TRAVEL_STEPS_MIN = 0;
long fullTravelSteps = FULL_TRAVEL_STEPS_DEFAULT;

const int STEP_MANUAL_FACTOR_DEFAULT = 50;
const int STEP_MANUAL_FACTOR_MIN = 1;
const int STEP_MANUAL_FACTOR_MAX = 1000;
int stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT;

const int STEP_MANUAL_FACTOR_SW_DEFAULT = 10;
const int STEP_MANUAL_FACTOR_SW_MIN = 1;
const int STEP_MANUAL_FACTOR_SW_MAX = 1000;
int stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT;

const int BUTTON_HOLD_INTERVAL_DEFAULT = 500;
const int BUTTON_HOLD_INTERVAL_MIN = 50;
const int BUTTON_HOLD_INTERVAL_MAX = 2000;
int buttonHoldInterval = BUTTON_HOLD_INTERVAL_DEFAULT;

const float DYNAMIC_SPEED_FACTOR_DEFAULT = 1.0;
const float DYNAMIC_SPEED_FACTOR_MIN = 0.1;
const float DYNAMIC_SPEED_FACTOR_MAX = 1.0;
float dynamicSpeedFactor = DYNAMIC_SPEED_FACTOR_DEFAULT;

const bool IS_MANUAL_MODE_DEFAULT = false;
bool isManualMode = IS_MANUAL_MODE_DEFAULT;

const bool INVERT_LED_LOGIC = false;

unsigned long lastFullDisplayRefresh = 0;
const unsigned long FULL_DISPLAY_REFRESH_INTERVAL = 5000; // 5 sec

// ====================================================================
// CENTERING SPEED VARIABLES
// ====================================================================
const int CENTER_MOVE_SPEED_DEFAULT = 800;    // Fast speed for moving to center
const int CENTER_MOVE_SPEED_MIN = 50;
const int CENTER_MOVE_SPEED_MAX = 5000;
int centerMoveSpeed = CENTER_MOVE_SPEED_DEFAULT;

// Split fine speed into two stages for faster convergence
const int CENTER_COARSE_FINE_SPEED_DEFAULT = 200;    // Faster initial fine tuning
const int CENTER_COARSE_FINE_SPEED_MIN = 50;
const int CENTER_COARSE_FINE_SPEED_MAX = 1000;
int centerCoarseFineSpeed = CENTER_COARSE_FINE_SPEED_DEFAULT;

const int CENTER_FINE_SPEED_DEFAULT = 50;    // Slower final convergence
const int CENTER_FINE_SPEED_MIN = 10;
const int CENTER_FINE_SPEED_MAX = 1000;
int centerFineSpeed = CENTER_FINE_SPEED_DEFAULT;

// Add transition threshold - when to switch from coarse to fine
const long CENTER_TRANSITION_STEPS = 100;  // Switch to fine when within 100 steps of transition

long centeringTransitionPosition = 0;  // Position where sensor transition is detected
bool centeringTransitionFound = false;  // Track if we've found the transition

// ====================================================================
// POSITION HOLD VARIABLES
// ====================================================================
bool holdPositionMode = false;
unsigned long holdPositionStartTime = 0;
const unsigned long HOLD_POSITION_DURATION = 2000; // 2 seconds

// ====================================================================
// LCD CONFIGURATION
// ====================================================================
LiquidCrystal lcd(33, 35, 37, 39, 41, 43);

// ====================================================================
// ENCODER CONFIGURATION
// ====================================================================
Encoder myEnc(2, 3);
#define ENCODER_SW 4

// ====================================================================
// STEPPER MOTOR CONFIGURATION
// ====================================================================
#define STEP_PIN 11
#define DIRECTION_PIN 12
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);

// Motor current control pins
#define C0_PIN 13  // needs to be High for 100% power
#define C1_PIN 10  // needs to be Low for 100% power

// ====================================================================
// LIMIT SWITCHES
// ====================================================================
#define LEFT_LIMIT_PIN 45
#define RIGHT_LIMIT_PIN 47
#define CENTER_LIMIT_PIN 49
#define LIMIT_ACTIVE_HIGH true

// ====================================================================
// BUTTONS
// ====================================================================
#define LEFT_BUTTON_PIN 29
#define RIGHT_BUTTON_PIN 31
#define CENTER_BUTTON_PIN 4
#define AUTO_BUTTON_PIN 27
#define SET_BUTTON_PIN 26

// ====================================================================
// LEDS
// ====================================================================
#define LEFT_LED_PIN 48
#define RIGHT_LED_PIN 50
#define MODE_LED_PIN 46
#define SET_LED_PIN 44
#define CENTER_LED_PIN 52

// ====================================================================
// PID & POSITION TRACKING
// ====================================================================
double input = 0;
double output = 0;
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, pidDirection);

long currentPosition = 0;
unsigned long lastUpdateTime = 0;

// ====================================================================
// HOMING STATE
// ====================================================================
enum HomingState { HOMING_IDLE, HOMING_INIT, HOMING_LEFT, HOMING_RIGHT, HOMING_CENTER, HOMING_COMPLETE, HOMING_ERROR };
HomingState homingState = HOMING_IDLE;
bool homingDone = false;
bool homeToLeft = true;
unsigned long homingStartTime = 0;
const unsigned long HOMING_TIMEOUT_MS = 60000;

// ====================================================================
// CALIBRATION STATE
// ====================================================================
enum CalibState { CALIB_IDLE, CALIB_INIT, CALIB_MOVE_LEFT, CALIB_LEFT_CONFIRM, CALIB_MOVE_RIGHT, CALIB_RIGHT_CONFIRM };
CalibState calibState = CALIB_IDLE;
unsigned long calibTimer = 0;
const int calibSpeed = 200;
const unsigned long CALIB_TIMEOUT_MS = 60000;

int currentCalibMenuItem = 0;
const char* calibMenuItems[] = {"Continue", "Cancel"};
const int numCalibMenuItems = sizeof(calibMenuItems) / sizeof(calibMenuItems[0]);

// ====================================================================
// CENTERING WITH SENSOR STATE
// ====================================================================
enum CenteringState {
  CENTERING_IDLE,
  CENTERING_MOVE_TO_CENTER,
  CENTERING_FIND_TRANSITION,
  CENTERING_COMPLETE
};

CenteringState centeringState = CENTERING_IDLE;
int lastCenterSensorState = HIGH;
int currentCenterSensorState = HIGH;
unsigned long centerSensorDebounceTime = 0;
const unsigned long CENTER_SENSOR_DEBOUNCE_MS = 10;

bool centeringActive = false;

// ====================================================================
// MENU SYSTEM
// ====================================================================
enum MenuState { NORMAL, MENU_BROWSE, MENU_EDIT, MENU_CALIB, MENU_DIGIT_EDIT, MENU_HOMING_SUB, MENU_MOTOR_CURRENT_SUB, MENU_EXIT_CONFIRM };
MenuState menuState = NORMAL;
int currentMenuItem = 0;
int menuTopItem = 0;
const int LCD_MENU_ROWS = 4;

const char* menuItems[] = {
  "PID Kp", "PID Ki", "PID Kd", "PID Direction", "Sensor SetPoint",
  "Deadband", "Operation Mode", "Min Speed", "Max Speed",
  "Homing Speed", "Auto Speed", "Calib Steps", "Manual Factor",
  "Manual Fac SW", "Manual Speed", "Acceleration", "Manual Btn Hold tm", "Dyn Speed %","Center Speed",
  "Mot. Current", "Home...", "EXIT MENU"
};

const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);

long lastMenuValues[numMenuItems];
int lastMenuTopItem = -1;
int lastCurrentMenuItem = -1;
int lastCalibMenuItem = -1;

enum MotorCurrentLevel { CURRENT_0_PERCENT, CURRENT_50_80_PERCENT, CURRENT_100_PERCENT, CURRENT_120_150_PERCENT };
const char* motorCurrentSubmenuItems[] = {"0%", "50-80%", "100%", "120-150%", "Back"};
const int numMotorCurrentSubmenuItems = sizeof(motorCurrentSubmenuItems) / sizeof(motorCurrentSubmenuItems[0]);
int currentMotorCurrentSubMenuItem = 0;
int lastMotorCurrentSubMenuItem = -1;
MotorCurrentLevel currentMotorLevel = CURRENT_100_PERCENT;

const char* homingSubmenuItems[] = {"Home Left", "Home Right", "Back"};
const int numHomingSubmenuItems = sizeof(homingSubmenuItems) / sizeof(homingSubmenuItems[0]);
int currentHomingSubMenuItem = 0;
int lastHomingSubMenuItem = -1;

char editValueStr[24];
int currentDigitPos = 0;
unsigned long blinkTimer = 0;
bool blinkState = true;
const unsigned long BLINK_INTERVAL = 500;

// ====================================================================
// BUTTON STATE VARIABLES
// ====================================================================
unsigned long setButtonDownTime = 0;
bool setButtonPrevState = HIGH;
const unsigned long SET_BUTTON_LONG_PRESS_MS = 3000;

unsigned long autoButtonDownTime = 0;
bool autoButtonPrevState = HIGH;
const unsigned long AUTO_BUTTON_DEBOUNCE_MS = 100;

unsigned long centerButtonDownTime = 0;
bool centerButtonPrevState = HIGH;
const unsigned long CENTER_BUTTON_DEBOUNCE_MS = 100;

unsigned long centerLimitDebounceTime = 0;
bool centerLimitPrevState = HIGH;
const unsigned long LIMIT_DEBOUNCE_MS = 50;

bool setLedBlinkError = false;
unsigned long setLedBlinkTimer = 0;
long lastHandledEncoderPos = 0;

// ====================================================================
// MANUAL BUTTON HOLD TRACKING
// ====================================================================
unsigned long leftButtonHoldTimer = 0;
unsigned long rightButtonHoldTimer = 0;
bool leftButtonHeld = false;
bool rightButtonHeld = false;

// ====================================================================
// OPERATIONAL VARIABLES
// ====================================================================
unsigned long previousMillis = 0;
const long interval = 20;
unsigned long voltageReadTimer = 0;
const unsigned long VOLTAGE_READ_INTERVAL_AUTO = 20;
const unsigned long VOLTAGE_READ_INTERVAL_MANUAL = 500;
long lastDisplayedPosition = -1;
unsigned long lastPositionUpdate = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 300;
unsigned long encoderButtonDownTime = 0;
bool encoderButtonPrevState = HIGH;
const unsigned long LONG_PRESS_MS = 1000;
const unsigned long SHORT_PRESS_MS = 50;
long lastEncoderPos = 0;
unsigned long leftLimitDebounceTime = 0;
unsigned long rightLimitDebounceTime = 0;
bool leftLimitState = LOW;
bool rightLimitState = LOW;
bool centerLimitState = LOW;

unsigned long displayLockoutUntil = 0;
const unsigned long DISPLAY_LOCKOUT_MS = 500;
bool justFinishedHoming = false;

// Manual operation timeout
const unsigned long MANUAL_OPERATION_TIMEOUT = 30000; // 30 seconds
unsigned long lastManualActivity = 0;

// System error state tracking
bool systemErrorState = false;
unsigned long systemErrorDisplayTime = 0;
const unsigned long SYSTEM_ERROR_DISPLAY_DURATION = 3000; // 3 seconds

// ====================================================================
// EEPROM CONFIGURATION
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
  uint16_t checksum;
};

const int EEPROM_ADDRESS = 0;

// ====================================================================
// CUSTOM LCD BAR GRAPH CHARACTERS
// ====================================================================
uint8_t barChars[8][8] = {
  {0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000},
  {0b00001,0b00001,0b00001,0b00001,0b00001,0b00001,0b00001,0b00001},
  {0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011},
  {0b00111,0b00111,0b00111,0b00111,0b00111,0b00111,0b00111,0b00111},
  {0b01111,0b01111,0b01111,0b01111,0b01111,0b01111,0b01111,0b01111},
  {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111},
  {0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011},
  {0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000}
};

uint8_t barCharsMirrored[8][8] = {
  {0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000,0b00000},
  {0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b10000},
  {0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000},
  {0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100,0b11100},
  {0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110,0b11110},
  {0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111,0b11111},
  {0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011,0b00011},
  {0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000,0b11000}
};

/**
 * Creates custom bar graph characters for LCD display
 */
void createBarGraphChars() {
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, barChars[i]);
  }
}

// ====================================================================
// FUNCTION PROTOTYPES
// ====================================================================
void handleMenuSystem();
void runCalibration();
void handleSetButton();
void handleAutoButton();
void handleCenterButton();
void handleManualButtons();
float readVoltage();
void displayPositionGraph(bool isLeft = false);
void updateMainDisplay();
void saveSettings();
void loadSettings();
void initMenuValues();
void drawMenu();
void drawCalibMenu();
long getMenuValueInt(int idx);
void adjustParameter(int delta);
void getMenuValueString(int idx, char* buffer);
bool isNumericItem(int idx);
bool isToggleItem(int idx);
void parseEditedValue(int idx);
void updateDirectionLEDsFromVelocity(float speed);
bool readStableButtonState(int pin);
void runHomingSequence();
void updateSetLedBlink();
bool updateCenterSensor();
void startSmartCentering();
void runSmartCentering();
void drawMotorCurrentSubmenu();
void drawHomingSubmenu();
bool isValidFloat(float val, float minVal, float maxVal);
bool isValidInt(long val, long minVal, long maxVal);
bool isSafeToMove(long targetPos);
void verifyCenterPosition();
void verifyPositionAfterHoming();
bool updateLimitSwitch(int pin, unsigned long& debounceTime, bool& lastState);
void updateVoltageReading();
void updateCurrentPosition(long newPosition);
bool checkSystemStatus();
bool validateConfiguration();
void resetEncoderPosition();
void showSystemError();
void clearSystemError();

// ====================================================================
// HELPER FUNCTION FOR PROGMEM STRINGS
// ====================================================================
void lcdPrint_P(const char* str) {
  char buf[21]; // Buffer for 20 characters + null terminator
  strcpy_P(buf, str);
  lcd.print(buf);
}

// ====================================================================
// SYSTEM ERROR DISPLAY FUNCTIONS
// ====================================================================
void showSystemError() {
  systemErrorState = true;
  systemErrorDisplayTime = millis();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("System Error!"));
  lcd.setCursor(0, 1);
  lcdPrint_P(STR_CALIBRATE_FIRST);

  // Show hint for menu access
  delay(1500);
  lcd.setCursor(0, 1);
  lcdPrint_P(STR_PRESS_SET_MENU);
}

void clearSystemError() {
  systemErrorState = false;
  updateMainDisplay();
}

// ====================================================================
// CENTRALIZED POSITION UPDATE FUNCTION
// ====================================================================
/**
 * Updates the current position and ensures all dependent systems are notified
 * @param newPosition The new position in steps
 */
void updateCurrentPosition(long newPosition) {
  // Only update if the position has actually changed
  if (currentPosition != newPosition) {
    currentPosition = newPosition;
    stepper.setCurrentPosition(newPosition);

    // Update position-dependent states
    if (fullTravelSteps > 0) {
      bool atCenter = abs(currentPosition - fullTravelSteps/2) < 5;
      digitalWrite(CENTER_LED_PIN, atCenter ? HIGH : LOW);
    }
  }
}

// ====================================================================
// SYSTEM STATUS CHECK FUNCTION
// ====================================================================
bool checkSystemStatus() {
  // Check if critical systems are operational
  if (fullTravelSteps <= 0) {
    // Don't show error if we're already in a menu or calibration state
    if (menuState == NORMAL && calibState == CALIB_IDLE && !systemErrorState) {
      showSystemError();
    }
    return false;
  }

  // Skip position check during homing/centering operations
  if (!homingDone || homingState != HOMING_IDLE || centeringActive) {
    return true;
  }

  // Check if we're at a limit but not tracking it
  if ((leftLimitState && currentPosition > 10) ||
      (rightLimitState && currentPosition < fullTravelSteps - 10)) {
    if (!systemErrorState) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Position Error!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Rehome needed"));
      delay(2000);
      homingState = HOMING_INIT;
      homingDone = false;
    }
    return false;
  }

  return true;
}

// ====================================================================
// CONFIGURATION VALIDATION FUNCTION
// ====================================================================
bool validateConfiguration() {
  // Check for logical inconsistencies in settings
  if (stepperMinSpeed > stepperMaxSpeed) {
    // Swap values if they're reversed
    int temp = stepperMinSpeed;
    stepperMinSpeed = stepperMaxSpeed;
    stepperMaxSpeed = temp;
  }

  if (stepManualFactor <= 0) {
    stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT;
  }

  if (stepManualFactorSW <= 0) {
    stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT;
  }

  // Ensure PID values are reasonable
  if (Kp <= 0) Kp = KP_DEFAULT;
  if (Ki < 0) Ki = KI_DEFAULT;
  if (Kd < 0) Kd = KD_DEFAULT;

  // Update PID controller with validated values
  myPID.SetTunings(Kp, Ki, Kd);

  return true;
}

// ====================================================================
// HELPER FUNCTIONS FOR RELIABILITY
// ====================================================================
bool isValidFloat(float val, float minVal, float maxVal) {
  if (val != val) return false; // Check for NaN
  if (val == INFINITY || val == -INFINITY) return false;
  return (val >= minVal && val <= maxVal);
}

bool isValidInt(long val, long minVal, long maxVal) {
  return (val >= minVal && val <= maxVal);
}

bool isSafeToMove(long targetPos) {
  if (targetPos < 0) return false;
  if (fullTravelSteps > 0 && targetPos > fullTravelSteps) return false;
  if (targetPos <= 0 && leftLimitState) return false;
  if (fullTravelSteps > 0 && targetPos >= fullTravelSteps && rightLimitState) return false;
  return true;
}

// ====================================================================
// EEPROM FUNCTIONS
// ====================================================================
uint16_t calculateChecksum(Settings& settings) {
  uint16_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&settings;
  for (size_t i = 0; i < sizeof(Settings) - sizeof(uint16_t); i++) {
    checksum += ptr[i];
  }
  return checksum;
}

void saveSettings() {
  Settings settings = {
    (float)Kp, (float)Ki, (float)Kd, pidDirection, (float)setPoint,
    (float)deadband, isManualMode,  // ← This line saves the mode to EEPROM
    stepperMinSpeed, stepperMaxSpeed,
    homingSpeed, autoModeSpeed, fullTravelSteps, stepManualFactor,
    stepManualFactorSW, stepperAcceleration, buttonHoldInterval,
    dynamicSpeedFactor, currentMotorLevel, centerMoveSpeed, centerFineSpeed, centerCoarseFineSpeed, 0
  };
  settings.checksum = calculateChecksum(settings);
  wdt_reset();
  EEPROM.put(EEPROM_ADDRESS, settings);
  wdt_reset();
}


void loadSettings() {
  Settings settings;
  EEPROM.get(EEPROM_ADDRESS, settings);
  uint16_t storedChecksum = settings.checksum;
  settings.checksum = 0;
  uint16_t calculatedChecksum = calculateChecksum(settings);

  if (storedChecksum != calculatedChecksum) {
    // Use default values if checksum doesn't match
    wdt_reset();
    Kp = KP_DEFAULT;
    Ki = KI_DEFAULT;
    Kd = KD_DEFAULT;
    pidDirection = PID_DIRECTION_DEFAULT;
    setPoint = SETPOINT_DEFAULT;
    deadband = DEADBAND_DEFAULT;
    isManualMode = IS_MANUAL_MODE_DEFAULT;
    stepperMinSpeed = STEPPER_MIN_SPEED_DEFAULT;
    stepperMaxSpeed = STEPPER_MAX_SPEED_DEFAULT;
    homingSpeed = HOMING_SPEED_DEFAULT;
    autoModeSpeed = AUTO_MODE_SPEED_DEFAULT;
    fullTravelSteps = FULL_TRAVEL_STEPS_DEFAULT;
    stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT;
    stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT;
    stepperAcceleration = STEPPER_ACCELERATION_DEFAULT;
    buttonHoldInterval = BUTTON_HOLD_INTERVAL_DEFAULT;
    dynamicSpeedFactor = DYNAMIC_SPEED_FACTOR_DEFAULT;
    currentMotorLevel = CURRENT_100_PERCENT;
    centerMoveSpeed = CENTER_MOVE_SPEED_DEFAULT;
    centerFineSpeed = CENTER_FINE_SPEED_DEFAULT;
    centerCoarseFineSpeed = CENTER_COARSE_FINE_SPEED_DEFAULT;

  } else {
    // Use loaded values if they're valid
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
      // Swap if min > max
      int temp = loadedMinSpeed;
      loadedMinSpeed = loadedMaxSpeed;
      loadedMaxSpeed = temp;
    }

    stepperMinSpeed = isValidInt(loadedMinSpeed, STEPPER_MIN_SPEED_MIN, STEPPER_MAX_SPEED_MAX) ? loadedMinSpeed : STEPPER_MIN_SPEED_DEFAULT;
    stepperMaxSpeed = isValidInt(loadedMaxSpeed, STEPPER_MIN_SPEED_MIN, STEPPER_MAX_SPEED_MAX) ? loadedMaxSpeed : STEPPER_MAX_SPEED_DEFAULT;
    homingSpeed = isValidInt(settings.homingSpeed, HOMING_SPEED_MIN, HOMING_SPEED_MAX) ? settings.homingSpeed : HOMING_SPEED_DEFAULT;
    autoModeSpeed = isValidInt(settings.autoModeSpeed, AUTO_MODE_SPEED_MIN, AUTO_MODE_SPEED_MAX) ? settings.autoModeSpeed : AUTO_MODE_SPEED_DEFAULT;

    wdt_reset();

    fullTravelSteps = settings.fullTravelSteps >= FULL_TRAVEL_STEPS_MIN ? settings.fullTravelSteps : FULL_TRAVEL_STEPS_DEFAULT;
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

// ====================================================================
// HELPER FUNCTION
// ====================================================================
void setStepperSpeedsForMode() {
  if (isManualMode) {
    stepper.setMaxSpeed(manualModeSpeed);
  } else {
    stepper.setMaxSpeed(autoModeSpeed);
  }
  stepper.setAcceleration(stepperAcceleration);
}

// ====================================================================
// MARLIN-LIKE HOMING SEQUENCE
// ====================================================================
void runHomingSequence() {
  wdt_reset();
  unsigned long now = millis();
  bool timeout = (now - homingStartTime > HOMING_TIMEOUT_MS);

  // Add emergency stop if we hit a limit unexpectedly
  if ((homeToLeft && rightLimitState) || (!homeToLeft && leftLimitState)) {
    stepper.stop();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Homing Error!"));
    lcd.setCursor(0, 1);
    lcd.print(F("Wrong limit hit"));
    delay(2000);
    homingState = HOMING_INIT;
    homingStartTime = now;
    return;
  }

  switch(homingState) {
    case HOMING_INIT:
      lcd.clear();
      delay(100);
      lcd.setCursor(0, 0);
      lcdPrint_P(STR_HOMING);

      homingStartTime = now;

      // Update limit states before starting
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

      if (homeToLeft) {
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to left limit"));

        // Safety check: if we're somehow at the right limit, we're stuck
        if (rightLimitState) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Stuck at right!"));
          lcd.setCursor(0, 1);
          lcd.print(F("Reversing..."));
          delay(1000);
          homeToLeft = false;  // Reverse direction
          stepper.setSpeed(-homingSpeed);
          homingState = HOMING_RIGHT;
        } else {
          stepper.setSpeed(homingSpeed);
          homingState = HOMING_LEFT;
        }
      } else {
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to right limit"));

        // Safety check: if we're somehow at the left limit, we're stuck
        if (leftLimitState) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Stuck at left!"));
          lcd.setCursor(0, 1);
          lcd.print(F("Reversing..."));
          delay(1000);
          homeToLeft = true;  // Reverse direction
          stepper.setSpeed(homingSpeed);
          homingState = HOMING_LEFT;
        } else {
          stepper.setSpeed(-homingSpeed);
          homingState = HOMING_RIGHT;
        }
      }
      break;

    case HOMING_LEFT:
      if (timeout) {
        stepper.stop();
        homingState = HOMING_ERROR;
        break;
      }
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      if (leftLimitState) {
        stepper.stop();
        delay(50);
        stepper.setCurrentPosition(0);
        currentPosition = 0;  // EXPLICIT: Track that we're at position 0
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_LEFT_LIMIT);
        lcd.print(F("    ")); // Add spaces to clear rest of line
        delay(200);

        // Update limit states after stopping
        leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
        rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

        if (fullTravelSteps > 0) {
          long centerPos = fullTravelSteps / 2;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcdPrint_P(STR_CENTERING);
          stepper.setMaxSpeed(homingSpeed);
          stepper.setAcceleration(stepperAcceleration);
          stepper.moveTo(centerPos);
          homingState = HOMING_CENTER;
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        stepper.runSpeed();
      }
      break;

    case HOMING_RIGHT:
      if (timeout) {
        stepper.stop();
        homingState = HOMING_ERROR;
        break;
      }
      rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
      if (rightLimitState) {
        stepper.stop();
        delay(50);
        stepper.setCurrentPosition(fullTravelSteps);
        currentPosition = fullTravelSteps;  // EXPLICIT: Track that we're at max position
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_RIGHT_LIMIT);
        lcd.print(F("   ")); // Add spaces to clear rest of line
        delay(200);

        // Update limit states after stopping
        leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
        rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

        if (fullTravelSteps > 0) {
          long centerPos = fullTravelSteps / 2;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcdPrint_P(STR_CENTERING);
          stepper.setMaxSpeed(homingSpeed);
          stepper.setAcceleration(stepperAcceleration);
          stepper.moveTo(centerPos);
          homingState = HOMING_CENTER;
        } else {
          homingState = HOMING_COMPLETE;
        }
      } else {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(CENTER_LED_PIN, LOW);
        stepper.runSpeed();
      }
      break;

    case HOMING_CENTER:
      if (timeout) {
        stepper.stop();
        homingState = HOMING_ERROR;
        break;
      }
      stepper.run();
  if (stepper.distanceToGo() == 0) {
    currentPosition = stepper.currentPosition();

    // === STOP COMPLETELY ===
    stepper.stop();
    stepper.setSpeed(0);
    stepper.moveTo(currentPosition);  // Prevent further movement

    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
    digitalWrite(CENTER_LED_PIN, HIGH);
    lcd.setCursor(0, 1);
    char buffer[21];
    snprintf(buffer, sizeof(buffer), "Home: %ld", currentPosition);
    buffer[20] = '\0';
    lcd.print(buffer);
    delay(500);
    homingState = HOMING_COMPLETE;
  }
  break;

    case HOMING_COMPLETE:
      homingDone = true;
      lcd.clear();
      delay(100);
      lcd.setCursor(0, 0);
      lcd.print(F("Homing Complete"));
      delay(500);
      homingState = HOMING_IDLE;
      justFinishedHoming = true;

      // NEW: Verify position after homing
      verifyPositionAfterHoming();
      break;

    case HOMING_ERROR:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcdPrint_P(STR_HOMING_ERROR);
      lcd.setCursor(0, 1);
      lcdPrint_P(STR_TIMEOUT);

      // Stay in this error state until the system is reset or user intervenes
      homingState = HOMING_IDLE; // Prevent re-triggering error
      homingDone = false;       // Critical: do not allow operation
      showSystemError();
      break;

    default:
      break;
  }
}

// ====================================================================
// POSITION VERIFICATION AFTER HOMING
// ====================================================================
void verifyPositionAfterHoming() {
  // Check if we're actually at the expected position after homing
  if (homeToLeft && !leftLimitState && currentPosition == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_HOMING_ERROR);
    lcd.print(F("!"));
    lcd.setCursor(0, 1);
    lcdPrint_P(STR_LEFT_NOT_DETECTED);
    delay(2000);
  } else if (!homeToLeft && !rightLimitState && currentPosition == fullTravelSteps) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_HOMING_ERROR);
    lcd.print(F("!"));
    lcd.setCursor(0, 1);
    lcdPrint_P(STR_RIGHT_NOT_DETECTED);
    delay(2000);
  } else if (abs(currentPosition - fullTravelSteps/2) > 50) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_CENTER_OFFSET);
    lcd.setCursor(0, 1);
    lcdPrint_P(STR_RECALIBRATING);
    delay(1000);

    // Try to move to center again
    stepper.setMaxSpeed(homingSpeed);
    stepper.setAcceleration(stepperAcceleration);
    stepper.moveTo(fullTravelSteps/2);
    homingState = HOMING_CENTER;
  }
}

// ====================================================================
// CENTERING WITH OPTICAL SENSOR FUNCTIONS
// ====================================================================
bool updateCenterSensor() {
  unsigned long now = millis();
  int rawState = digitalRead(CENTER_LIMIT_PIN);

  if (rawState != lastCenterSensorState && (now - centerSensorDebounceTime) >= CENTER_SENSOR_DEBOUNCE_MS) {
    lastCenterSensorState = rawState;
    centerSensorDebounceTime = now;
    currentCenterSensorState = rawState;
    return true;
  }
  return false;
}

void startSmartCentering() {


  if (homingDone && !centeringActive && fullTravelSteps > 0) {
    centeringActive = true;
    // === DISABLE PID DURING CENTERING ===
    if (!isManualMode) {
      myPID.SetMode(MANUAL);
      output = 0.0;
    }
    centeringState = CENTERING_MOVE_TO_CENTER;
    centeringTransitionFound = false;  // Reset transition tracking

    long targetCenter = fullTravelSteps / 2;
    long currentPos = stepper.currentPosition();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_CENTERING);

    stepper.setMaxSpeed(centerMoveSpeed);
    stepper.setAcceleration(stepperAcceleration);

    if (leftLimitState) {
      stepper.moveTo(targetCenter);
      lcd.setCursor(0, 1);
      lcd.print(F("From left limit ->"));
    }
    else if (rightLimitState) {
      stepper.moveTo(targetCenter);
      lcd.setCursor(0, 1);
      lcd.print(F("<- From right limit"));
    }
    else if (currentPos < targetCenter) {
      stepper.moveTo(targetCenter);
      lcd.setCursor(0, 1);
      lcd.print(F("Moving Right ->"));
    }
    else if (currentPos > targetCenter) {
      stepper.moveTo(targetCenter);
      lcd.setCursor(0, 1);
      lcd.print(F("<- Moving Left"));
    }
    else if (currentPos == targetCenter) {
      // Already at center, start fine adjustment immediately
      lcd.setCursor(0, 1);
      lcd.print(F("At center, fine adj"));
      centeringState = CENTERING_FIND_TRANSITION;
      centeringTransitionFound = false;

      currentCenterSensorState = digitalRead(CENTER_LIMIT_PIN);
      lastCenterSensorState = currentCenterSensorState;

      // Move slightly off center to ensure we detect the transition
      stepper.moveTo(targetCenter + 50);
      stepper.setSpeed(centerCoarseFineSpeed);
    }
  }
  else if (!homingDone) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_HOME_FIRST);
    delay(1000);
    updateMainDisplay();
  }
  else if (fullTravelSteps <= 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_CALIBRATE_FIRST);
    delay(1000);
    updateMainDisplay();
  }
}

void runSmartCentering() {
  if (!centeringActive) return;

  // Debounce & read limits
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
  bool sensorChanged = updateCenterSensor();

  static unsigned long centeringStartTime = 0;
  const unsigned long CENTERING_TIMEOUT_MS = 15000; // 15 sec safety

  if (centeringState == CENTERING_MOVE_TO_CENTER) {
    centeringStartTime = millis(); // Start timeout clock
  }

  // Timeout protection with recovery
  if (millis() - centeringStartTime > CENTERING_TIMEOUT_MS) {
    stepper.stop();
    centeringActive = false;
    centeringState = CENTERING_IDLE;

    // START FALLBACK HOMING
    lcd.clear();
    lcd.setCursor(0, 0);
    lcdPrint_P(STR_CENTER_NOT_FOUND);
    lcd.setCursor(0, 1);
    lcdPrint_P(STR_HOMING_TO_LEFT);
    delay(1500);
    homeToLeft = true;
    homingState = HOMING_INIT;
    homingDone = false;
    return;
  }

  switch (centeringState) {
    case CENTERING_MOVE_TO_CENTER:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        delay(20);

        // Read initial sensor state
        currentCenterSensorState = digitalRead(CENTER_LIMIT_PIN);
        lastCenterSensorState = currentCenterSensorState;
        centeringTransitionFound = false;

        // Calculate sweep parameters - SMALLER SWEEP
        long idealCenter = fullTravelSteps / 2;
        long sweepOffset = 80; // Reduced from 200 to 80 steps
        long sweepTarget;

        // Determine sweep direction based on current sensor state
        // If sensor is HIGH (left side), sweep right to find falling edge
        // If sensor is LOW (right side), sweep left to find rising edge
        if (currentCenterSensorState == HIGH) {
          // On left side, sweep RIGHT to cross center
          sweepTarget = idealCenter + sweepOffset;
          lcd.setCursor(0, 1);
          lcd.print(F("Sweep R (find fall)"));
        } else {
          // On right side, sweep LEFT to cross center
          sweepTarget = idealCenter - sweepOffset;
          lcd.setCursor(0, 1);
          lcd.print(F("Sweep L (find rise)"));
        }

        // Ensure we don't exceed travel limits
        sweepTarget = constrain(sweepTarget, 0, fullTravelSteps);

        stepper.setMaxSpeed(centerCoarseFineSpeed);
        stepper.setAcceleration(stepperAcceleration);
        stepper.moveTo(sweepTarget);
        centeringState = CENTERING_FIND_TRANSITION;
      }
      break;

    case CENTERING_FIND_TRANSITION:
      stepper.run(); // Use run() instead of runSpeed() for position-based movement

      // Detect edge transition ONCE
      if (sensorChanged && !centeringTransitionFound) {
        centeringTransitionFound = true;
        long transitionPosition = stepper.currentPosition();

        // Stop immediately at transition point
        stepper.stop();
        delay(10);

        // This IS the center - no offset calculation needed
        stepper.setMaxSpeed(centerFineSpeed);
        stepper.setAcceleration(stepperAcceleration);
        stepper.moveTo(transitionPosition); // Stay exactly here

        lcd.setCursor(0, 1);
        lcd.print(F("Edge found!      "));
        delay(300);

        centeringState = CENTERING_COMPLETE;
      }

      // Safety: abort on limits
      if (leftLimitState || rightLimitState) {
        stepper.stop();
        centeringActive = false;
        centeringState = CENTERING_IDLE;

        // Re-enable PID if in auto mode
        if (!isManualMode) {
          myPID.SetMode(AUTOMATIC);
        }

        lcd.clear();
        lcdPrint_P(STR_LIMIT_HIT);
        delay(1000);
        updateMainDisplay();
      }

      // Safety: if sweep completes without finding edge, we missed it
      if (stepper.distanceToGo() == 0 && !centeringTransitionFound) {
        stepper.stop();
        centeringActive = false;
        centeringState = CENTERING_IDLE;

        // START FALLBACK HOMING
        lcd.clear();
        lcd.setCursor(0, 0);
        lcdPrint_P(STR_EDGE_NOT_FOUND);
        lcd.setCursor(0, 1);
        lcdPrint_P(STR_HOMING_TO_LEFT);
        delay(1500);
        homeToLeft = true;
        homingState = HOMING_INIT;
        homingDone = false;
        return; // Exit to allow homing to take over
      }
      break;

    case CENTERING_COMPLETE:
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        delay(50);

        // === COMPLETE STOP ===
        stepper.stop();
        stepper.setSpeed(0);
        long finalPos = stepper.currentPosition();
        stepper.setCurrentPosition(finalPos);
        stepper.moveTo(finalPos);

        currentPosition = finalPos;

        // Display completion
        lcd.clear();
        lcd.setCursor(0, 0);
        lcdPrint_P(STR_CENTER_OK);
        lcd.setCursor(0, 1);
        char buf[21];
        snprintf(buf, sizeof(buf), "Pos:%ld", currentPosition);
        lcd.print(buf);
        digitalWrite(CENTER_LED_PIN, HIGH);

        // === CRITICAL: DEACTIVATE CENTERING COMPLETELY ===
        centeringActive = false;
        centeringState = CENTERING_IDLE;

        // Re-enable PID if in auto mode
        if (!isManualMode) {
          myPID.SetMode(AUTOMATIC);
        }

        delay(1500);

        // Return to normal display
        updateMainDisplay();
      }
      break;

    default:
      centeringActive = false;
      centeringState = CENTERING_IDLE;

      // Re-enable PID if in auto mode
      if (!isManualMode) {
        myPID.SetMode(AUTOMATIC);
      }
      break;
  }
}

// ====================================================================
// CENTER POSITION VERIFICATION
// ====================================================================
void verifyCenterPosition() {
  long expectedCenter = fullTravelSteps / 2;
  long actualPosition = stepper.currentPosition();

  const int POSITION_TOLERANCE = 5;

  if (abs(actualPosition - expectedCenter) > POSITION_TOLERANCE) {
    // Position is off, correct it
    stepper.setMaxSpeed(centerFineSpeed);
    stepper.setAcceleration(stepperAcceleration / 2);
    stepper.moveTo(expectedCenter);

    unsigned long startTime = millis();
    const unsigned long MOVE_TIMEOUT = 5000;

    while (stepper.distanceToGo() != 0 && (millis() - startTime < MOVE_TIMEOUT)) {
      stepper.run();
      wdt_reset();
    }

    // === FORCE ABSOLUTE STOP AFTER CORRECTION ===
    stepper.stop();
    stepper.setSpeed(0);
    long correctedPos = stepper.currentPosition();
    stepper.setCurrentPosition(correctedPos);
    stepper.moveTo(correctedPos);

    stepper.setMaxSpeed(stepperMaxSpeed);
    stepper.setAcceleration(stepperAcceleration);

    if (abs(stepper.currentPosition() - expectedCenter) > POSITION_TOLERANCE) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Centering Error!"));
      lcd.setCursor(0, 1);
      lcd.print(F("Try manual adj"));
      delay(2000);
    }
  }

  currentPosition = stepper.currentPosition();
  stepper.setCurrentPosition(currentPosition);
}

// ====================================================================
// LED CONTROL
// ====================================================================
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
// MENU SYSTEM FUNCTIONS
// ====================================================================
void initMenuValues() {
  for (int i = 0; i < numMenuItems; i++) {
    lastMenuValues[i] = 0;
  }
}

bool isNumericItem(int idx) {
  switch(idx) {
    case 0: case 1: case 2: case 4: case 5: case 7: case 8: case 9: case 10:
    case 12: case 13: case 14: case 15: case 16: case 17: case 18: return true;
    default: return false;
  }
}

bool isToggleItem(int idx) {
  switch(idx) {
    case 3: case 6: return true;
    default: return false;
  }
}

void drawMenu() {
  wdt_reset();
  static unsigned long lastMenuUpdate = 0;
  static bool lastBlinkState = true;

  if (millis() - lastMenuUpdate < 50 && blinkState == lastBlinkState) return;
  lastMenuUpdate = millis();
  lastBlinkState = blinkState;

  bool forceRedraw = (lastMenuTopItem == -1) ||
                     (lastCurrentMenuItem != currentMenuItem) ||
                     (lastMenuTopItem != menuTopItem);

  for (int i = 0; i < 2; i++) {
    int idx = menuTopItem + i;

    if (idx < 0 || idx >= numMenuItems) continue;

    long currentValue = getMenuValueInt(idx);

    if (forceRedraw || lastMenuValues[idx] != currentValue || (menuState == MENU_DIGIT_EDIT && idx == currentMenuItem)) {
      lcd.setCursor(0, i);
      lcd.print(F("                    "));

      if (idx < numMenuItems) {
        lcd.setCursor(0, i);
        lcd.print(idx == currentMenuItem ? ">" : " ");
        lcd.print(menuItems[idx]);

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

        int valueLen = strlen(valueStr);
        if (valueLen <= 20) {
          lcd.setCursor(20 - valueLen, i);
          lcd.print(valueStr);
        }
        lastMenuValues[idx] = currentValue;
      }
    }
  }

  lastMenuTopItem = menuTopItem;
  lastCurrentMenuItem = currentMenuItem;
}

void drawCalibMenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;

  if (lastCalibMenuItem != currentCalibMenuItem || millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    lcd.clear();

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
    case 14: return manualModeSpeed;      // Manual Speed
    case 15: return stepperAcceleration;   // Acceleration
    case 16: return buttonHoldInterval;    // Button Hold
    case 17: return (long)(dynamicSpeedFactor * 100);  // Dynamic Speed
    case 18: return centerMoveSpeed;       // Center Speed
    case 19: return 0;                     // Motor current (submenu)
    case 20: return 0;                     // Home (submenu)
    case 21: return 0;                     // EXIT MENU
    default: return 0;
  }
}

void adjustParameter(int delta) {
  switch(currentMenuItem) {
    case 3:
      pidDirection = (pidDirection == DIRECT ? REVERSE : DIRECT);
      myPID.SetControllerDirection(pidDirection);
      break;
    case 6:
      // Stop motion cleanly
      stepper.stop();
      long pos = stepper.currentPosition();
      stepper.setCurrentPosition(pos);
      stepper.moveTo(pos);

      isManualMode = !isManualMode;
      digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);

      if (isManualMode) {
        handleManualButtons();
        myPID.SetMode(MANUAL);
        output = 0.0;
      } else {
        myPID.SetMode(AUTOMATIC);
        setStepperSpeedsForMode();
      }
      break;
  }
}

// ====================================================================
// UPDATE parseEditedValue() FUNCTION
// ====================================================================
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
    case 7: stepperMinSpeed = constrain((int)atol(tempStr), STEPPER_MIN_SPEED_MIN, stepperMaxSpeed); break;
    case 8: stepperMaxSpeed = constrain((int)atol(tempStr), stepperMinSpeed, STEPPER_MAX_SPEED_MAX); break;
    case 9: homingSpeed = constrain((int)atol(tempStr), HOMING_SPEED_MIN, HOMING_SPEED_MAX); break;
    case 10: autoModeSpeed = constrain((int)atol(tempStr), AUTO_MODE_SPEED_MIN, AUTO_MODE_SPEED_MAX); break;
    case 12: stepManualFactor = constrain((int)atol(tempStr), STEP_MANUAL_FACTOR_MIN, STEP_MANUAL_FACTOR_MAX); break;
    case 13: stepManualFactorSW = constrain((int)atol(tempStr), STEP_MANUAL_FACTOR_SW_MIN, STEP_MANUAL_FACTOR_SW_MAX); break;
    case 14: manualModeSpeed = constrain((int)atol(tempStr), MANUAL_MODE_SPEED_MIN, MANUAL_MODE_SPEED_MAX); break;  // Manual Speed
    case 15: stepperAcceleration = constrain((int)atol(tempStr), STEPPER_ACCELERATION_MIN, STEPPER_ACCELERATION_MAX); break;  // Acceleration
    case 16: buttonHoldInterval = constrain((int)atol(tempStr), BUTTON_HOLD_INTERVAL_MIN, BUTTON_HOLD_INTERVAL_MAX); break;  // Button Hold
    case 17: dynamicSpeedFactor = constrain(atof(tempStr) / 100.0, DYNAMIC_SPEED_FACTOR_MIN, DYNAMIC_SPEED_FACTOR_MAX); break;  // Dynamic Speed
    case 18: centerMoveSpeed = constrain((int)atol(tempStr), CENTER_MOVE_SPEED_MIN, CENTER_MOVE_SPEED_MAX); break;  // Center Speed
  }

  wdt_reset();
}

// ====================================================================
// CALIBRATION
// ====================================================================
void runCalibration() {
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
      delay(150);
      lcd.setCursor(0, 0);
      lcd.print(F("Calibration Mode"));
      lcd.setCursor(0, 1);
      lcd.print(F("Finding Left Limit"));
      calibTimer = currentMillis;
      stepper.setSpeed(calibSpeed);
      calibState = CALIB_MOVE_LEFT;
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
        delay(50);
        stepper.setCurrentPosition(0);
        currentPosition = 0;
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        calibTimer = currentMillis;
        calibState = CALIB_LEFT_CONFIRM;
      }
      break;

    case CALIB_LEFT_CONFIRM:
      wdt_reset();
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      digitalWrite(CENTER_LED_PIN, LOW);
      digitalWrite(SET_LED_PIN, LOW);
      if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
        calibState = CALIB_IDLE;
        menuState = NORMAL;
        lcd.clear();
        delay(150);
        lcd.setCursor(0, 0);
        lcd.print(F("Calib Timed Out"));
        delay(1000);
        wdt_enable(WDTO_4S);
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
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
            stepper.setSpeed(-calibSpeed);
            digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
            digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
            digitalWrite(CENTER_LED_PIN, LOW);
            digitalWrite(SET_LED_PIN, LOW);
            lcd.clear();
            delay(150);
            lcd.setCursor(0, 0);
            lcd.print(F("Finding Right Limit"));
            calibState = CALIB_MOVE_RIGHT;
          } else {
            calibState = CALIB_IDLE;
            menuState = NORMAL;
            lcd.clear();
            delay(150);
            lcd.print(F("Calib Cancelled"));
            delay(1000);
            wdt_enable(WDTO_4S);
            displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
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
        delay(50);
        fullTravelSteps = stepper.currentPosition();
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        digitalWrite(CENTER_LED_PIN, LOW);
        digitalWrite(SET_LED_PIN, LOW);
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        calibTimer = currentMillis;
        calibState = CALIB_RIGHT_CONFIRM;
      }
      break;

    case CALIB_RIGHT_CONFIRM:
  wdt_reset();
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(SET_LED_PIN, LOW);

  if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
    calibState = CALIB_IDLE;
    menuState = NORMAL;
    lcd.clear();
    delay(150);
    lcd.setCursor(0, 0);
    lcd.print(F("Calib Timed Out"));
    delay(1000);
    wdt_enable(WDTO_4S);
    displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
    break;
  }

  if (delta != 0) {
    currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
    lastCalibMenuItem = -1;
  }

  if (setButtonState == LOW && setButtonPrevState == HIGH) {
    setButtonDownTime = currentMillis;
  }
  else if (setButtonState == HIGH && setButtonPrevState == LOW) {
    unsigned long held = currentMillis - setButtonDownTime;
    if (held >= AUTO_BUTTON_DEBOUNCE_MS && held < SET_BUTTON_LONG_PRESS_MS) {
      if (currentCalibMenuItem == 0) {
        // Save and exit - NO auto-centering
        saveSettings();
        calibState = CALIB_IDLE;
        menuState = NORMAL;
        lcd.clear();
        lcd.print(F("Calib Saved"));
        delay(500);
        updateMainDisplay();  // Just show normal display
        wdt_enable(WDTO_4S);
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
      } else {
        // Cancel calibration
        calibState = CALIB_IDLE;
        menuState = NORMAL;
        lcd.clear();
        delay(150);
        lcd.print(F("Calib Cancelled"));
        delay(1000);
        wdt_enable(WDTO_4S);
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
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
}

void updateMainDisplay() {
  // Skip update if position hasn't changed and update interval hasn't passed
  if (currentPosition == lastDisplayedPosition &&
      millis() - lastPositionUpdate < POSITION_UPDATE_INTERVAL) {
    return;
  }

  lcd.clear();
  delayMicroseconds(50);

  // Format voltage with error checking
  char voltageStr[7];

  // Ensure input is valid
  if (input < 0.0 || input > 5.5) {
    strncpy(voltageStr, "x.xx", 7);
  } else {
    // Format as "X.XX"
    dtostrf(input, 4, 2, voltageStr);
  }

  // Use full words, but shorten "Manual" → "MAN" only if space is tight
  const char* modeStr = isManualMode ? STR_MANUAL : STR_AUTO;

  // Position: "P:12345" → max ~8 chars (e.g., P:99999)
  char posStr[9];
  snprintf(posStr, sizeof(posStr), "P:%ld", currentPosition);
  posStr[8] = '\0';

  // Build line: "S:2.50V Auto      P:1234"
  char line0[21];
  int printed = snprintf(line0, 20, "S:%sV %s", voltageStr, modeStr);
  if (printed < 0 || printed >= 20) {
    // Fallback on overflow (unlikely, but safe)
    strncpy(line0, "S:x.xxV ??", 20);
    line0[10] = '\0';
  }

  // Right-align position
  int posLen = strlen(posStr);
  int spaceAvailable = 20 - strlen(line0);
  if (spaceAvailable >= posLen) {
    for (int i = 0; i < spaceAvailable - posLen; i++) {
      strncat(line0, " ", 1);
    }
    strncat(line0, posStr, posLen);
  } else {
    // Not enough room? Prioritize position — truncate mode
    const char* shortMode = isManualMode ? "MAN" : "AUT";
    snprintf(line0, 20, "S:%sV %s", voltageStr, shortMode);
    spaceAvailable = 20 - strlen(line0);
    if (spaceAvailable >= posLen) {
      for (int i = 0; i < spaceAvailable - posLen; i++) strncat(line0, " ", 1);
      strncat(line0, posStr, posLen);
    } else {
      // Absolute fallback: show voltage + position, drop mode
      snprintf(line0, 20, "S:%sV %s", voltageStr, posStr);
    }
  }
  line0[20] = '\0';

  lcd.setCursor(0, 0);
  lcd.print(line0);

  // Line 1: Position graph
  displayPositionGraph();

  // Update trackers
  lastDisplayedPosition = currentPosition;
  lastPositionUpdate = millis();
  lastFullDisplayRefresh = millis();
}

// ====================================================================
// UPDATE getMenuValueString() FUNCTION
// ====================================================================
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
      for (int i = 0; temp[i] ==' '; i++) temp[i] = '_';
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
    case 14:  // MANUAL SPEED
      snprintf(temp, 24, "%4d", manualModeSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
    case 15:  // ACCELERATION
      snprintf(temp, 24, "%4d", stepperAcceleration);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
    case 16:  // BUTTON HOLD
      snprintf(temp, 24, "%4d", buttonHoldInterval);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
    case 17:  // DYN SPEED
      snprintf(temp, 24, "%3d", (int)(dynamicSpeedFactor * 100));
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
    case 18:  // CENTER SPEED
      snprintf(temp, 24, "%4d", centerMoveSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strncpy(buffer, temp, 23);
      break;
    case 19: strcpy(buffer, ""); break;  // Motor current
    case 20: strcpy(buffer, ""); break;  // Home
    case 21: strcpy(buffer, ""); break;  // EXIT MENU
    default: strcpy(buffer, ""); break;
  }
  buffer[23] = '\0';
}

void displayPositionGraph(bool isLeft) {
  lcd.setCursor(0, 1);
  if (fullTravelSteps <= 0) {
    lcdPrint_P(STR_NEEDS_CALIBRATION);
    return;
  }
  if (leftLimitState) {
    lcdPrint_P(STR_LEFT_LIMIT_ROMANIAN);
    return;
  }
  if (rightLimitState) {
    lcdPrint_P(STR_RIGHT_LIMIT_ROMANIAN);
    return;
  }

  lcd.print(F("                    "));

  lcd.setCursor(9, 1);
  lcd.write((uint8_t)6);
  lcd.setCursor(10, 1);
  lcd.write((uint8_t)7);

  long pos = currentPosition;
  long centerPos = fullTravelSteps / 2;
  long halfRange = fullTravelSteps / 2;
  if (halfRange <= 0) return;

  long distanceFromCenter = abs(pos - centerPos);
  long stepsPerChar = halfRange / 10;
  if (stepsPerChar <= 0) stepsPerChar = 1;

  int fullChars = (int)(distanceFromCenter / stepsPerChar);
  if (fullChars > 9) fullChars = 9;

  long remainder = distanceFromCenter % stepsPerChar;
  int partialFill = (int)map(remainder, 0, stepsPerChar, 0, 5);

  bool isLeftOfCenter = pos < centerPos;
  static bool lastDirection = true;

  if (isLeftOfCenter != lastDirection) {
    if (isLeftOfCenter) {
      for (int i = 0; i < 8; i++) lcd.createChar(i, barChars[i]);
    } else {
      for (int i = 0; i < 8; i++) lcd.createChar(i, barCharsMirrored[i]);
    }
    lastDirection = isLeftOfCenter;
  }

  for (int i = 0; i < 10; i++) {
    int lcdPos = isLeftOfCenter ? (9 - i) : (10 + i);
    lcd.setCursor(lcdPos, 1);
    if (i < fullChars) {
      lcd.write((uint8_t)5);
    } else if (i == fullChars) {
      lcd.write((uint8_t)partialFill);
    } else {
      lcd.write((uint8_t)0);
    }
  }
}

bool updateLimitSwitch(int pin, unsigned long& debounceTime, bool& lastState) {
  unsigned long now = millis();
  bool rawState = digitalRead(pin);
  if (rawState != lastState && (now - debounceTime) >= LIMIT_DEBOUNCE_MS) {
    lastState = rawState;
    debounceTime = now;
    return LIMIT_ACTIVE_HIGH ? (rawState == HIGH) : (rawState == LOW);
  }
  return LIMIT_ACTIVE_HIGH ? (lastState == HIGH) : (lastState == LOW);
}

// ====================================================================
// BUTTON HANDLERS
// ====================================================================
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
      if (menuState == NORMAL) {
        menuState = MENU_BROWSE;
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
        for (int i = 0; i < numMenuItems; i++) {
          lastMenuValues[i] = 0;
        }
        lcd.clear();
        delay(150);
        drawMenu();
        // Clear system error when entering menu
        if (systemErrorState) {
          clearSystemError();
        }
      } else {
        menuState = NORMAL;
        lcd.clear();
        delay(150);
        updateMainDisplay();
        displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
      }
    } else if (held >= AUTO_BUTTON_DEBOUNCE_MS && menuState != NORMAL) {
      if (menuState == MENU_BROWSE) {
        // Menu structure after adding "Manual Speed":
        // 0-13: Settings
        // 14: Manual Speed (NEW)
        // 15: Acceleration
        // 16: Button Hold
        // 17: Dyn Speed
        // 18: Center Speed
        // 19: Motor Current (SUB-MENU)
        // 20: Home... (SUB-MENU)
        // 21: EXIT MENU

        if (currentMenuItem == 11) {
          // Calibration menu
          menuState = MENU_CALIB;
          calibState = CALIB_INIT;
          currentCalibMenuItem = 0;
          lastCalibMenuItem = -1;
          lcd.clear();
          delay(150);
        }
        else if (currentMenuItem == 19) {
          // Motor Current submenu
          menuState = MENU_MOTOR_CURRENT_SUB;
          currentMotorCurrentSubMenuItem = (int)currentMotorLevel;
          lastMotorCurrentSubMenuItem = -1;
          lcd.clear();
          delay(150);
          drawMotorCurrentSubmenu();
        }
        else if (currentMenuItem == 20) {
          // Homing submenu
          menuState = MENU_HOMING_SUB;
          currentHomingSubMenuItem = 0;
          lastHomingSubMenuItem = -1;
          lcd.clear();
          delay(150);
          drawHomingSubmenu();
        }
        else if (currentMenuItem == 21) {
          // Exit & Save
          menuState = MENU_EXIT_CONFIRM;
          currentCalibMenuItem = 0;
          lastCalibMenuItem = -1;
          lcd.clear();
          delay(150);
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
        saveSettings();
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
      }
      else if (menuState == MENU_DIGIT_EDIT) {
        do {
          currentDigitPos++;
          if (editValueStr[currentDigitPos] == '\0') {
            parseEditedValue(currentMenuItem);
            menuState = MENU_BROWSE;
            saveSettings();
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
          delay(150);
          drawMenu();
        } else {
          setMotorCurrentLevel((MotorCurrentLevel)currentMotorCurrentSubMenuItem);
          saveSettings();
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          delay(150);
          drawMenu();
        }
      }
      else if (menuState == MENU_HOMING_SUB) {
        if (currentHomingSubMenuItem == 0) {
          homeToLeft = true;
          menuState = NORMAL;
          lcd.clear();
          delay(150);
          homingState = HOMING_INIT;
          homingDone = false;
          justFinishedHoming = false;
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        }
        else if (currentHomingSubMenuItem == 1) {
          homeToLeft = false;
          menuState = NORMAL;
          lcd.clear();
          delay(150);
          homingState = HOMING_INIT;
          homingDone = false;
          justFinishedHoming = false;
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        }
        else if (currentHomingSubMenuItem == 2) {
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          delay(150);
          drawMenu();
        }
      }
      else if (menuState == MENU_EXIT_CONFIRM) {
        if (currentCalibMenuItem == 0) {
          saveSettings();
          menuState = NORMAL;
          lcd.clear();
          delay(150);
          updateMainDisplay();
          displayLockoutUntil = millis() + DISPLAY_LOCKOUT_MS;
        } else {
          menuState = MENU_BROWSE;
          lastMenuTopItem = -1;
          lastCurrentMenuItem = -1;
          lcd.clear();
          delay(150);
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

void drawMotorCurrentSubmenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;

  if (lastMotorCurrentSubMenuItem != currentMotorCurrentSubMenuItem || millis() - lastUpdate > 100) {
    lastUpdate = millis();
    lcd.clear();

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

void drawHomingSubmenu() {
  wdt_reset();
  static unsigned long lastUpdate = 0;

  if (lastHomingSubMenuItem != currentHomingSubMenuItem || millis() - lastUpdate > 100) {
    lastUpdate = millis();
    lcd.clear();

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

// ====================================================================
// MENU SYSTEM FUNCTIONS
// ====================================================================
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
        adjustParameter(delta);
        drawMenu();
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

  if (menuState == NORMAL) {
    bool encBtnState = digitalRead(ENCODER_SW);
    unsigned long now_enc = millis();
    if (encBtnState == LOW && encoderButtonPrevState == HIGH) {
      encoderButtonDownTime = now_enc;
    } else if (encBtnState == HIGH && encoderButtonPrevState == LOW) {
      unsigned long held = now_enc - encoderButtonDownTime;
      if (held >= SHORT_PRESS_MS && fullTravelSteps > 0 && !centeringActive) {
        startSmartCentering();
      }
    }
    encoderButtonPrevState = encBtnState;
  }

  if (menuState != NORMAL || menuState != lastMenuState) {
    if (menuState == MENU_EXIT_CONFIRM) {
      drawCalibMenu();
    } else if (menuState == MENU_HOMING_SUB) {
      drawHomingSubmenu();
    } else if (menuState == MENU_MOTOR_CURRENT_SUB) {
      drawMotorCurrentSubmenu();
    } else if (menuState != NORMAL) {
      drawMenu();
    }
  }
  lastMenuState = menuState;
}

void handleAutoButton() {
  bool autoButtonState = digitalRead(AUTO_BUTTON_PIN);
  unsigned long now = millis();

  if (autoButtonState == LOW && autoButtonPrevState == HIGH) {
    // Button pressed
    autoButtonDownTime = now;
  }
  else if (autoButtonState == HIGH && autoButtonPrevState == LOW) {
    // Button released
    unsigned long held = now - autoButtonDownTime;

    if (held >= AUTO_BUTTON_DEBOUNCE_MS) {
      // Valid press - toggle mode

      // === COMPLETE STOP BEFORE SWITCHING ===
      stepper.stop();
      delay(10);
      long pos = stepper.currentPosition();
      stepper.setCurrentPosition(pos);
      stepper.moveTo(pos);
      delay(10);

      // === TOGGLE MODE ===
      isManualMode = !isManualMode;
      digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);

      // === UPDATE PID STATE ===
      if (isManualMode) {
        // Switching TO manual mode
        myPID.SetMode(MANUAL);
        output = 0.0;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Manual Mode"));
        delay(500);
        updateMainDisplay();
      } else {
        // Switching TO auto mode
        myPID.SetMode(AUTOMATIC);

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Auto Mode"));
        delay(500);
        updateMainDisplay();
      }

      // === SAVE MODE TO EEPROM ===
      saveSettings();
    }
  }

  autoButtonPrevState = autoButtonState;
}


void handleCenterButton() {
  bool centerButtonState = digitalRead(CENTER_BUTTON_PIN);
  unsigned long now = millis();

  if (centerButtonState == LOW && centerButtonPrevState == HIGH) {
    centerButtonDownTime = now;
  } else if (centerButtonState == HIGH && centerButtonPrevState == LOW) {
    unsigned long held = now - centerButtonDownTime;
    if (held >= CENTER_BUTTON_DEBOUNCE_MS && homingDone && !centeringActive) {
      if (fullTravelSteps > 0) {
        startSmartCentering();
      }
    }
  }
  centerButtonPrevState = centerButtonState;
}

// ====================================================================
// SETUP
// ====================================================================
void setup() {
  pinMode(LEFT_LIMIT_PIN, INPUT);
  pinMode(RIGHT_LIMIT_PIN, INPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CENTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AUTO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SET_BUTTON_PIN, INPUT_PULLUP);

  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  pinMode(CENTER_LED_PIN, OUTPUT);
  pinMode(MODE_LED_PIN, OUTPUT);
  pinMode(SET_LED_PIN, OUTPUT);

  pinMode(C0_PIN, OUTPUT);
  pinMode(C1_PIN, OUTPUT);

  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  digitalWrite(CENTER_LED_PIN, LOW);
  digitalWrite(SET_LED_PIN, LOW);
  digitalWrite(MODE_LED_PIN, HIGH);

  loadSettings();



  lcd.begin(20, 2);
  delay(100);
  createBarGraphChars();
  lcd.setCursor(0, 0);
  lcdPrint_P(STR_WEB_GUIDE);
  lcd.setCursor(0, 1);
  lcdPrint_P(STR_INITIALIZING);
  delay(2000);
  lcd.clear();
  delay(150);
  loadSettings();

  // Initialize limit switch states
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

  stepper.setMaxSpeed(stepperMaxSpeed);
  stepper.setAcceleration(stepperAcceleration);
  stepper.setPinsInverted(true, false, false);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-stepperMaxSpeed, stepperMaxSpeed);
  myPID.SetSampleTime(20);
  initMenuValues();
  lastEncoderPos = myEnc.read();
  lastHandledEncoderPos = myEnc.read();
  lastUpdateTime = millis();
  voltageReadTimer = millis();
  wdt_enable(WDTO_4S);

  // ===== MANDATORY STARTUP CENTERING =====
  if (fullTravelSteps > 0) {
    // Check limits to set initial position before centering
    leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
    rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
    if (leftLimitState && !rightLimitState) {
      stepper.setCurrentPosition(0);
    } else if (rightLimitState && !leftLimitState) {
      stepper.setCurrentPosition(fullTravelSteps);
    }

    // Calibration exists, so we must find the center.
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("System Start"));
    lcd.setCursor(0, 1);
    lcd.print(F("Centering..."));
    delay(1500);

    // Set homingDone to true to allow centering to start
    homingDone = true;
    startSmartCentering();
  } else {
    // No calibration data. User must calibrate manually.
    showSystemError();
    homingDone = false; // Prevent operations until calibrated
    homingState = HOMING_IDLE;
  }


}  // *** END OF loop() ***

// ====================================================================
// MAIN LOOP — POSITION-BASED CONTROL IN ALL MODES
// ====================================================================
void loop() {
  wdt_reset();
  unsigned long currentMillis = millis();

  // Always process menu inputs, even if there's a system error
  handleSetButton();
  handleAutoButton();
  handleCenterButton();
  handleMenuSystem();

  // Handle system error display timeout
  if (systemErrorState && (currentMillis - systemErrorDisplayTime > SYSTEM_ERROR_DISPLAY_DURATION)) {
    if ((currentMillis / 1000) % 2 == 0) {
      lcd.setCursor(0, 1);
      lcdPrint_P(STR_PRESS_SET_MENU);
    } else {
      lcd.setCursor(0, 1);
      lcdPrint_P(STR_CALIBRATE_FIRST);
    }
  }

  // Check system status periodically (only when not homing)
  static unsigned long lastSystemCheck = 0;
  if (homingDone && (currentMillis - lastSystemCheck > 10000)) {
    checkSystemStatus();
    lastSystemCheck = currentMillis;
  }

  // Update limit switch states
  static unsigned long lastLimitSwitchUpdate = 0;
  if (currentMillis - lastLimitSwitchUpdate > 10) {
    leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
    rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
    lastLimitSwitchUpdate = currentMillis;
  }

  // Read sensor voltage periodically
  updateVoltageReading();

  // ===== HANDLE NON-OPERATIONAL STATES FIRST =====
  if (!homingDone && homingState != HOMING_IDLE) {
    runHomingSequence();
    return;
  }

  if (calibState != CALIB_IDLE) {
    runCalibration();
    return;
  }

  if (centeringActive) {
    runSmartCentering();
    return;
  }

  if (justFinishedHoming) {
    lcd.clear();
    updateMainDisplay();
    justFinishedHoming = false;
  }

  updateSetLedBlink();

  // Periodic full LCD refresh
  if (menuState == NORMAL &&
      calibState == CALIB_IDLE &&
      homingState == HOMING_IDLE &&
      !centeringActive &&
      !systemErrorState &&
      (currentMillis - lastFullDisplayRefresh >= FULL_DISPLAY_REFRESH_INTERVAL)) {
    updateMainDisplay();
    lastFullDisplayRefresh = currentMillis;
  }

  // ===== OPERATIONAL MODES (only if system is OK) =====
  if (menuState == NORMAL && calibState == CALIB_IDLE && !centeringActive && !systemErrorState) {

    if (isManualMode) {
      // ===== MANUAL MODE =====
      handleManualButtons();
    }
    else {
      // ===== AUTO MODE: POSITION-BASED PID =====
      setStepperSpeedsForMode();



      // ===== NORMAL AUTO MODE: PID CONTROL =====
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        double error = setPoint - input;
        bool inDeadband = abs(error) < deadband;

        if (!inDeadband) {
          myPID.Compute();
        } else {
          output = 0;
        }

        static long targetPosition = stepper.currentPosition();

        if (inDeadband) {
          targetPosition = stepper.currentPosition();
        } else {
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

        // === ALLOW POSITION TO GO TO LIMITS ===
        // PID should be able to move to edges if sensor requires it
        if (fullTravelSteps > 0) {
          targetPosition = constrain(targetPosition, 0, fullTravelSteps);
        } else {
          long cur = stepper.currentPosition();
          targetPosition = constrain(targetPosition, cur - 1000, cur + 1000);
        }

        // === ALWAYS ALLOW MOVEMENT (no isSafeToMove restriction) ===
        stepper.moveTo(targetPosition);
      }

      stepper.run();
      currentPosition = stepper.currentPosition();
      updateDirectionLEDsFromVelocity(stepper.speed());

      if (currentMillis - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        updateMainDisplay();
        lastPositionUpdate = currentMillis;
      }
    }
  }
}  // *** END OF loop() ***

// ====================================================================
// SENSOR FUNCTION WITH TIMING
// ====================================================================
float readVoltage() {
  long sum = 0;
  const int samples = 10;

  // Read ADC multiple times and average
  for (int i = 0; i < samples; i++) {
    sum += analogRead(A0);
    delayMicroseconds(100);  // Use microseconds instead of milliseconds for faster reads
  }

  float adcValue = sum / (float)samples;

  // Convert ADC value to voltage
  float Vout = (adcValue / ADC_RESOLUTION) * ADC_REF_VOLTAGE;

  // Apply voltage divider formula: Vin = Vout * (R1 + R2) / R2
  float Vin = Vout * (R1 + R2) / R2;

  // Validate the reading (should be between 0V and ~5V for typical sensor)
  if (Vin < 0.0 || Vin > 5.5) {
    // If out of range, return last valid reading (don't update)
    return input;  // Keep previous value
  }

  return Vin;
}

// ====================================================================
// updateVoltageReading() - FASTER IN MANUAL MODE
// ====================================================================
void updateVoltageReading() {
  unsigned long now = millis();

  // In AUTO mode: read voltage frequently (20ms) for PID control
  // In MANUAL mode: read voltage slowly (500ms) - just for display, not critical
  unsigned long readInterval = isManualMode ? VOLTAGE_READ_INTERVAL_MANUAL : VOLTAGE_READ_INTERVAL_AUTO;

  if (now - voltageReadTimer >= readInterval) {
    voltageReadTimer = now;
    input = readVoltage();  // Update the global input variable
  }
}

// ====================================================================
// MANUAL MODE BUTTON HANDLING
// ====================================================================
bool readStableButtonState(int pin) {
  const int samples = 5;
  const int sampleDelay = 2;
  int lowCount = 0;
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) == LOW) lowCount++;
    delay(sampleDelay);
  }
  return lowCount >= (samples * 0.8);
}

// ====================================================================
// MANUAL MODE BUTTON HANDLING (OPTIMIZED)
// ====================================================================
void handleManualButtons() {
  unsigned long now = millis();
  wdt_reset();

  long newEncoderPos = myEnc.read();
  long encoderDiff = (newEncoderPos - lastHandledEncoderPos) / 4;

  long targetPos = stepper.currentPosition();

  // ===== ENCODER MOVEMENT (Priority 1) =====
  if (encoderDiff != 0) {
    // Encoder is being used
    stepper.setMaxSpeed(manualModeSpeed);
    stepper.setAcceleration(stepperAcceleration);

    long movementSteps = encoderDiff * stepManualFactor;
    targetPos = stepper.currentPosition() + movementSteps;
    lastHandledEncoderPos = newEncoderPos;
  }
  // ===== BUTTON MOVEMENT (Priority 2) =====
  else {
    setStepperSpeedsForMode();

    bool leftBtn = (digitalRead(LEFT_BUTTON_PIN) == LOW);
    bool rightBtn = (digitalRead(RIGHT_BUTTON_PIN) == LOW);

    if (leftBtn && !leftLimitState) {
      targetPos = stepper.currentPosition() - stepManualFactorSW;
    }
    else if (rightBtn && !rightLimitState) {
      targetPos = stepper.currentPosition() + stepManualFactorSW;
    }
  }

  // Always allow movement - PID knows what it's doing
  stepper.moveTo(targetPos);

  stepper.run();
  currentPosition = stepper.currentPosition();
  // Update limit states in real time
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

  updateDirectionLEDsFromVelocity(stepper.speed());

  if (now - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
    updateMainDisplay();
    lastPositionUpdate = now;
  }
}

// ====================================================================
// RESET ENCODER POSITION FUNCTION
// ====================================================================
void resetEncoderPosition() {
  lastEncoderPos = myEnc.read();
  lastHandledEncoderPos = myEnc.read();
}
