#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <PID_v1.h>
#include <Encoder.h>
#include <avr/wdt.h>
#include <ctype.h>

// ====================================================================
// PARAMETER CONFIGURATION
// ====================================================================
// PID Controller Parameters
const double KP_DEFAULT = 20.0;          // Default proportional gain
const double KP_MIN = 0.0;               // Minimum proportional gain
const double KP_MAX = 1000.0;            // Maximum proportional gain
double Kp = KP_DEFAULT;                  // Proportional gain
const double KI_DEFAULT = 0.0;           // Reduced for velocity mode
const double KI_MIN = 0.0;               // Minimum integral gain
const double KI_MAX = 1.0;               // Keep very low to avoid windup
double Ki = KI_DEFAULT;                  // Integral gain
const double KD_DEFAULT = 0.5;           // Default derivative gain
const double KD_MIN = 0.0;               // Minimum derivative gain
const double KD_MAX = 1000.0;            // Maximum derivative gain
double Kd = KD_DEFAULT;                  // Derivative gain
const int PID_DIRECTION_DEFAULT = DIRECT; // Default PID direction (DIRECT or REVERSE)
int pidDirection = PID_DIRECTION_DEFAULT; // PID direction
const double SETPOINT_DEFAULT = 2.5;      // Default target voltage (center position)
const double SETPOINT_MIN = 0.0;          // Minimum target voltage
const double SETPOINT_MAX = 5.0;          // Maximum target voltage
double setPoint = SETPOINT_DEFAULT;       // Target voltage
const double DEADBAND_DEFAULT = 0.05;     // Default deadband for small errors (V)
const double DEADBAND_MIN = 0.0;         // Minimum deadband
const double DEADBAND_MAX = 0.2;         // Maximum deadband
double deadband = DEADBAND_DEFAULT;       // Deadband

// Stepper Motor Parameters
const int STEPPER_MIN_SPEED_DEFAULT = 50;    // Default minimum stepper speed
const int STEPPER_MIN_SPEED_MIN = 0;         // Minimum stepper speed
int stepperMinSpeed = STEPPER_MIN_SPEED_DEFAULT; // Minimum stepper speed
const int STEPPER_MAX_SPEED_DEFAULT = 1000;  // Default maximum stepper speed
const int STEPPER_MAX_SPEED_MAX = 10000;     // Maximum stepper speed
int stepperMaxSpeed = STEPPER_MAX_SPEED_DEFAULT; // Maximum stepper speed

// Homing Speed Parameter - Marlin-like homing feedrate
const int HOMING_SPEED_DEFAULT = 500;        // Default homing speed
const int HOMING_SPEED_MIN = 50;             // Minimum homing speed
const int HOMING_SPEED_MAX = 10000;           // Maximum homing speed
int homingSpeed = HOMING_SPEED_DEFAULT;      // Homing speed

// Auto Mode Speed Parameter
const int AUTO_MODE_SPEED_DEFAULT = 800;     // Default auto mode speed
const int AUTO_MODE_SPEED_MIN = 50;          // Minimum auto mode speed
const int AUTO_MODE_SPEED_MAX = 10000;       // Maximum auto mode speed
int autoModeSpeed = AUTO_MODE_SPEED_DEFAULT; // Auto mode speed

// Adaptive Speed Parameters
const double LARGE_ERROR_THRESHOLD = 0.5;    // Fast movement threshold (V)
const double MEDIUM_ERROR_THRESHOLD = 0.2;   // Medium movement threshold (V) 
const int FAST_SPEED_MULTIPLIER = 3;         // 3x speed for large errors
const int MEDIUM_SPEED_MULTIPLIER = 2;       // 2x speed for medium errors

const long FULL_TRAVEL_STEPS_DEFAULT = 0;    // Default full travel steps (set by calibration)
const long FULL_TRAVEL_STEPS_MIN = 0;        // Minimum full travel steps
long fullTravelSteps = FULL_TRAVEL_STEPS_DEFAULT; // Full travel steps

const int STEP_MANUAL_FACTOR_DEFAULT = 50;   // Default manual velocity multiplier
const int STEP_MANUAL_FACTOR_MIN = 1;        // Minimum manual factor
const int STEP_MANUAL_FACTOR_MAX = 100;      // Maximum manual factor
int stepManualFactor = STEP_MANUAL_FACTOR_DEFAULT; // Manual factor (encoder)

const int STEP_MANUAL_FACTOR_SW_DEFAULT = 10; // Default manual factor (buttons)
const int STEP_MANUAL_FACTOR_SW_MIN = 1;      // Minimum manual factor (buttons)
const int STEP_MANUAL_FACTOR_SW_MAX = 100;    // Maximum manual factor (buttons)
int stepManualFactorSW = STEP_MANUAL_FACTOR_SW_DEFAULT; // Manual factor (buttons)

const int STEPPER_ACCELERATION_DEFAULT = 1000; // Increased acceleration for Marlin-like smooth moves
const int STEPPER_ACCELERATION_MIN = 1;       // Minimum stepper acceleration
const int STEPPER_ACCELERATION_MAX = 5000;    // Maximum stepper acceleration
int stepperAcceleration = STEPPER_ACCELERATION_DEFAULT; // Stepper acceleration

// Button Hold Interval (unused in velocity mode)
const int BUTTON_HOLD_INTERVAL_DEFAULT = 500; // Default hold interval for button repeat (ms)
const int BUTTON_HOLD_INTERVAL_MIN = 50;      // Minimum hold interval
const int BUTTON_HOLD_INTERVAL_MAX = 2000;    // Maximum hold interval
int buttonHoldInterval = BUTTON_HOLD_INTERVAL_DEFAULT; // Hold interval

// Dynamic Speed Factor for Manual
const float DYNAMIC_SPEED_FACTOR_DEFAULT = 1.0; // Default speed factor (100%)
const float DYNAMIC_SPEED_FACTOR_MIN = 0.1;     // Minimum speed factor (10%)
const float DYNAMIC_SPEED_FACTOR_MAX = 1.0;     // Maximum speed factor (100%)
float dynamicSpeedFactor = DYNAMIC_SPEED_FACTOR_DEFAULT; // Speed factor

// Operational Parameters
const bool IS_MANUAL_MODE_DEFAULT = false;    // Default operation mode (false=AUTO, true=MANUAL)
bool isManualMode = IS_MANUAL_MODE_DEFAULT;   // Operation mode

// LED Configuration
const bool INVERT_LED_LOGIC = false;          // Set to true if DIRECTION_PIN logic is inverted

// ====================================================================
// LCD CONFIGURATION
// ====================================================================
LiquidCrystal lcd(33, 35, 37, 39, 41, 43);

// ====================================================================
// ENCODER CONFIGURATION
// ====================================================================
Encoder myEnc(3, 2);              
#define ENCODER_SW 4              

// ====================================================================
// STEPPER MOTOR CONFIGURATION
// ====================================================================
#define STEP_PIN 13               
#define DIRECTION_PIN 12          
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIRECTION_PIN);

// ====================================================================
// LIMIT SWITCHES
// ====================================================================
#define LEFT_LIMIT_PIN 25         
#define RIGHT_LIMIT_PIN 23        
#define LIMIT_ACTIVE_HIGH true    

// ====================================================================
// BUTTONS AND LEDS
// ====================================================================
#define TOGGLE_BUTTON_PIN 27      
#define LEFT_BUTTON_PIN 29        
#define RIGHT_BUTTON_PIN 31       
#define MODE_LED_PIN 44           
#define LEFT_LED_PIN 50           
#define RIGHT_LED_PIN 48          

// ====================================================================
// PID & POSITION TRACKING
// ====================================================================
double input = 0;
double output = 0;                
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, pidDirection);

long currentPosition = 0;         // Hardware-tracked position (Marlin-like)
unsigned long lastUpdateTime = 0;

// ====================================================================
// HOMING STATE - Marlin-like G28
// ====================================================================
enum HomingState {
  HOMING_IDLE, HOMING_INIT, HOMING_INIT_DELAY,
  HOMING_LEFT, HOMING_TIMEOUT_DELAY, HOMING_LEFT_HIT_DELAY1, HOMING_LEFT_HIT_DELAY2,
  HOMING_CENTER, HOMING_CENTER_TIMEOUT_DELAY, HOMING_CENTER_DONE_DELAY,
  HOMING_COMPLETE, HOMING_COMPLETE_DELAY1, HOMING_COMPLETE_DELAY2
};
HomingState homingState = HOMING_IDLE;
bool homingDone = false;
unsigned long homingStartTime = 0;
const unsigned long HOMING_TIMEOUT_MS = 60000;  // Increased to 60s for safety

// ====================================================================
// CALIBRATION STATE
// ====================================================================
enum CalibState {
  CALIB_IDLE,
  CALIB_INIT, CALIB_INIT_DELAY,
  CALIB_MOVE_LEFT,
  CALIB_LEFT_CONFIRM,
  CALIB_MOVE_RIGHT,
  CALIB_RIGHT_CONFIRM,
  CALIB_TIMEOUT_DELAY,
  CALIB_CANCEL_DELAY,
  CALIB_SAVE_DELAY1, CALIB_SAVE_DELAY2
};
CalibState calibState = CALIB_IDLE;
unsigned long calibTimer = 0;
const int calibSpeed = 200;  // Increased calib speed
const unsigned long CALIB_TIMEOUT_MS = 60000;

// ====================================================================
// MENU SYSTEM
// ====================================================================
enum MenuState {
  NORMAL,
  MENU_BROWSE,
  MENU_EDIT,
  MENU_CALIB,
  MENU_DIGIT_EDIT,
  MENU_EXIT_CONFIRM,
  MENU_TRANSITION_TO_NORMAL,
  MENU_TRANSITION_TO_BROWSE,
  MENU_TRANSITION_TO_CALIB,
  MENU_TRANSITION_TO_EXIT_CONFIRM
};
MenuState menuState = NORMAL;
int currentMenuItem = 0;
int menuTopItem = 0;
const int LCD_MENU_ROWS = 4;
const char* menuItems[] = {
  "PID Kp", "PID Ki", "PID Kd", "PID Direction", "Sensor SetPoint",
  "Deadband", "Operation Mode", "Min Speed", "Max Speed",
  "Homing Speed", "Auto Speed", "Calib Steps", "Manual Factor", 
  "Manual Fac SW", "Acceleration", "Btn Hold Int", "Dyn Speed %", "EXIT MENU"
};
const int numMenuItems = sizeof(menuItems) / sizeof(menuItems[0]);
long lastMenuValues[20];
int lastMenuTopItem = -1;
int lastCurrentMenuItem = -1;
const char* calibMenuItems[] = { "Save", "Cancel" };
const int numCalibMenuItems = 2;
int currentCalibMenuItem = 0;
int lastCalibMenuItem = -1;

char editValueStr[24];
int currentDigitPos = 0;
unsigned long blinkTimer = 0;
bool blinkState = true;
const unsigned long BLINK_INTERVAL = 500;

// ====================================================================
// OPERATIONAL VARIABLES
// ====================================================================
unsigned long previousMillis = 0;
const long interval = 20;
long lastDisplayedPosition = -1;
unsigned long lastPositionUpdate = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 300;
unsigned long encoderButtonDownTime = 0;
bool encoderButtonPrevState = HIGH;
const unsigned long LONG_PRESS_MS = 1000;
const unsigned long SHORT_PRESS_MS = 50;
long lastEncoderPos = 0;
bool toggleButtonPrevState = LOW;
unsigned long toggleButtonDownTime = 0;
const unsigned long DEBOUNCE_MS = 100;
bool leftLimitState = LOW;
bool rightLimitState = LOW;
unsigned long leftLimitDebounceTime = 0;
unsigned long rightLimitDebounceTime = 0;
const unsigned long LIMIT_DEBOUNCE_MS = 50;

unsigned long displayLockoutUntil = 0;
const unsigned long DISPLAY_LOCKOUT_MS = 500;
unsigned long stateTimer = 0;

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

void createBarGraphChars() { 
  for (int i = 0; i < 8; i++) {
    lcd.createChar(i, barChars[i]); 
  }
}

enum SystemState { STATE_SETUP, STATE_RUNNING };
SystemState systemState = STATE_SETUP;

void loop() {
  if (systemState == STATE_SETUP) {
    setup_loop();
  } else {
    main_loop();
  }
}


// ====================================================================
// FUNCTION PROTOTYPES
// ====================================================================
void handleMenuSystem();
void runCalibration();
void handleToggleButton();
void handleManualButtons();
float readVoltage();
void displayPositionGraph(bool isLeft);
void updateMainDisplay();
void saveSettings();
void loadSettings();
void initMenuValues();
void drawMenu();
void drawCalibMenu();
long getMenuValueInt(int idx);
void adjustParameter(int delta);
void getMenuValueString(int idx, char* buffer);
bool updateLimitSwitch(int pin, unsigned long& debounceTime, bool& lastState);
bool isNumericItem(int idx);
bool isToggleItem(int idx);
void parseEditedValue(int idx);
void updateDirectionLEDsFromVelocity(float speed);
bool readStableButtonState(int pin);
void startCentering();
void runHomingSequence();  // Marlin-like homing

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
    (float)deadband, isManualMode, stepperMinSpeed, stepperMaxSpeed,
    homingSpeed, autoModeSpeed, fullTravelSteps, stepManualFactor, 
    stepManualFactorSW, stepperAcceleration, buttonHoldInterval, 
    dynamicSpeedFactor, 0
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
  } else {
    Kp = (isnan(settings.Kp) || settings.Kp < KP_MIN || settings.Kp > KP_MAX) ? KP_DEFAULT : settings.Kp;
    Ki = (isnan(settings.Ki) || settings.Ki < KI_MIN || settings.Ki > KI_MAX) ? KI_DEFAULT : settings.Ki;
    Kd = (isnan(settings.Kd) || settings.Kd < KD_MIN || settings.Kd > KD_MAX) ? KD_DEFAULT : settings.Kd;
    pidDirection = (settings.pidDirection == DIRECT || settings.pidDirection == REVERSE) ? settings.pidDirection : PID_DIRECTION_DEFAULT;
    setPoint = (isnan(settings.setPoint) || settings.setPoint < SETPOINT_MIN || settings.setPoint > SETPOINT_MAX) ? SETPOINT_DEFAULT : settings.setPoint;
    deadband = (isnan(settings.deadband) || settings.deadband < DEADBAND_MIN || settings.deadband > DEADBAND_MAX) ? DEADBAND_DEFAULT : settings.deadband;
    isManualMode = settings.isManualMode;
    stepperMinSpeed = (settings.stepperMinSpeed < STEPPER_MIN_SPEED_MIN || settings.stepperMinSpeed > settings.stepperMaxSpeed) ? STEPPER_MIN_SPEED_DEFAULT : settings.stepperMinSpeed;
    stepperMaxSpeed = (settings.stepperMaxSpeed < settings.stepperMinSpeed || settings.stepperMaxSpeed > STEPPER_MAX_SPEED_MAX) ? STEPPER_MAX_SPEED_DEFAULT : settings.stepperMaxSpeed;
    homingSpeed = (settings.homingSpeed < HOMING_SPEED_MIN || settings.homingSpeed > HOMING_SPEED_MAX) ? HOMING_SPEED_DEFAULT : settings.homingSpeed;
    autoModeSpeed = (settings.autoModeSpeed < AUTO_MODE_SPEED_MIN || settings.autoModeSpeed > AUTO_MODE_SPEED_MAX) ? AUTO_MODE_SPEED_DEFAULT : settings.autoModeSpeed;
    fullTravelSteps = (settings.fullTravelSteps < FULL_TRAVEL_STEPS_MIN) ? FULL_TRAVEL_STEPS_DEFAULT : settings.fullTravelSteps;
    stepManualFactor = (settings.stepManualFactor < STEP_MANUAL_FACTOR_MIN || settings.stepManualFactor > STEP_MANUAL_FACTOR_MAX) ? STEP_MANUAL_FACTOR_DEFAULT : settings.stepManualFactor;
    stepManualFactorSW = (settings.stepManualFactorSW < STEP_MANUAL_FACTOR_SW_MIN || settings.stepManualFactorSW > STEP_MANUAL_FACTOR_SW_MAX) ? STEP_MANUAL_FACTOR_SW_DEFAULT : settings.stepManualFactorSW;
    stepperAcceleration = (settings.stepperAcceleration < STEPPER_ACCELERATION_MIN || settings.stepperAcceleration > STEPPER_ACCELERATION_MAX) ? STEPPER_ACCELERATION_DEFAULT : settings.stepperAcceleration;
    buttonHoldInterval = (settings.buttonHoldInterval < BUTTON_HOLD_INTERVAL_MIN || settings.buttonHoldInterval > BUTTON_HOLD_INTERVAL_MAX) ? BUTTON_HOLD_INTERVAL_DEFAULT : settings.buttonHoldInterval;
    dynamicSpeedFactor = (isnan(settings.dynamicSpeedFactor) || settings.dynamicSpeedFactor < DYNAMIC_SPEED_FACTOR_MIN || settings.dynamicSpeedFactor > DYNAMIC_SPEED_FACTOR_MAX) ? DYNAMIC_SPEED_FACTOR_DEFAULT : settings.dynamicSpeedFactor;
  }
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetControllerDirection(pidDirection);
  digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);
}

// ====================================================================
// MARLIN-LIKE HOMING SEQUENCE (G28 equivalent)
// ====================================================================
void runHomingSequence() {
  wdt_reset();
  unsigned long now = millis();
  bool timeout = (now - homingStartTime > HOMING_TIMEOUT_MS);

  switch(homingState) {
    case HOMING_INIT:
      lcd.clear();
      homingState = HOMING_INIT_DELAY;
      stateTimer = now;
      break;

    case HOMING_INIT_DELAY:
      if (now - stateTimer >= 100) {
        lcd.setCursor(0, 0);
        lcd.print(F("HOMING SEQUENCE"));
        lcd.setCursor(0, 1);
        lcd.print(F("Homing left..."));
        homingStartTime = now;
        stepper.setSpeed(-homingSpeed);
        homingState = HOMING_LEFT;
      }
      break;

    case HOMING_LEFT:
      if (timeout) {
        lcd.setCursor(0, 1);
        lcd.print(F("Homing timeout"));
        homingState = HOMING_TIMEOUT_DELAY;
        stateTimer = now;
        break;
      }
      leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
      if (leftLimitState) {
        stepper.stop();
        homingState = HOMING_LEFT_HIT_DELAY1;
        stateTimer = now;
      } else {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        stepper.runSpeed();
      }
      break;

    case HOMING_TIMEOUT_DELAY:
      if (now - stateTimer >= 1000) {
        homingState = HOMING_COMPLETE;
      }
      break;

    case HOMING_LEFT_HIT_DELAY1:
      if (now - stateTimer >= 50) {
        stepper.setCurrentPosition(0);
        currentPosition = 0;
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        lcd.setCursor(0, 1);
        lcd.print(F("Left limit hit"));
        homingState = HOMING_LEFT_HIT_DELAY2;
        stateTimer = now;
      }
      break;

    case HOMING_LEFT_HIT_DELAY2:
      if (now - stateTimer >= 200) {
        if (fullTravelSteps > 0) {
          long centerPos = fullTravelSteps / 2;
          lcd.setCursor(0, 2);
          lcd.print(F("Moving to center"));
          stepper.setMaxSpeed(homingSpeed);
          stepper.setAcceleration(stepperAcceleration);
          stepper.moveTo(centerPos);
          homingState = HOMING_CENTER;
        } else {
          homingState = HOMING_COMPLETE;
        }
      }
      break;

    case HOMING_CENTER:
      if (timeout) {
        lcd.setCursor(0, 2);
        lcd.print(F("Center timeout  "));
        homingState = HOMING_CENTER_TIMEOUT_DELAY;
        stateTimer = now;
        break;
      }
      stepper.run();
      if (stepper.distanceToGo() == 0) {
        currentPosition = stepper.currentPosition();
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        lcd.setCursor(0, 2);
        lcd.print(F("Centered at: "));
        lcd.print(currentPosition);
        lcd.print(F("   "));
        homingState = HOMING_CENTER_DONE_DELAY;
        stateTimer = now;
      } else {
        if (stepper.currentPosition() < currentPosition) {
          digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
          digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        } else {
          digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
          digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        }
      }
      break;

    case HOMING_CENTER_TIMEOUT_DELAY:
      if (now - stateTimer >= 1000) {
        homingState = HOMING_COMPLETE;
      }
      break;

    case HOMING_CENTER_DONE_DELAY:
      if (now - stateTimer >= 500) {
        homingState = HOMING_COMPLETE;
      }
      break;

    case HOMING_COMPLETE:
      homingDone = true;
      lcd.clear();
      homingState = HOMING_COMPLETE_DELAY1;
      stateTimer = now;
      break;

    case HOMING_COMPLETE_DELAY1:
      if (now - stateTimer >= 100) {
        lcd.setCursor(0, 0);
        lcd.print(F("HOMING COMPLETE"));
        homingState = HOMING_COMPLETE_DELAY2;
        stateTimer = now;
      }
      break;

    case HOMING_COMPLETE_DELAY2:
      if (now - stateTimer >= 500) {
        updateMainDisplay();
        homingState = HOMING_IDLE;
      }
      break;

    default:
      break;
  }
}

// ====================================================================
// CENTERING FUNCTION - Marlin G0 to center
// ====================================================================
bool centeringActive = false;

void startCentering() {
  if (fullTravelSteps > 0 && !leftLimitState && !rightLimitState) {
    centeringActive = true;
    long centerPos = fullTravelSteps / 2;
    stepper.setMaxSpeed(homingSpeed);
    stepper.setAcceleration(stepperAcceleration);
    stepper.moveTo(centerPos);
    lcd.setCursor(0, 1);
    lcd.print(F("Moving to center..."));
  }
}

void runCentering() {
  if (!centeringActive) return;

  stepper.run();
  if (stepper.distanceToGo() == 0) {
    currentPosition = stepper.currentPosition();
    centeringActive = false;
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
  } else {
    float speed = stepper.speed();
    updateDirectionLEDsFromVelocity(speed);
  }
}

// ====================================================================
// LED CONTROL
// ====================================================================
void updateDirectionLEDsFromVelocity(float speed) {
  if (abs(speed) < 1.0) {
    digitalWrite(LEFT_LED_PIN, LOW);
    digitalWrite(RIGHT_LED_PIN, LOW);
  } else {
    if (INVERT_LED_LOGIC) {
      digitalWrite(LEFT_LED_PIN, speed < 0 ? HIGH : LOW);
      digitalWrite(RIGHT_LED_PIN, speed > 0 ? HIGH : LOW);
    } else {
      digitalWrite(LEFT_LED_PIN, speed < 0 ? LOW : HIGH);
      digitalWrite(RIGHT_LED_PIN, speed > 0 ? LOW : HIGH);
    }
  }
}

// ====================================================================
// MENU SYSTEM FUNCTIONS
// ====================================================================
void initMenuValues() { 
  for (int i = 0; i < numMenuItems; i++) {
    lastMenuValues[i] = 0x7fffffffffffffffLL; 
  }
}

bool isNumericItem(int idx) {
  switch(idx) {
    case 0: case 1: case 2: case 4: case 5: case 7: case 8: case 9: case 10: 
    case 12: case 13: case 14: case 15: case 16: return true;
    default: return false;
  }
}

bool isToggleItem(int idx) {
  switch(idx) {
    case 3: case 6: return true;
    default: return false;
  }
}

void handleMenuSystem() {
  wdt_reset();
  if (calibState != CALIB_IDLE || homingState != HOMING_IDLE || centeringActive) return;
  static MenuState lastMenuState = NORMAL;
  static long lastEncoderPos = 0;
  long newEncoderPos = myEnc.read();
  int delta = 0;
  if (menuState != NORMAL && menuState != MENU_EXIT_CONFIRM) {
    long encoderDiff = (newEncoderPos - lastEncoderPos) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos = newEncoderPos;
      delta = encoderDiff > 0 ? 1 : -1;
      if (menuState == MENU_BROWSE) {
        currentMenuItem = (currentMenuItem + delta + numMenuItems) % numMenuItems;
        if (currentMenuItem < menuTopItem) {
          menuTopItem = currentMenuItem;
        } else if (currentMenuItem > menuTopItem + 2) {
          menuTopItem = currentMenuItem - 2;
        }
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
        drawMenu();
      } else if (menuState == MENU_EDIT) {
        adjustParameter(delta);
        drawMenu();
      } else if (menuState == MENU_DIGIT_EDIT) {
        if (isdigit(editValueStr[currentDigitPos]) || editValueStr[currentDigitPos] == '_') {
          int digitVal = (editValueStr[currentDigitPos] == '_') ? 0 : (editValueStr[currentDigitPos] - '0');
          digitVal = (digitVal + delta + 10) % 10;
          editValueStr[currentDigitPos] = (digitVal == 0 && isNumericItem(currentMenuItem)) ? '_' : ('0' + digitVal);
        }
        drawMenu();
      }
    }
  } else if (isManualMode && menuState == NORMAL) {
    // Encoder handled in handleManualButtons
  } else if (menuState == MENU_EXIT_CONFIRM) {
    long encoderDiff = (newEncoderPos - lastEncoderPos) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos = newEncoderPos;
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

  bool encBtnState = digitalRead(ENCODER_SW);
  unsigned long now = millis();
  if (encBtnState == LOW && encoderButtonPrevState == HIGH) {
    encoderButtonDownTime = now;
  } else if (encBtnState == HIGH && encoderButtonPrevState == LOW) {
    unsigned long held = now - encoderButtonDownTime;
    if (held >= LONG_PRESS_MS) {
      if (menuState == NORMAL) {
        menuState = MENU_TRANSITION_TO_BROWSE;
        stateTimer = now;
      } else {
        menuState = MENU_TRANSITION_TO_NORMAL;
        stateTimer = now;
      }
    } else if (held >= SHORT_PRESS_MS) {
      if (menuState == MENU_BROWSE) {
        if (currentMenuItem == 11) {
          menuState = MENU_TRANSITION_TO_CALIB;
          stateTimer = now;
        } else if (currentMenuItem == numMenuItems - 1) {
          menuState = MENU_TRANSITION_TO_EXIT_CONFIRM;
          stateTimer = now;
        } else if (isNumericItem(currentMenuItem)) {
          menuState = MENU_DIGIT_EDIT;
          getMenuValueString(currentMenuItem, editValueStr);
          currentDigitPos = 0;
          blinkTimer = millis();
          blinkState = true;
        } else if (isToggleItem(currentMenuItem)) {
          menuState = MENU_EDIT;
        }
      } else if (menuState == MENU_EDIT) {
        menuState = MENU_BROWSE;
        saveSettings();
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
      } else if (menuState == MENU_DIGIT_EDIT) {
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
      } else if (menuState == MENU_EXIT_CONFIRM) {
        if (currentCalibMenuItem == 0) {
          saveSettings();
          menuState = MENU_TRANSITION_TO_NORMAL;
          stateTimer = now;
        } else {
          menuState = MENU_TRANSITION_TO_BROWSE;
          stateTimer = now;
        }
      } else if (menuState == NORMAL) {
        if (fullTravelSteps > 0 && !leftLimitState && !rightLimitState) {
          startCentering();
        }
      }
    }
  }
  encoderButtonPrevState = encBtnState;

  // Handle menu state transitions
  switch (menuState) {
    case MENU_TRANSITION_TO_BROWSE:
      if (now - stateTimer >= 150) {
        menuState = MENU_BROWSE;
        lastMenuTopItem = -1;
        lastCurrentMenuItem = -1;
        for (int i = 0; i < numMenuItems; i++) {
          lastMenuValues[i] = 0x7FFFFFFFFFFFFFFFLL;
        }
        lcd.clear();
        drawMenu();
      }
      break;
    case MENU_TRANSITION_TO_NORMAL:
      if (now - stateTimer >= 150) {
        menuState = NORMAL;
        lcd.clear();
        updateMainDisplay();
        displayLockoutUntil = now + DISPLAY_LOCKOUT_MS;
      }
      break;
    case MENU_TRANSITION_TO_CALIB:
      if (now - stateTimer >= 150) {
        menuState = MENU_CALIB;
        calibState = CALIB_INIT;
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        lcd.clear();
      }
      break;
    case MENU_TRANSITION_TO_EXIT_CONFIRM:
      if (now - stateTimer >= 150) {
        menuState = MENU_EXIT_CONFIRM;
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        lcd.clear();
      }
      break;
    default:
      if (menuState != NORMAL || menuState != lastMenuState) {
        if (menuState == MENU_EXIT_CONFIRM) {
          drawCalibMenu();
        } else if (menuState != NORMAL) {
          drawMenu();
        }
      }
      break;
  }
  lastMenuState = menuState;
}

void drawMenu() {
  wdt_reset();
  static unsigned long lastMenuUpdate = 0;
  if (millis() - lastMenuUpdate < 50) return;
  lastMenuUpdate = millis();
  bool forceRedraw = (lastMenuTopItem == -1) || 
                     (lastCurrentMenuItem != currentMenuItem) || 
                     (lastMenuTopItem != menuTopItem) ||
                     (menuState == MENU_DIGIT_EDIT || 
                      menuState == MENU_EDIT && lastMenuValues[currentMenuItem] != getMenuValueInt(currentMenuItem));
  if (forceRedraw) {
    lcd.setCursor(0, 0);
    lcd.print(F("--- Settings Menu ---"));
  }
  for (int i = 0; i < 3; i++) {
    int idx = menuTopItem + i;
    lcd.setCursor(0, i + 1);
    lcd.print(F("                    "));
    if (idx >= numMenuItems) continue;
    lcd.setCursor(0, i + 1);
    if (idx == currentMenuItem) {
      lcd.print('>');
    } else {
      lcd.print(' ');
    }
    lcd.print(menuItems[idx]);
    int currentPos = strlen(menuItems[idx]) + 1;
    lcd.setCursor(currentPos, i + 1);
    char valueStr[24];
    if (menuState == MENU_DIGIT_EDIT && idx == currentMenuItem) {
      strcpy(valueStr, editValueStr);
    } else {
      getMenuValueString(idx, valueStr);
    }
    int valueLen = strlen(valueStr);
    int valuePos = 20 - valueLen;
    if (valuePos < currentPos) valuePos = currentPos;
    lcd.setCursor(valuePos, i + 1);
    if (menuState == MENU_DIGIT_EDIT && idx == currentMenuItem) {
      for (int k = 0; k < valueLen; k++) {
        if (k == currentDigitPos && !blinkState && (isdigit(valueStr[k]) || valueStr[k] == '_')) {
          lcd.print(' ');
        } else {
          lcd.print(valueStr[k]);
        }
        lcd.setCursor(valuePos + k + 1, i + 1);
      }
    } else {
      lcd.print(valueStr);
    }
    lastMenuValues[idx] = getMenuValueInt(idx);
  }
  lastMenuTopItem = menuTopItem;
  lastCurrentMenuItem = currentMenuItem;
}

void drawCalibMenu() {
  wdt_reset();
  static unsigned long lastCalibUpdate = 0;
  if (millis() - lastCalibUpdate < 100) return;
  lastCalibUpdate = millis();
  bool forceRedraw = (lastCalibMenuItem != currentCalibMenuItem);
  if (forceRedraw) {
    lcd.setCursor(0, 0);
    lcd.print(F("                    "));
    lcd.setCursor(0, 0);
    if (calibState == CALIB_LEFT_CONFIRM) {
      lcd.print(F("Left Limit Reached"));
    } else if (calibState == CALIB_RIGHT_CONFIRM) {
      lcd.print(F("Calib Done: "));
      lcd.print(fullTravelSteps);
    } else if (menuState == MENU_EXIT_CONFIRM) {
      lcd.print(F("Exit Menu?"));
    }
  }
  for (int i = 0; i < 2; i++) {
    lcd.setCursor(0, i + 1);
    lcd.print(F("                    "));
    if (i < numCalibMenuItems) {
      lcd.setCursor(0, i + 1);
      if (i == currentCalibMenuItem) {
        lcd.print('>');
      } else {
        lcd.print(' ');
      }
      lcd.print(calibMenuItems[i]);
    }
  }
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  lcd.setCursor(0, 3);
  lcd.print(F("Press to confirm"));
  lastCalibMenuItem = currentCalibMenuItem;
}

long getMenuValueInt(int idx) {
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
    case 14: return stepperAcceleration;
    case 15: return buttonHoldInterval;
    case 16: return (long)(dynamicSpeedFactor * 100);
    case 17: return 0;
    default: return 0;
  }
}

void adjustParameter(int delta) {
  switch(currentMenuItem) {
    case 3: pidDirection = (pidDirection == DIRECT ? REVERSE : DIRECT); myPID.SetControllerDirection(pidDirection); break;
    case 6: isManualMode = !isManualMode; digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH); break;
  }
}

void parseEditedValue(int idx) {
  char tempStr[24];
  strcpy(tempStr, editValueStr);
  for (int i = 0; tempStr[i]; i++) {
    if (tempStr[i] == '_') tempStr[i] = '0';
  }
  switch(idx) {
    case 0: Kp = constrain(atof(tempStr), KP_MIN, KP_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 1: Ki = constrain(atof(tempStr), KI_MIN, KI_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 2: Kd = constrain(atof(tempStr), KD_MIN, KD_MAX); myPID.SetTunings(Kp, Ki, Kd); break;
    case 4: setPoint = constrain(atof(tempStr), SETPOINT_MIN, SETPOINT_MAX); break;
    case 5: deadband = constrain(atof(tempStr), DEADBAND_MIN, DEADBAND_MAX); break;
    case 7: stepperMinSpeed = constrain(atol(tempStr), STEPPER_MIN_SPEED_MIN, stepperMaxSpeed); break;
    case 8: stepperMaxSpeed = constrain(atol(tempStr), stepperMinSpeed, STEPPER_MAX_SPEED_MAX); break;
    case 9: homingSpeed = constrain(atol(tempStr), HOMING_SPEED_MIN, HOMING_SPEED_MAX); break;
    case 10: autoModeSpeed = constrain(atol(tempStr), AUTO_MODE_SPEED_MIN, AUTO_MODE_SPEED_MAX); break;
    case 12: stepManualFactor = constrain(atol(tempStr), STEP_MANUAL_FACTOR_MIN, STEP_MANUAL_FACTOR_MAX); break;
    case 13: stepManualFactorSW = constrain(atol(tempStr), STEP_MANUAL_FACTOR_SW_MIN, STEP_MANUAL_FACTOR_SW_MAX); break;
    case 14: stepperAcceleration = constrain(atol(tempStr), STEPPER_ACCELERATION_MIN, STEPPER_ACCELERATION_MAX); break;
    case 15: buttonHoldInterval = constrain(atol(tempStr), BUTTON_HOLD_INTERVAL_MIN, BUTTON_HOLD_INTERVAL_MAX); break;
    case 16: dynamicSpeedFactor = constrain(atof(tempStr) / 100.0, DYNAMIC_SPEED_FACTOR_MIN, DYNAMIC_SPEED_FACTOR_MAX); break;
  }
}

// ====================================================================
// CALIBRATION - Position-based for precision
// ====================================================================
void runCalibration() {
  unsigned long currentMillis = millis();
  bool encBtnState = digitalRead(ENCODER_SW);
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);
  int delta = 0;
  static long lastEncoderPos = 0;
  long newEncoderPos = myEnc.read();
  if (calibState != CALIB_IDLE) {
    long encoderDiff = (newEncoderPos - lastEncoderPos) / 4;
    if (encoderDiff != 0) {
      lastEncoderPos = newEncoderPos;
      delta = encoderDiff > 0 ? 1 : -1;
    }
  }

  unsigned long dt = currentMillis - lastUpdateTime;
  lastUpdateTime = currentMillis;

  switch(calibState) {
    case CALIB_INIT:
      wdt_disable();
      lcd.clear();
      stateTimer = currentMillis;
      calibState = CALIB_INIT_DELAY;
      break;

    case CALIB_INIT_DELAY:
      if (currentMillis - stateTimer >= 150) {
        lcd.setCursor(0, 0);
        lcd.print(F("Calibrating..."));
        calibTimer = currentMillis;
        stepper.setSpeed(-calibSpeed);
        calibState = CALIB_MOVE_LEFT;
      }
      break;

    case CALIB_MOVE_LEFT:
      wdt_reset();
      if (!leftLimitState) {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        lcd.setCursor(0, 1);
        lcd.print(F("Moving Left..."));
        stepper.runSpeed();
      } else {
        stepper.stop();
        stepper.setCurrentPosition(0);
        currentPosition = 0;
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Left Limit Reached"));
        currentCalibMenuItem = 0;
        lastCalibMenuItem = -1;
        calibTimer = currentMillis;
        stateTimer = currentMillis;
        calibState = CALIB_LEFT_CONFIRM;
      }
      break;

    case CALIB_LEFT_CONFIRM:
      wdt_reset();
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
        calibState = CALIB_TIMEOUT_DELAY;
        break;
      }
      if (delta != 0) {
        currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
        lastCalibMenuItem = -1;
      }
      if (encBtnState == LOW && encoderButtonPrevState == HIGH) {
        encoderButtonDownTime = currentMillis;
      } else if (encBtnState == HIGH && encoderButtonPrevState == LOW) {
        unsigned long held = currentMillis - encoderButtonDownTime;
        if (held >= SHORT_PRESS_MS && held < LONG_PRESS_MS) {
          if (currentCalibMenuItem == 0) {
            calibTimer = currentMillis;
            stepper.setSpeed(calibSpeed);
            digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
            digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print(F("Calibrating..."));
            calibState = CALIB_MOVE_RIGHT;
          } else {
            calibState = CALIB_CANCEL_DELAY;
          }
        } else if (held >= LONG_PRESS_MS) {
          calibState = CALIB_CANCEL_DELAY;
        }
      }
      encoderButtonPrevState = encBtnState;
      drawCalibMenu();
      break;

    case CALIB_MOVE_RIGHT:
      wdt_reset();
      if (!rightLimitState) {
        digitalWrite(LEFT_LED_PIN, INVERT_LED_LOGIC ? HIGH : LOW);
        digitalWrite(RIGHT_LED_PIN, INVERT_LED_LOGIC ? LOW : HIGH);
        lcd.setCursor(0, 1);
        lcd.print(F("Steps: "));
        lcd.print(stepper.currentPosition());
        lcd.print(F("    "));
        stepper.runSpeed();
      } else {
        stepper.stop();
        fullTravelSteps = stepper.currentPosition();
        digitalWrite(LEFT_LED_PIN, LOW);
        digitalWrite(RIGHT_LED_PIN, LOW);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calib Done: "));
        lcd.print(fullTravelSteps);
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
      if (currentMillis - calibTimer >= CALIB_TIMEOUT_MS) {
        calibState = CALIB_TIMEOUT_DELAY;
        break;
      }
      if (delta != 0) {
        currentCalibMenuItem = (currentCalibMenuItem + delta + numCalibMenuItems) % numCalibMenuItems;
        lastCalibMenuItem = -1;
      }
      if (encBtnState == LOW && encoderButtonPrevState == HIGH) {
        encoderButtonDownTime = currentMillis;
      } else if (encBtnState == HIGH && encoderButtonPrevState == LOW) {
        unsigned long held = currentMillis - encoderButtonDownTime;
        if (held >= SHORT_PRESS_MS && held < LONG_PRESS_MS) {
          if (currentCalibMenuItem == 0) {
            saveSettings();
            startCentering();
            calibState = CALIB_SAVE_DELAY1;
            stateTimer = currentMillis;
          } else {
            calibState = CALIB_CANCEL_DELAY;
          }
        } else if (held >= LONG_PRESS_MS) {
          calibState = CALIB_CANCEL_DELAY;
        }
      }
      encoderButtonPrevState = encBtnState;
      drawCalibMenu();
      break;

    case CALIB_TIMEOUT_DELAY:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Calib timed out"));
      wdt_enable(WDTO_4S);
      updateMainDisplay();
      displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
      calibState = CALIB_IDLE;
      menuState = NORMAL;
      break;

    case CALIB_CANCEL_DELAY:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Calib cancelled"));
      wdt_enable(WDTO_4S);
      updateMainDisplay();
      displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
      calibState = CALIB_IDLE;
      menuState = NORMAL;
      break;

    case CALIB_SAVE_DELAY1:
      if (currentMillis - stateTimer >= 150) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Calib saved"));
        lcd.setCursor(0, 1);
        lcd.print(F("Moving to center..."));
        calibState = CALIB_SAVE_DELAY2;
        stateTimer = currentMillis;
      }
      break;

    case CALIB_SAVE_DELAY2:
      if (currentMillis - stateTimer >= 1000) {
        wdt_enable(WDTO_4S);
        updateMainDisplay();
        displayLockoutUntil = currentMillis + DISPLAY_LOCKOUT_MS;
        calibState = CALIB_IDLE;
        menuState = NORMAL;
      }
      break;

    case CALIB_IDLE:
    default:
      digitalWrite(LEFT_LED_PIN, LOW);
      digitalWrite(RIGHT_LED_PIN, LOW);
      break;
  }
}

// ====================================================================
// DISPLAY FUNCTIONS
// ====================================================================
void updateMainDisplay() {
  lcd.clear();
  for (int row = 0; row < 4; row++) {
    lcd.setCursor(0, row);
    lcd.print(F("                    "));
  }
  lcd.setCursor(0, 0);
  lcd.print(F("Senzor: "));
  char voltageStr[6];
  dtostrf(input, 4, 2, voltageStr);
  lcd.print(voltageStr);
  lcd.print(F("V             "));
  lcd.setCursor(0, 2);
  lcd.print(F("POZITIE: "));
  char posStr[12];
  sprintf(posStr, "%ld", currentPosition);
  lcd.print(posStr);
  lcd.print(F("            "));
  bool isLeft = (setPoint - input) > 0 && !isManualMode;
  displayPositionGraph(isLeft);
  lastDisplayedPosition = currentPosition;
  lastPositionUpdate = millis();
}

void getMenuValueString(int idx, char* buffer) {
  char temp[24];
  switch(idx) {
    case 0:
      dtostrf(Kp, 7, 2, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 1:
      dtostrf(Ki, 7, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 2:
      dtostrf(Kd, 7, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 3: strcpy(buffer, pidDirection == DIRECT ? "DIR" : "REV"); break;
    case 4:
      dtostrf(setPoint, 4, 2, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 5:
      dtostrf(deadband, 4, 3, temp);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 6: strcpy(buffer, isManualMode ? "MAN" : "AUTO"); break;
    case 7:
      sprintf(temp, "%5d", stepperMinSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 8:
      sprintf(temp, "%5d", stepperMaxSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 9:
      sprintf(temp, "%4d", homingSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 10:
      sprintf(temp, "%5d", autoModeSpeed);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 11: sprintf(buffer, "%ld", fullTravelSteps); break;
    case 12:
      sprintf(temp, "%3d", stepManualFactor);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 13:
      sprintf(temp, "%3d", stepManualFactorSW);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 14:
      sprintf(temp, "%4d", stepperAcceleration);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 15:
      sprintf(temp, "%4d", buttonHoldInterval);
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 16:
      sprintf(temp, "%3d", (int)(dynamicSpeedFactor * 100));
      for (int i = 0; temp[i] == ' '; i++) temp[i] = '_';
      strcpy(buffer, temp);
      break;
    case 17: strcpy(buffer, ""); break;
    default: strcpy(buffer, ""); break;
  }
}

void displayPositionGraph(bool isLeft) {
  if (fullTravelSteps <= 0) {
    lcd.setCursor(0, 3);
    lcd.print(F("                    "));
    return;
  }
  if (leftLimitState) {
    lcd.setCursor(0, 3);
    lcd.print(F("<LIMITA STANGA     "));
    return;
  } 
  if (rightLimitState) {
    lcd.setCursor(0, 3);
    lcd.print(F(">LIMITA DREAPTA    "));
    return;
  }
  lcd.setCursor(0, 3);
  lcd.print(F("                    "));
  for (int i = 0; i < 20; i++) {
    lcd.setCursor(i, 3);
    lcd.write((uint8_t)0);
  }
  lcd.setCursor(9, 3);
  lcd.write((uint8_t)6);
  lcd.setCursor(10, 3);
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
      for (int i = 0; i < 8; i++) {
        lcd.createChar(i, barChars[i]);
      }
    } else {
      for (int i = 0; i < 8; i++) {
        lcd.createChar(i, barCharsMirrored[i]);
      }
    }
    lastDirection = isLeftOfCenter;
    lcd.setCursor(9, 3);
    lcd.write((uint8_t)6);
    lcd.setCursor(10, 3);
    lcd.write((uint8_t)7);
  }
  for (int i = 0; i < 10; i++) {
    int lcdPos;
    if (isLeftOfCenter) {
      lcdPos = 9 - i;
    } else {
      lcdPos = 10 + i;
    }
    lcd.setCursor(lcdPos, 3);
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
// SETUP
// ====================================================================
void setup() {
  // This function is intentionally left empty.
  // Initialization is handled by real_setup().
}

void real_setup() {
  pinMode(LEFT_LIMIT_PIN, INPUT);
  pinMode(RIGHT_LIMIT_PIN, INPUT);
  pinMode(DIRECTION_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENCODER_SW, INPUT_PULLUP);
  pinMode(TOGGLE_BUTTON_PIN, INPUT);
  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MODE_LED_PIN, OUTPUT);
  pinMode(LEFT_LED_PIN, OUTPUT);
  pinMode(RIGHT_LED_PIN, OUTPUT);
  digitalWrite(LEFT_LED_PIN, LOW);
  digitalWrite(RIGHT_LED_PIN, LOW);
  lcd.begin(20, 4);
  createBarGraphChars();

  // The rest of the setup is handled in the setup_loop() state machine
}

void setup_loop() {
  static int setup_step = 0;
  static unsigned long welcome_timer = 0;
  unsigned long currentMillis = millis();

  switch(setup_step) {
    case 0: // Start LCD and show welcome message
      lcd.setCursor(0, 0);
      lcd.print(F(" WEB GUIDE CONTROL"));
      lcd.setCursor(0, 1);
      lcd.print(F("  FIRMWARE - V2.9.22"));
      lcd.setCursor(0, 2);
      lcd.print(F("Designe by --------"));
      lcd.setCursor(0, 3);
      lcd.print(F(" for ------- S.R.L."));
      welcome_timer = currentMillis;
      setup_step++;
      break;

    case 1: // Wait for 2 seconds
      if (currentMillis - welcome_timer >= 2000) {
        lcd.clear();
        welcome_timer = currentMillis;
        setup_step++;
      }
      break;

    case 2: // Wait for 150ms
      if (currentMillis - welcome_timer >= 150) {
        loadSettings();
        stepper.setMaxSpeed(stepperMaxSpeed);
        stepper.setAcceleration(stepperAcceleration);
        myPID.SetMode(AUTOMATIC);
        myPID.SetSampleTime(20);
        initMenuValues();
        lastEncoderPos = myEnc.read();
        lastUpdateTime = millis();
        wdt_enable(WDTO_4S);
        if (fullTravelSteps > 0) {
          homingState = HOMING_INIT;
          homingDone = false;
        } else {
          homingDone = true;
          updateMainDisplay();
        }
        systemState = STATE_RUNNING; // Transition to the main loop
      }
      break;
  }
}

// ====================================================================
// MAIN LOOP - Mixed velocity and position modes
// ====================================================================
void main_loop() {
  wdt_reset();
  unsigned long currentMillis = millis();

  // Update limit switches
  leftLimitState = updateLimitSwitch(LEFT_LIMIT_PIN, leftLimitDebounceTime, leftLimitState);
  rightLimitState = updateLimitSwitch(RIGHT_LIMIT_PIN, rightLimitDebounceTime, rightLimitState);

  // Homing
  if (!homingDone && homingState != HOMING_IDLE) {
    runHomingSequence();
    return;
  }

  // Calibration
  if (calibState != CALIB_IDLE) {
    runCalibration();
    return;
  }

  // Centering
  if (centeringActive) {
    runCentering();
    return;  // Prioritize centering
  }

  // Menu & buttons
  handleMenuSystem();
  handleToggleButton();
  if (isManualMode) {
    handleManualButtons();
  }

  if (menuState == NORMAL && calibState == CALIB_IDLE && !centeringActive) {
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      input = readVoltage();
      lcd.setCursor(0, 0);
      lcd.print(F("Senzor: "));
      lcd.print(input, 2);
      lcd.print(F("V   "));

      if (!isManualMode) {
        // AUTO MODE: Velocity PID
        double error = setPoint - input;
        bool inDeadband = abs(error) < deadband;

        myPID.Compute(); // Always compute for state updates

        if (inDeadband) {
          output = 0.0;
        }

        // Apply direction
        if (error < 0) output = -abs(output);
        else if (error > 0) output = abs(output);
        else output = 0;

        // Adaptive speed limits
        int maxVel = autoModeSpeed;
        if (abs(error) > LARGE_ERROR_THRESHOLD) {
          maxVel = autoModeSpeed * FAST_SPEED_MULTIPLIER;
        } else if (abs(error) > MEDIUM_ERROR_THRESHOLD) {
          maxVel = autoModeSpeed * MEDIUM_SPEED_MULTIPLIER;
        }
        maxVel = constrain(maxVel, stepperMinSpeed, STEPPER_MAX_SPEED_MAX);
        output = constrain(output, -maxVel, maxVel);

        // Enforce limits
        if (leftLimitState && output < 0) output = 0;
        if (rightLimitState && output > 0) output = 0;

        // Update position (velocity integration)
        unsigned long dt = currentMillis - lastUpdateTime;
        lastUpdateTime = currentMillis;
        if (dt > 0) {
          currentPosition += (long)((output * dt) / 1000.0);
        }
        currentPosition = constrain(currentPosition, 0L, fullTravelSteps);
        stepper.setCurrentPosition(currentPosition);  // Sync with hardware tracker

        // Drive motor in velocity mode
        stepper.setSpeed((float)output);
        stepper.runSpeed();
        updateDirectionLEDsFromVelocity((float)output);

        // Display
        lcd.setCursor(0, 1);
        bool isLeft = error > 0;
        lcd.print(isLeft ? F("STANGA  << ") : F("DREAPTA >> "));
        lcd.print(abs(error), 2);
        lcd.print(F("V  "));

        if (currentMillis - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
          if (currentPosition != lastDisplayedPosition) {
            lcd.setCursor(0, 2);
            lcd.print(F("POZITIE: "));
            lcd.print(currentPosition);
            lcd.print(F("    "));
            lastDisplayedPosition = currentPosition;
          }
          lastPositionUpdate = currentMillis;
        }
        displayPositionGraph(isLeft);

      } else {
        // MANUAL MODE: Velocity from buttons/encoder
        stepper.runSpeed();
        lcd.setCursor(0, 1);
        lcd.print(F("CONTROL MANUAL      "));
        if (currentMillis - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
          if (currentPosition != lastDisplayedPosition) {
            lcd.setCursor(0, 2);
            lcd.print(F("POZITIE: "));
            lcd.print(currentPosition);
            lcd.print(F("    "));
            lastDisplayedPosition = currentPosition;
          }
          lastPositionUpdate = currentMillis;
        }
        displayPositionGraph(false);
      }
    }
  }
}

// ====================================================================
// SENSOR FUNCTION
// ====================================================================
float readVoltage() {
    long sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += analogRead(A0);
    }
    return (sum / 10.0) * (5.0 / 1023.0);
}

// ====================================================================
// BUTTON HANDLING
// ====================================================================
void handleToggleButton() {
  bool toggleBtnState = digitalRead(TOGGLE_BUTTON_PIN);
  unsigned long now = millis();
  if (toggleBtnState == HIGH && toggleButtonPrevState == LOW) {
    toggleButtonDownTime = now;
  } else if (toggleBtnState == LOW && toggleButtonPrevState == HIGH) {
    if (now - toggleButtonDownTime >= DEBOUNCE_MS) {
      isManualMode = !isManualMode;
      digitalWrite(MODE_LED_PIN, isManualMode ? LOW : HIGH);
      if (!isManualMode && calibState == CALIB_IDLE) {
        myPID.SetMode(AUTOMATIC);
      }
      saveSettings();
      if (menuState == NORMAL && calibState == CALIB_IDLE) {
        lcd.setCursor(0, 1);
        lcd.print(isManualMode ? F("CONTROL MANUAL      ") : F("AUTO MODE          "));
      }
    }
  }
  toggleButtonPrevState = toggleBtnState;
}

bool readStableButtonState(int pin) {
    const int samples = 5;
    int lowCount = 0;
    for (int i = 0; i < samples; i++) {
        if (digitalRead(pin) == LOW) lowCount++;
    }
    return lowCount >= (samples * 0.8);
}

void handleManualButtons() {
  if (!isManualMode || menuState != NORMAL || calibState != CALIB_IDLE || centeringActive) {
    return;
  }

  unsigned long now = millis();
  bool leftBtnState = readStableButtonState(LEFT_BUTTON_PIN);
  bool rightBtnState = readStableButtonState(RIGHT_BUTTON_PIN);

  float speed = 0.0;

  // Encoder velocity
  static long prevEncPos = 0;
  static unsigned long lastEncTime = 0;
  long currEnc = myEnc.read();
  unsigned long dt = now - lastEncTime;
  if (dt > 0) {
    long rawDelta = currEnc - prevEncPos;
    long ticks = rawDelta / 4;
    double rate = (double)ticks * (1000.0 / dt);
    double encSpeed = rate * stepManualFactor;
    speed = encSpeed * dynamicSpeedFactor;
    speed = constrain(speed, -stepperMaxSpeed, stepperMaxSpeed);
    prevEncPos = currEnc;
    lastEncTime = now;
  }

  // Button overrides
  if (leftBtnState && !rightBtnState && !leftLimitState) {
    speed = -stepperMaxSpeed * dynamicSpeedFactor;
  } else if (rightBtnState && !leftBtnState && !rightLimitState) {
    speed = stepperMaxSpeed * dynamicSpeedFactor;
  }

  // Enforce limits
  if (leftLimitState && speed < 0) speed = 0;
  if (rightLimitState && speed > 0) speed = 0;

  // Update position
  unsigned long dt_pos = now - lastUpdateTime;
  lastUpdateTime = now;
  if (dt_pos > 0 && abs(speed) >= 1) {
    currentPosition += (long)((speed * dt_pos) / 1000.0);
    currentPosition = constrain(currentPosition, 0L, fullTravelSteps);
    stepper.setCurrentPosition(currentPosition);
  }

  // Drive
  stepper.setSpeed(speed);
  stepper.runSpeed();
  updateDirectionLEDsFromVelocity(speed);
}