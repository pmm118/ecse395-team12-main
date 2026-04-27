/*
#include <Arduino.h>

/*
Cat Litter Box Prototype
ESP32 + PIR Motion Sensor + TT Motor (L9110S)

Logic:
1. Detect cat motion using PIR
2. If no motion for QUIET_TIME_MS → start cleaning
3. Motor runs CW ~10 turns
4. If still no motion → run CCW ~10 turns
5. If motion detected anytime → stop motor
6. After one cycle → wait for next cat activity


//
// Pin configuration
//
static const int PIN_MOTOR_IN1 = A0;   // motor driver IN1
static const int PIN_MOTOR_IN2 = A1;   // motor driver IN2
static const int PIN_PIR_OUT   = A2;   // PIR sensor output
static const int PIN_LED       = LED_BUILTIN;

//
// PWM configuration
//
static const int CH1 = 0;
static const int CH2 = 1;
static const int PWM_FREQ_HZ = 20000;
static const int PWM_RES_BITS = 8;
static const uint8_t MOTOR_DUTY = 220;

//
// Timing parameters
//
static const uint32_t QUIET_TIME_MS = 10UL * 1000UL; // wait 10s with no motion
static const uint32_t RUN_10_TURNS_MS = 2300;        // approx time for 10 motor turns

//
// PIR settings
//
static bool PIR_ACTIVE_HIGH = true; // PIR HIGH = motion

//
// System states
//
enum class SysState {
  WAIT_FOR_CAT_ACTIVITY,
  WAIT_FOR_QUIET,
  CLEANING_CW,
  CHECK_BEFORE_CCW,
  CLEANING_CCW,
  WAIT_NEXT_CAT_ACTIVITY
};

static SysState state = SysState::WAIT_FOR_CAT_ACTIVITY;

static uint32_t t_state_enter = 0;
static uint32_t t_last_motion = 0;
static uint32_t t_motor_start = 0;

//
// Return state name (for serial print)
//
const char* stateName(SysState s) {
  switch (s) {
    case SysState::WAIT_FOR_CAT_ACTIVITY: return "WAIT_FOR_CAT_ACTIVITY";
    case SysState::WAIT_FOR_QUIET: return "WAIT_FOR_QUIET";
    case SysState::CLEANING_CW: return "CLEANING_CW";
    case SysState::CHECK_BEFORE_CCW: return "CHECK_BEFORE_CCW";
    case SysState::CLEANING_CCW: return "CLEANING_CCW";
    case SysState::WAIT_NEXT_CAT_ACTIVITY: return "WAIT_NEXT_CAT_ACTIVITY";
    default: return "UNKNOWN";
  }
}

//
// Change state and print to serial
//
void setState(SysState s) {
  state = s;
  t_state_enter = millis();
  Serial.print("[STATE] -> ");
  Serial.println(stateName(s));
}

//
// Read PIR motion sensor
//
bool motionDetected() {
  int v = digitalRead(PIN_PIR_OUT);
  return PIR_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
}

//
// Stop motor
//
void motorStop() {
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
}

//
// Motor clockwise
//
void motorCW(uint8_t duty) {
  ledcWrite(CH2, 0);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  ledcWrite(CH1, duty);
}

//
// Motor counter-clockwise
//
void motorCCW(uint8_t duty) {
  ledcWrite(CH1, 0);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  ledcWrite(CH2, duty);
}

//
// Print system status
//
void printStatus() {
  Serial.print("State=");
  Serial.print(stateName(state));
  Serial.print(" | motion=");
  Serial.print(motionDetected() ? "YES" : "NO");
  Serial.print(" | ms_since_last_motion=");
  Serial.println(millis() - t_last_motion);
}

//
// Setup
//
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Cat Litter Box System");

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PIR_OUT, INPUT);
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);

  ledcSetup(CH1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH2, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_MOTOR_IN1, CH1);
  ledcAttachPin(PIN_MOTOR_IN2, CH2);

  motorStop();

  t_last_motion = millis();
  setState(SysState::WAIT_FOR_CAT_ACTIVITY);
  printStatus();
}

//
// Main loop
//
void loop() {
  uint32_t now = millis();
  bool motion = motionDetected();

  // update last motion time
  if (motion) {
    t_last_motion = now;
  }

  digitalWrite(PIN_LED, motion ? HIGH : LOW);

  switch (state) {

    // wait for first cat activity
    case SysState::WAIT_FOR_CAT_ACTIVITY:
      if (motion) {
        Serial.println("Cat activity detected");
        setState(SysState::WAIT_FOR_QUIET);
      }
      break;

    // wait until quiet period
    case SysState::WAIT_FOR_QUIET:
      if ((now - t_last_motion) >= QUIET_TIME_MS) {
        Serial.println("No motion → start CW cleaning");
        t_motor_start = now;
        motorCW(MOTOR_DUTY);
        setState(SysState::CLEANING_CW);
      }
      break;

    // clockwise cleaning
    case SysState::CLEANING_CW:
      if (motion) {
        Serial.println("Motion detected → STOP");
        motorStop();
        setState(SysState::WAIT_FOR_QUIET);
        break;
      }

      if ((now - t_motor_start) >= RUN_10_TURNS_MS) {
        motorStop();
        Serial.println("CW finished");
        setState(SysState::CHECK_BEFORE_CCW);
      }
      break;

    // check motion before reverse
    case SysState::CHECK_BEFORE_CCW:
      if (motion) {
        Serial.println("Motion detected → wait again");
        setState(SysState::WAIT_FOR_QUIET);
      } else {
        Serial.println("Start CCW cleaning");
        t_motor_start = now;
        motorCCW(MOTOR_DUTY);
        setState(SysState::CLEANING_CCW);
      }
      break;

    // reverse cleaning
    case SysState::CLEANING_CCW:
      if (motion) {
        Serial.println("Motion detected → STOP");
        motorStop();
        setState(SysState::WAIT_FOR_QUIET);
        break;
      }

      if ((now - t_motor_start) >= RUN_10_TURNS_MS) {
        motorStop();
        Serial.println("Cleaning cycle complete");
        setState(SysState::WAIT_NEXT_CAT_ACTIVITY);
      }
      break;

    // wait for next cat visit
    case SysState::WAIT_NEXT_CAT_ACTIVITY:
      if (motion) {
        Serial.println("New cat activity");
        setState(SysState::WAIT_FOR_QUIET);
      }
      break;

    default:
      motorStop();
      break;
  }

  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 1000) {
    lastPrint = now;
    printStatus();
  }

  delay(10);
}

*/