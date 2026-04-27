#if __has_include(<Arduino.h>)
#include <Arduino.h>
#else
// Minimal Arduino-compatible stubs for hosts/IDEs where Arduino.h is unavailable
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define LED_BUILTIN 13
#define A0 0
#define A1 1
#define A2 2

inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return LOW; }
inline unsigned long millis() { return 0UL; }
inline void delay(unsigned long) {}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int, int) {}

struct SerialClass {
  void begin(int) {}
  int available() { return 0; }
  int read() { return -1; }

  void print(const char*) {}
  void println(const char*) {}

  void print(char c) { (void)c; }
  void println(char c) { (void)c; }

  void print(int v) { (void)v; }
  void println(int v) { (void)v; }

  void print(unsigned int v) { (void)v; }
  void println(unsigned int v) { (void)v; }

  void print(unsigned long v) { (void)v; }
  void println(unsigned long v) { (void)v; }

  void print(bool b) { (void)b; }
  void println(bool b) { (void)b; }
} Serial;
#endif

/*
  Cat Litterbox Prototype - Motor Subsystem (TT Motor + L9110S + IR Obstacle Sensor)
  Board: Adafruit Feather ESP32 V2
  Framework: Arduino (PlatformIO)

  ---------- Hardware wiring ----------
  Motor driver (L9110S, Motor-B):
  - ESP32 A0 -> B1A
  - ESP32 A1 -> B2A (or the other Motor-B input pin on your board)
  - TT motor -> MOTOR B terminals
  - Driver VCC -> VBUS (USB 5V recommended)
  - Driver GND -> ESP32 GND (common ground REQUIRED)

  IR Obstacle Module (SunFounder):
  - VCC -> ESP32 3V (3.3V)
  - GND -> ESP32 GND
  - OUT -> ESP32 A2

  ---------- Prototype logic ----------
  We do NOT clean while the cat is present.
  We clean AFTER the cat leaves (IR no longer detects obstacle),
  plus a short delay (leave-confirm delay), then run one cleaning cycle.
  A cooldown prevents repeated cleaning too frequently.

  ---------- Manual controls ----------
  Serial commands:
  - h : help menu
  - p : print status
  - c : force cleaning now
  - s : emergency stop (locks in STOPPED state)
  - r : reset from STOPPED back to IDLE
  - d : toggle trigger edge (detect vs leave)
  - l : toggle active logic (active-low vs active-high)
*/

//
// -------------------- Pin Mapping --------------------
//
static const int PIN_IN1 = A0;      // L9110S Motor-B input 1
static const int PIN_IN2 = A1;      // L9110S Motor-B input 2
static const int PIN_IR_OUT = A2;   // IR obstacle module digital output
static const int PIN_LED = LED_BUILTIN;

//
// -------------------- PWM (LEDC) Setup --------------------
//
static const int CH1 = 0;
static const int CH2 = 1;
static const int PWM_RES_BITS = 8;       // 0..255
static const int PWM_FREQ_HZ = 20000;    // quiet PWM

//
// -------------------- Motor Parameters --------------------
//
static const uint8_t DUTY_MAX = 220;             // adjust for torque vs noise (0..255)
static const uint8_t RAMP_STEP = 5;              // duty increment/decrement per step
static const uint32_t RAMP_STEP_DELAY_MS = 8;    // ramp smoothness

//
// -------------------- Timing Parameters --------------------
//
static const uint32_t CLEAN_RUN_MS = 3500;                 // cleaning action duration
static const uint32_t POST_STOP_MS = 800;                  // pause after motor stops
static const uint32_t COOLDOWN_MS = 3UL * 60UL * 1000UL;   // 3 minutes
static const uint32_t LEAVE_CONFIRM_MS = 1500;             // wait after "cat leaves" before cleaning
static const uint32_t DEBOUNCE_MS = 80;                    // debounce for IR transitions

//
// -------------------- IR Logic Parameters --------------------
//
// Many IR obstacle modules are ACTIVE-LOW when an obstacle is detected (OUT=LOW).
// If your module is opposite, you can toggle at runtime via serial command 'l'.
//
static bool IR_ACTIVE_LOW = true;

// Trigger policy:
// If TRIGGER_ON_DETECT = false: event fires when obstacle is no longer detected (cat leaves) -> recommended.
// If true: event fires when obstacle is detected (cat arrives).
static bool TRIGGER_ON_DETECT = false;

//
// -------------------- State Machine --------------------
//
enum class SysState {
  IDLE,             // waiting for cat presence/leave event
  CAT_PRESENT,      // obstacle currently detected
  WAIT_LEAVE_DELAY, // cat left, waiting LEAVE_CONFIRM_MS before cleaning
  CLEANING,         // run motor cycle
  COOLDOWN,         // cooldown after cleaning
  STOPPED_MANUAL    // manual emergency stop (latched)
};

static SysState state = SysState::IDLE;

static uint32_t t_state_enter = 0;
static uint32_t t_last_clean = 0;
static uint32_t t_last_edge_ms = 0;

//
// -------------------- Serial Monitoring --------------------
//
static uint32_t g_lastAliveMs = 0;
static bool g_lastObstacle = false;
static bool g_hasLastObstacle = false;

static void logStateChange(const char* nextState) {
  Serial.print("[STATE] -> ");
  Serial.println(nextState);
}

static void logMotorAction(const char* msg) {
  Serial.print("[MOTOR] ");
  Serial.println(msg);
}

static void heartbeat() {
  uint32_t now = millis();
  if (now - g_lastAliveMs >= 1000) {
    g_lastAliveMs = now;
    unsigned long b = now / 1000UL;
    Serial.print("Alive:");
    Serial.print(b);
    Serial.println(' ');
  }
}

static void logIRChangeIfAny(bool obstacleNow) {
  if (!g_hasLastObstacle) {
    g_hasLastObstacle = true;
    g_lastObstacle = obstacleNow;
    Serial.print("[IR] init -> ");
    Serial.println(obstacleNow ? "BLOCKED" : "CLEAR");
    return;
  }

  if (obstacleNow != g_lastObstacle) {
    g_lastObstacle = obstacleNow;
    Serial.print("[IR] change -> ");
    Serial.println(obstacleNow ? "BLOCKED" : "CLEAR");
  }
}

//
// -------------------- Motor Helper Functions --------------------
//
static void motorStopHard() {
  ledcWrite(CH1, 0);
  ledcWrite(CH2, 0);
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
}

static void motorForwardDuty(uint8_t duty) {
  // Forward: IN1 PWM, IN2 LOW
  ledcWrite(CH2, 0);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(CH1, duty);
}

static void motorReverseDuty(uint8_t duty) {
  // Reverse: IN1 LOW, IN2 PWM
  ledcWrite(CH1, 0);
  digitalWrite(PIN_IN1, LOW);
  ledcWrite(CH2, duty);
}

static void motorRampForward(uint8_t dutyTarget) {
  logMotorAction("Soft-start (ramp up)");
  for (uint16_t d = 0; d <= dutyTarget; d += RAMP_STEP) {
    motorForwardDuty((uint8_t)d);
    delay(RAMP_STEP_DELAY_MS);
  }
  logMotorAction("Ramp up complete");
}

static void motorRampStopFrom(uint8_t dutyStart) {
  logMotorAction("Soft-stop (ramp down)");
  for (int d = dutyStart; d >= 0; d -= (int)RAMP_STEP) {
    motorForwardDuty((uint8_t)d);
    delay(RAMP_STEP_DELAY_MS);
  }
  motorStopHard();
  logMotorAction("Motor hard stop issued");
}

static void setState(SysState s) {
  state = s;
  t_state_enter = millis();

  logStateChange(
    (s == SysState::IDLE) ? "IDLE" :
    (s == SysState::CAT_PRESENT) ? "CAT_PRESENT" :
    (s == SysState::WAIT_LEAVE_DELAY) ? "WAIT_LEAVE_DELAY" :
    (s == SysState::CLEANING) ? "CLEANING" :
    (s == SysState::COOLDOWN) ? "COOLDOWN" :
    (s == SysState::STOPPED_MANUAL) ? "STOPPED_MANUAL" : "UNKNOWN"
  );
}

//
// -------------------- IR Input Helpers --------------------
//
static bool irObstacleDetectedRaw() {
  int v = digitalRead(PIN_IR_OUT);
  return IR_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

static bool irEdgeEventDebounced(bool obstacleNow) {
  static bool lastObstacle = false;
  uint32_t now = millis();

  if ((now - t_last_edge_ms) < DEBOUNCE_MS) {
    lastObstacle = obstacleNow;
    return false;
  }

  bool event = false;

  if (TRIGGER_ON_DETECT) {
    if (obstacleNow && !lastObstacle) event = true;
  } else {
    if (!obstacleNow && lastObstacle) event = true;
  }

  if (event) {
    t_last_edge_ms = now;
  }

  lastObstacle = obstacleNow;
  return event;
}

//
// -------------------- Serial UI --------------------
//
static void printHelp() {
  Serial.println(' ');
  Serial.println("=== Cat Litterbox Motor Prototype (IR + L9110S) ===");
  Serial.println("h : help");
  Serial.println("p : print status");
  Serial.println("c : force cleaning now");
  Serial.println("s : emergency stop (latched)");
  Serial.println("r : reset from STOPPED back to IDLE");
  Serial.println("d : toggle trigger edge (detect <-> leave)");
  Serial.println("l : toggle IR active logic (active-low <-> active-high)");
  Serial.println("===================================================");
}

static const char* stateName(SysState s) {
  switch (s) {
    case SysState::IDLE: return "IDLE";
    case SysState::CAT_PRESENT: return "CAT_PRESENT";
    case SysState::WAIT_LEAVE_DELAY: return "WAIT_LEAVE_DELAY";
    case SysState::CLEANING: return "CLEANING";
    case SysState::COOLDOWN: return "COOLDOWN";
    case SysState::STOPPED_MANUAL: return "STOPPED_MANUAL";
    default: return "UNKNOWN";
  }
}

static void printStatus() {
  Serial.print("State=");
  Serial.print(stateName(state));
  Serial.print(" | IR_ACTIVE_LOW=");
  Serial.print(IR_ACTIVE_LOW ? "true" : "false");
  Serial.print(" | TRIGGER_ON_DETECT=");
  Serial.print(TRIGGER_ON_DETECT ? "true" : "false");
  Serial.print(" | obstacleDetected=");
  Serial.print(irObstacleDetectedRaw() ? "true" : "false");
  Serial.print(" | lastCleanAgo(ms)=");
  Serial.println((unsigned long)(millis() - t_last_clean));
}

static void handleSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') continue;

    if (c == 'h') {
      printHelp();
    } else if (c == 'p') {
      printStatus();
    } else if (c == 's') {
      Serial.println("Manual STOP (latched)");
      logMotorAction("Manual STOP requested");
      motorStopHard();
      setState(SysState::STOPPED_MANUAL);
    } else if (c == 'r') {
      Serial.println("Reset -> IDLE");
      logMotorAction("Manual reset requested");
      motorStopHard();
      setState(SysState::IDLE);
    } else if (c == 'c') {
      Serial.println("Manual CLEAN start");
      logMotorAction("Manual CLEAN requested");
      if (state != SysState::STOPPED_MANUAL) {
        setState(SysState::CLEANING);
      }
    } else if (c == 'd') {
      TRIGGER_ON_DETECT = !TRIGGER_ON_DETECT;
      Serial.print("TRIGGER_ON_DETECT toggled -> ");
      Serial.println(TRIGGER_ON_DETECT ? "true (detect event)" : "false (leave event)");
    } else if (c == 'l') {
      IR_ACTIVE_LOW = !IR_ACTIVE_LOW;
      Serial.print("IR_ACTIVE_LOW toggled -> ");
      Serial.println(IR_ACTIVE_LOW ? "true (active LOW)" : "false (active HIGH)");
    }
  }
}

//
// -------------------- Setup / Loop --------------------
//
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(' ');
  Serial.println("[BOOT] TT Motor + L9110S + IR Obstacle Sensor (Safety Gating Prototype)");
  Serial.println("[BOOT] Use 'h' for help. Monitor baud = 115200.");

  Serial.println("Boot OK - Cat Litterbox Motor Prototype (IR + L9110S)");
  Serial.println("Power note: Driver VCC should be VBUS (USB 5V). 3.3V may not spin TT motor.");

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_IR_OUT, INPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);

  ledcSetup(CH1, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(CH2, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_IN1, CH1);
  ledcAttachPin(PIN_IN2, CH2);

  motorStopHard();
  t_last_clean = 0;

  setState(SysState::IDLE);
  printHelp();
  printStatus();
  logIRChangeIfAny(irObstacleDetectedRaw());
}

void loop() {
  handleSerial();
  heartbeat();

  if (state == SysState::STOPPED_MANUAL) {
    digitalWrite(PIN_LED, LOW);
    motorStopHard();
    delay(10);
    return;
  }

  uint32_t now = millis();
  bool obstacleNow = irObstacleDetectedRaw();

  logIRChangeIfAny(obstacleNow);

  if (obstacleNow && state == SysState::IDLE) {
    setState(SysState::CAT_PRESENT);
    Serial.println("IR: obstacle detected -> CAT_PRESENT");
  }

  bool edgeEvent = irEdgeEventDebounced(obstacleNow);
  if (edgeEvent) {
    if (TRIGGER_ON_DETECT) {
      Serial.println("IR EVENT: DETECT");
    } else {
      Serial.println("IR EVENT: LEAVE");
    }
  }

  switch (state) {
    case SysState::IDLE: {
      digitalWrite(PIN_LED, LOW);
      break;
    }

    case SysState::CAT_PRESENT: {
      digitalWrite(PIN_LED, HIGH);

      if (!TRIGGER_ON_DETECT && edgeEvent) {
        Serial.println("Cat left -> starting leave-confirm delay");
        setState(SysState::WAIT_LEAVE_DELAY);
      }
      break;
    }

    case SysState::WAIT_LEAVE_DELAY: {
      digitalWrite(PIN_LED, HIGH);

      if (obstacleNow) {
        Serial.println("Cat returned during delay -> back to CAT_PRESENT");
        setState(SysState::CAT_PRESENT);
        break;
      }

      if (t_last_clean != 0 && (now - t_last_clean) < COOLDOWN_MS) {
        Serial.println("Cooldown active -> skipping cleaning");
        setState(SysState::COOLDOWN);
        break;
      }

      if ((now - t_state_enter) >= LEAVE_CONFIRM_MS) {
        Serial.println("Leave confirmed -> start CLEANING");
        setState(SysState::CLEANING);
      }
      break;
    }

    case SysState::CLEANING: {
      digitalWrite(PIN_LED, HIGH);

      if (obstacleNow) {
        Serial.println("Safety: obstacle detected during cleaning -> STOP");
        logMotorAction("Safety stop triggered by IR during cleaning");
        motorStopHard();
        setState(SysState::CAT_PRESENT);
        break;
      }

      logMotorAction("Cleaning cycle begin");
      motorRampForward(DUTY_MAX);
      logMotorAction("Run at DUTY_MAX");
      motorForwardDuty(DUTY_MAX);
      delay(CLEAN_RUN_MS);
      motorRampStopFrom(DUTY_MAX);
      delay(POST_STOP_MS);

      t_last_clean = millis();
      Serial.println("Cleaning complete -> COOLDOWN");
      logMotorAction("Cleaning cycle complete");
      setState(SysState::COOLDOWN);
      break;
    }

    case SysState::COOLDOWN: {
      digitalWrite(PIN_LED, LOW);

      if (t_last_clean != 0 && (now - t_last_clean) >= COOLDOWN_MS) {
        Serial.println("Cooldown finished -> IDLE");
        setState(SysState::IDLE);
      }
      break;
    }

    case SysState::STOPPED_MANUAL:
    default:
      break;
  }

  delay(5);
}