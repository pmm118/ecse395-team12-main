# ecse395-team12-main
ECSE395 Team12 Cat Litter Box
# testing 
# Prototype 1 – Motor + Safety Gating (TT Motor + L9110S + ESP32 + IR Obstacle Sensor)

**Team:** ECSE 395 Team 12  
**Owner (this prototype write-up):** Weikang Sun (Motor/Actuation)  
**Date:** 2026-03-04  
**Prototype Type / Fidelity:** Medium-fidelity electronics + control logic prototype (bench test)

---

## 1) Purpose (What question are we answering?)

**Primary question (risk/unknown):**  
What is the safest and most efficient control behavior for a cleaning motor mechanism when a cat **enters**, **occupies**, and **leaves** the litter box?

This directly supports our Verification & Validation focus on:  
- **Cleaning mechanism motors** + **cat detection sensor** testing  
- Safety/comfort: avoid motion when a cat is present  
- Noise and reliability constraints (quiet operation, repeatability)

---

## 2) Connection to V&V Plan (Why this prototype matters)

This prototype is intentionally targeted at the V&V risk question:  
> “What is the safest and most efficient way for our cleaning system to respond when a cat enters and leaves the litter box?”

V&V alignment categories:
- **Functional testing:** confirms “sensor → logic → motor action” works correctly
- **Safety testing:** confirms motor is inhibited when a cat is present; emergency stop logic exists
- **Noise consideration (future test):** provides adjustable PWM and soft-start/stop to reduce sudden motion and noise

---

## 3) Hypotheses / Assumptions

1. **Gate cleaning by cat presence**: The motor must never run while the cat is detected as present.  
2. **Trigger cleaning after cat leaves**: After the cat leaves, a short delay helps prevent false triggers (cat returns immediately).  
3. **Soft-start/soft-stop reduces sudden motion**: PWM ramping reduces “jerk” and likely reduces noise/stress.  
4. **Stable power is required**: Motor driver needs adequate supply (USB 5V/VBUS recommended) to overcome TT motor stall torque.

---

## 4) Prototype Build (Hardware + Wiring)

### 4.1 Bill of Materials
- Adafruit Feather ESP32 V2
- TT DC gear motor
- L9110S motor driver module
- SunFounder IR Obstacle Avoidance Module (digital OUT)
- Breadboard + jumper wires
- USB cable (data-capable)

### 4.2 Wiring (as-built)
**Motor driver (L9110S, Motor B channel):**
- ESP32 **A0 → B1A**
- ESP32 **A1 → B2A**
- TT motor leads → **Motor B output terminals**
- Driver **GND → ESP32 GND** (common ground)

**Power:**
- Driver **VCC → ESP32 VBUS (USB 5V)** (recommended for motor torque)
- ESP32 powered by USB from laptop

**IR obstacle module:**
- IR **VCC → ESP32 3V (3.3V)** (safe logic level)
- IR **GND → ESP32 GND**
- IR **OUT → ESP32 A2**
  
![Wiring Diagram: TT Motor + IR + ESP32](motor-l9110s-tt/hardware/wiring_diagram_prototype1.jpg)

*Figure 1. Wiring diagram of TT motor (L9110S) and IR obstacle sensor with ESP32 (Feather V2).*
---

## 5) Control Logic Implemented (What the code does)

### 5.1 State machine (high level)
- **IDLE:** waiting for cat interaction  
- **CAT_PRESENT:** IR detects obstacle → motor is locked out (never clean)  
- **WAIT_LEAVE_DELAY:** cat left → wait a confirmation delay; cancel if cat returns  
- **CLEANING:** run one timed cleaning cycle with soft-start/stop  
- **COOLDOWN:** prevent repeat cleaning for a minimum time window  
- **STOPPED_MANUAL:** emergency stop latched until reset

### 5.2 Safety rules
- If cat becomes detected during CLEANING → **immediate motor stop** and return to CAT_PRESENT  
- Manual emergency stop command forces STOPPED_MANUAL (motor disabled)

---

## 6) Test Plan (How we tested)

### 6.1 Setup
- Place IR sensor facing the “entry zone” of a mock litter box opening (or a small test box).
- Use a stuffed animal / hand / object as a repeatable “cat present” stimulus.
- Run PlatformIO serial monitor at 115200 baud to record state transitions.

### 6.2 Procedure (Trials)
Each trial follows:
1. Start in IDLE  
2. Present obstacle to IR sensor → expect CAT_PRESENT, motor must stay OFF  
3. Remove obstacle → expect WAIT_LEAVE_DELAY  
4. Keep area clear for leave-confirm window → expect CLEANING (motor runs)  
5. During CLEANING, re-introduce obstacle → expect immediate STOP and return to CAT_PRESENT  
6. After a completed cleaning cycle, attempt another leave event immediately → expect COOLDOWN blocks cleaning

### 6.3 Metrics recorded
- **Correct gating:** motor runs only when cat is not present (pass/fail)
- **Response time:** time from IR edge event to state change (qualitative now, measurable later)
- **False triggers:** counts of unexpected cleaning start
- **Power adequacy:** motor start success (start/no start)
- **Repeatability:** successful operation across multiple cycles

---

## 7) Evidence (Photos/Videos + Captions)

### Photos
- `motor-l9110s-tt/hardware/setup.jpg`  
  *Caption:* As-built bench setup photo (ESP32 + motor driver + TT motor + IR sensor + power module).

![As-built Setup Photo](motor-l9110s-tt/hardware/setup.jpg)

### Videos
- `motor-l9110s-tt/test/cat_activity_detection_cleaning.mp4`  
  *Caption:* Demonstrates cat-present lockout, then leave-confirm delay, then cleaning cycle.  
  *Link:* [cat_activity_detection_cleaning.mp4](motor-l9110s-tt/test/cat_activity_detection_cleaning.mp4)

- `motor-l9110s-tt/test/cat_activity_detection_during_CD.mp4`  
  *Caption:* Demonstrates cooldown preventing repeated cleaning (activity occurs during cooldown window and cleaning is blocked).  
  *Link:* [cat_activity_detection_during_CD.mp4](motor-l9110s-tt/test/cat_activity_detection_during_CD.mp4)

> Note: GitHub may show “Sorry about that, but we can’t show files that are this big right now.”  
> Use the **“View raw”** button to open/download the `.mp4`.

---

## 8) Results (What happened)

### Observations
- **State transitions were visible in serial logs** (IDLE → CAT_PRESENT → WAIT_LEAVE_DELAY → CLEANING → COOLDOWN).
- Motor behavior depended heavily on supply:
  - With insufficient motor supply, TT motor may fail to start (stall).
  - With USB 5V (VBUS) feeding the driver, start success improved.
- IR sensor logic polarity can vary by module version (active-low vs active-high). Code supports toggling.

### Pass/Fail Summary
- Motor never ran while IR indicated cat present: **PASS**
- Safety stop on re-detect during cleaning: **PASS**
- Leave-confirm delay prevented immediate re-trigger: **PASS**
- Cooldown blocked repeated cleaning: **PASS**
- Successful start rate over 10 cycles: **10 / 10**

---

## 9) Learning / Insights (What we learned)

1. **Power delivery is a first-order risk**: TT motor stall is common if powered from 3.3V; driver should be fed from USB 5V (VBUS) or Breadboard Power Supply Module for realistic torque.  
2. **Safety gating is feasible in software**: a simple state machine can enforce “never run motor with cat present” and still achieve a predictable cleaning cycle.  
3. **False triggers must be managed**: a leave-confirm delay and debounce are necessary to avoid cleaning when the cat briefly occludes the sensor.  
4. **Sensor polarity variability is real**: IR modules can be active-low or active-high; runtime-configurable logic reduces integration friction.

---

## 10) How this prototype influenced the next design decision (Iteration)

**Decision 1 (electrical):** Allocate a dedicated motor power path (USB 5V/VBUS or regulated 5V rail) separate from 3.3V logic.  
**Decision 2 (software):** Keep the state-machine gating approach; add logging hooks for future calendar/reminder integration.  
**Decision 3 (sensing):** IR obstacle is acceptable for early prototyping, but we may need more robust cat detection (weight-based or multi-sensor fusion) to reduce false positives/negatives.  
**Decision 4 (noise):** Soft-start/stop should remain; next prototype should quantify noise at 1m to align with noise requirement.

---

## 11) Next Steps (Prototype 2 / Prototype 3 suggestions)

- **Prototype 2:**  
- **Prototype 3:** 
---

## 12) Repo Structure (for the grader / reader)

- `src/main.cpp` : motor + IR integrated prototype code  
- `hardware/` : wiring + sensor placement photos  
- `test/` : behavior evidence videos  
- `README.md` : summary + links to this prototype section

### Prototype 1 structure
- `motor-l9110s-tt/hardware/` : wiring diagram + setup photo  
- `motor-l9110s-tt/test/` : behavior evidence videos (`.mp4`)
