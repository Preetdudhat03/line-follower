/*
================================================================
              TECHGEEKS SPIRIT SUPERFAST LFR
                  PID CONTROLLER V4.1 SIDE-BLACK/CENTER-GAP STRAIGHT + ACCURACY
================================================================

HARDWARE
---------------------------------------------------------------
Arduino Nano ATmega328P @ 16 MHz
ARC-8 Analog Sensor Array
TB6612FNG Dual Motor Driver
2 x N20 Motors
8520 Coreless Impeller + USC
Spirit PCB-cum-Chassis


PHYSICAL PIN MAP - YOUR SPIRIT BOARD
---------------------------------------------------------------

ARC-8:
    S0 = A0
    S1 = A1
    S2 = A2
    S3 = A3
    S4 = A4
    S5 = A5
    S6 = A6
    S7 = A7

MOTOR A:
    PWMA = D10
    AIN1 = D8
    AIN2 = D9

MOTOR B:
    PWMB = D5
    BIN1 = D7
    BIN2 = D6

BUTTONS:
    LB = D12
    RB = D4

LEDs:
    LED1 = D2
    LED2 = D3

IMPELLER:
    D11 / OC2A / TIMER2


VERIFIED HARDWARE BEHAVIOR
---------------------------------------------------------------

Motor A D1     = WORKS
Motor A D2     = WORKS

Motor B D1     = WORKS
Motor B D2     = DOES NOT WORK

A D1 + B D1    = ROBOT MOVES FORWARD

Therefore:
    NORMAL DRIVE = A D1 + B D1

NO MOTOR REVERSE IS USED BY PID.


BUTTON SEQUENCE
---------------------------------------------------------------

POWER ON
    |
    v
SYSTEM READY
    |
    | Press LB
    v
CALIBRATION READY / SENSOR VALUES LOADED
    |
    | Press LB again
    v
IMPELLER START
    |
    v
VACUUM RUNNING
    |
    | Press RB
    v
IMMEDIATE RUN AFTER RB
    |
    v
PID LINE FOLLOWING


IMPORTANT
---------------------------------------------------------------

This firmware uses the previously verified ARC-8 calibration:

S0 MIN 65  MAX 944
S1 MIN 52  MAX 956
S2 MIN 47  MAX 948
S3 MIN 54  MAX 958
S4 MIN 48  MAX 950
S5 MIN 67  MAX 822
S6 MIN 57  MAX 957
S7 MIN 90  MAX 964

The impeller uses Timer2 / D11.

The PID controller uses forward-only differential steering.

SPECIAL TRACK BEHAVIOR
---------------------------------------------------------------
Normal line width is 20 mm. A 6+ sensor wide pattern is treated
as an intersection and both motors are commanded equally so the
robot passes STRAIGHT instead of following the weighted average.

ADDED V4.1:
If both outer sides are black while at least one center sensor is
white (for example 1,1,1,0,1,1,1,1), this is also treated as a
VALID STRAIGHT TRACK PATTERN. Both motors receive equal PWM and
sharp-turn/intersection/PID steering is skipped for that cycle.

START/END black-box detection is removed. RB controls run/stop.

================================================================
*/


#include <Arduino.h>
#include <string.h>
#include <stdlib.h>


// ================================================================
// 1. PIN DEFINITIONS
// ================================================================

// ------------------------------------------------
// ARC-8 SENSOR ARRAY
// ------------------------------------------------

const uint8_t SENSOR_PINS[8] =
{
    A0,
    A1,
    A2,
    A3,
    A4,
    A5,
    A6,
    A7
};


// ------------------------------------------------
// MOTOR A
// ------------------------------------------------

const uint8_t PWMA = 10;
const uint8_t AIN1 = 8;
const uint8_t AIN2 = 9;


// ------------------------------------------------
// MOTOR B
// ------------------------------------------------

const uint8_t PWMB = 5;
const uint8_t BIN1 = 7;
const uint8_t BIN2 = 6;


// ------------------------------------------------
// BUTTONS
// ------------------------------------------------

const uint8_t LB = 12;
const uint8_t RB = 4;


// ------------------------------------------------
// STATUS LEDs
// ------------------------------------------------

const uint8_t LED1 = 2;
const uint8_t LED2 = 3;


// ------------------------------------------------
// IMPELLER
// ------------------------------------------------

const uint8_t IMPELLER = 11;


// ================================================================
// 2. STARTUP SENSOR CALIBRATION FALLBACK
// ================================================================

int sensorMin[8] =
{
    40,
    38,
    37,
    43,
    37,
    39,
    38,
    44
};


int sensorMax[8] =
{
    870,
    781,
    782,
    834,
    818,
    910,
    769,
    816
};


// ================================================================
// 3. SENSOR POSITION
// ================================================================

const int SENSOR_POSITION[8] =
{
    0,
    1000,
    2000,
    3000,
    4000,
    5000,
    6000,
    7000
};


const int CENTER_POSITION = 3500;


// ================================================================
// 4. LINE DETECTION
// ================================================================

// Hysteresis prevents a sensor hovering around the threshold from
// rapidly switching BLACK/WHITE and injecting steering noise.
const int LINE_THRESHOLD_ON  = 620;
const int LINE_THRESHOLD_OFF = 560;
const int LINE_THRESHOLD = 600;

// Minimum usable signal after thresholding.
const int LINE_MIN_TOTAL = 140;

// Sensor filtering: fast enough for 200 Hz control, but removes ADC noise.
const float SENSOR_FILTER_ALPHA = 0.45;

// Stronger sensor readings receive slightly more weight.  This improves
// centroid accuracy on a 20 mm line without requiring more sensors.
const uint8_t POSITION_WEIGHT_POWER = 2;

// ================================================================
// START / STOP CONTROL
// ================================================================
// No START/END black-box detection is used in V3.5.
// After LB #1, LB #2 and RB, the robot starts immediately.
// RB during a run stops the robot; RB while stopped starts it again.

// Official rulebook time limit: 180 seconds per run.
const unsigned long MAX_RUN_TIME_MS = 180000UL;

// ================================================================
// INTERSECTION DETECTION FRAMEWORK
// ================================================================
// The competition line is 20 mm wide. At a crossing, the sensor array
// can see black in a broader pattern than on the normal line.
//
// V3.5 changes the strategy: do NOT use the previous steering command
// at an intersection. That command can already contain a right/left
// bias and can pull the robot into a branch. Instead, once a junction
// is confirmed, the robot temporarily drives straight with equal motor
// PWM and ignores the ambiguous weighted position.
//
// Detection uses both a broad sensor count and a sudden increase in
// black coverage, which helps distinguish a crossing from a normal
// 20 mm curve.
const uint8_t INTERSECTION_MIN_SENSORS = 5;
const unsigned long INTERSECTION_CONFIRM_MS = 55UL;
const unsigned long INTERSECTION_COOLDOWN_MS = 500UL;
const unsigned long INTERSECTION_STRAIGHT_MS = 170UL;
const unsigned long INTERSECTION_CLEAR_MS = 70UL;
const int INTERSECTION_MAX_CENTER_ERROR = 500;

// A real crossing should have black on BOTH sides of the sensor array
// while the main line is still near the center. A curve/90-degree turn
// normally occupies one side only.
const uint8_t INTERSECTION_CENTER_MIN = 2;
const uint8_t INTERSECTION_LEFT_MIN = 1;
const uint8_t INTERSECTION_RIGHT_MIN = 1;

unsigned long intersectionWideSince = 0;
unsigned long lastIntersectionTime = 0;
unsigned long intersectionStraightUntil = 0;
unsigned long intersectionClearSince = 0;
bool intersectionActive = false;

// Used only for telemetry/debugging.
int lastMotorA = 0;
int lastMotorB = 0;
bool haveLastDrive = false;


// ================================================================
// 5. PID SETTINGS - V3 COMPETITION CONTROLLER
// ================================================================

/*
   V2 improvements:

   1. Fixed PID sample interval
   2. Real derivative = d(error) / dt
   3. Low-pass derivative filtering
   4. Time-based integral
   5. Integral anti-windup
   6. Live Serial tuning
   7. Live telemetry for tuning

   IMPORTANT:
   Kd uses seconds-based derivative in V2, so its numeric value
   is intentionally much smaller than the old Kd=160 setting.
*/

float Kp = 20.0;
float Ki = 0.0;
float Kd = 1.0;

float previousError = 0.0;
float integral = 0.0;
float filteredDerivative = 0.0;

// PID loop target: 200 Hz = 5 ms
unsigned long PID_SAMPLE_US = 5000UL;

// Derivative low-pass filter. 0.25 = moderate filtering.
float derivativeFilterAlpha = 0.25;

// Integral clamp. Tune Ki only after Kp and Kd are stable.
float INTEGRAL_LIMIT = 5000.0;

// ------------------------------------------------
// MOTOR SPEED
// ------------------------------------------------

int BASE_SPEED = 40; //210

int MAX_SPEED = 80; //255

int MIN_FORWARD_SPEED = 5;  

// ================================================================
// ADAPTIVE CURVE SPEED CONTROL
// ================================================================
// The track line is 20 mm wide. On a curve the line moves away from
// the sensor-array center, so |error| becomes larger. Instead of
// entering every curve at the full straight-line speed, the controller
// automatically reduces the forward speed as the turn demand grows.
//
// Straight:      error near 0       -> BASE_SPEED
// Gentle curve:  moderate error      -> small speed reduction
// Hard curve:    large error         -> much slower
//
// This speed reduction is applied only during normal PID tracking.
// Intersection handling happens before this calculation and therefore
// is not slowed by the curve-speed logic.
const int CURVE_MIN_SPEED = 25;
const float CURVE_ERROR_START = 220.0;
const float CURVE_ERROR_FULL = 1700.0;

// Smooth the requested speed so the robot does not suddenly accelerate
// or brake when the error changes by one sensor sample.
float adaptiveBaseSpeed = 40.0;
const float SPEED_SMOOTHING = 0.28;

// If the line is changing position very quickly, start slowing slightly
// earlier. This is an anticipation term; it is intentionally modest so
// normal sensor noise does not make the robot crawl.
const float CURVE_DERIVATIVE_START = 1400.0;
const float CURVE_DERIVATIVE_FULL = 7500.0;
const float DERIVATIVE_SPEED_WEIGHT = 0.20;

// ================================================================
// SHARP / 90-DEGREE TURN CONTROL
// ================================================================
// A 90-degree corner is NOT an intersection. When the line moves far
// from the center, slow the robot aggressively and give it a much
// stronger turning differential. This prevents the robot from running
// past the corner, losing the line, and entering forward-only recovery
// that can become a U-turn.
const float SHARP_TURN_ERROR = 1700.0;
const float SHARP_TURN_FULL_ERROR = 3000.0;
const int SHARP_TURN_MIN_SPEED = 8;
const int SHARP_TURN_OUTER_SPEED = 120;
const unsigned long SHARP_TURN_HOLD_MS = 120UL;
const unsigned long SHARP_TURN_MAX_MS = 380UL;

// Your latest test shows the current motor differential steers the wrong
// way. Set true to reverse the PID/90-degree steering direction.
const bool INVERT_STEERING = false;


// ------------------------------------------------
// LIVE TUNING / TELEMETRY
// ------------------------------------------------

bool telemetryEnabled = true;

unsigned long lastPIDTime = 0;
unsigned long lastTelemetryTime = 0;

const unsigned long TELEMETRY_INTERVAL_MS = 50;

// Serial command buffer
char serialBuffer[64];
uint8_t serialIndex = 0;

// ================================================================
// 6. IMPELLER SETTINGS
// ================================================================

/*
   Your Timer2 impeller test worked.

   Start conservatively at 30/255.

   Do NOT immediately increase this.

   TechGeeks documents the 2S Spirit impeller setup
   with conservative duty limits.
*/

const uint8_t IMPELLER_START_DUTY = 30;


// ================================================================
// 7. ROBOT STATES
// ================================================================

enum RobotState
{
    STATE_STARTUP,
    STATE_WAIT_CALIBRATION,
    STATE_CALIBRATION_COMPLETE,
    STATE_WAIT_IMPELLER,
    STATE_IMPELLER_STARTING,
    STATE_VACUUM_READY,
    STATE_RUNNING,
    STATE_LINE_LOST,
    STATE_STOPPED
};


RobotState state = STATE_STARTUP;


// ================================================================
// 8. BUTTON STATE
// ================================================================

bool previousLB = HIGH;
bool previousRB = HIGH;

unsigned long lastLBTime = 0;
unsigned long lastRBTime = 0;

const unsigned long BUTTON_DEBOUNCE = 80;


// ================================================================
// DOUBLE-LB SENSOR CALIBRATION
// ================================================================
// Fast double-click on LB starts a real 20-second ARC-8 calibration.
//
// During these 20 seconds, move the sensor array over BOTH:
//   - white floor
//   - black line
//
// LEDs:
//   - alternating LEDs = calibration running
//   - both LEDs ON solid = calibration complete and valid
//   - LED1 ON / LED2 OFF = calibration range is insufficient; retry
//
// The new calibration is kept in RAM for this power session.
const unsigned long LB_DOUBLE_CLICK_WINDOW_MS = 500UL;
const unsigned long SENSOR_CALIBRATION_TIME_MS = 20000UL;

bool calibrationRunning = false;
bool calibrationFinished = false;
bool calibrationValid = false;

unsigned long calibrationStartTime = 0;
unsigned long calibrationLastLEDTime = 0;

bool calibrationLEDState = false;

int calibrationMin[8];
int calibrationMax[8];
int sensorFiltered[8] = {0,0,0,0,0,0,0,0};
uint8_t sensorBlack[8] = {0,0,0,0,0,0,0,0};
bool sensorFilterInitialized = false;
int lastValidPosition = CENTER_POSITION;
int lastStrongSensor = 3;
int lastLineDirection = 0; // -1 left, +1 right
unsigned long lineLostSince = 0;
const unsigned long MAX_DASHED_GAP_MS = 180UL;


unsigned long firstLBPressTime = 0;
bool waitingForSecondLB = false;


// ================================================================
// 9. TIMERS
// ================================================================

unsigned long stateStartTime = 0;

// Run timer: starts whenever a new RUN begins.
unsigned long runStartTime = 0;

// Last PID steering command, retained for telemetry/debugging and
// future intersection handling.
float lastSteeringCorrection = 0.0;

unsigned long lastLEDTime = 0;

unsigned long lastDebugTime = 0;

bool ledState = false;


// ================================================================
// 10. MOTOR A
// ================================================================

/*
   VERIFIED:

   D1 = physical forward direction
*/

void motorAForward(int speed)
{
    speed = constrain(speed, 0, 255);

    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    analogWrite(PWMA, speed);
}


// ================================================================
// 11. MOTOR B
// ================================================================

/*
   VERIFIED:

   D1 = physical forward direction

   Motor B reverse is NOT used.
*/

void motorBForward(int speed)
{
    speed = constrain(speed, 0, 255);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(PWMB, speed);
}


// ================================================================
// 12. MOTOR STOP
// ================================================================

void stopMotors()
{
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}


// ================================================================
// 13. IMPELLER TIMER2 INITIALIZATION
// ================================================================

void setupImpellerPWM()
{
    pinMode(IMPELLER, OUTPUT);

    /*
       ATmega328P Timer2

       D11 = OC2A

       Fast PWM
       TOP = 255
       Prescaler = 1

       Frequency:

       16 MHz / 256
       = 62.5 kHz
    */

    TCCR2A = 0;

    TCCR2B = 0;

    // Fast PWM
    TCCR2A |= (1 << WGM20);
    TCCR2A |= (1 << WGM21);

    // Clear OC2A on compare match
    TCCR2A |= (1 << COM2A1);

    // No prescaler
    TCCR2B |= (1 << CS20);

    OCR2A = 0;
}


// ================================================================
// 14. IMPELLER OFF
// ================================================================

void impellerOff()
{
    OCR2A = 0;
}


// ================================================================
// 15. IMPELLER SET
// ================================================================

void setImpeller(uint8_t duty)
{
    OCR2A = duty;
}


// ================================================================
// 16. IMPELLER SOFT START
// ================================================================

void startImpeller()
{
    // IMPELLER DISABLED: not required for this configuration.
    impellerOff();
    Serial.println(F("[IMPELLER] DISABLED"));
}



// ================================================================
// 17. LED HELPERS
// ================================================================

void ledsOff()
{
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
}


void ledsOn()
{
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, HIGH);
}


// ================================================================
// 18. LED STATUS SYSTEM
// ================================================================

void updateLEDs()
{
    unsigned long now = millis();

    switch (state)
    {
        // --------------------------------------------------------
        // STARTUP
        // --------------------------------------------------------

        case STATE_STARTUP:

            if (now - lastLEDTime >= 250)
            {
                lastLEDTime = now;

                ledState = !ledState;

                digitalWrite(LED1, ledState);
                digitalWrite(LED2, ledState);
            }

            break;


        // --------------------------------------------------------
        // WAIT CALIBRATION
        // --------------------------------------------------------

        case STATE_WAIT_CALIBRATION:

            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, LOW);

            break;


        // --------------------------------------------------------
        // CALIBRATION COMPLETE
        // --------------------------------------------------------

        case STATE_CALIBRATION_COMPLETE:

            ledsOn();

            break;


        // --------------------------------------------------------
        // WAIT IMPELLER
        // --------------------------------------------------------

        case STATE_WAIT_IMPELLER:

            digitalWrite(LED1, HIGH);

            if (now - lastLEDTime >= 500)
            {
                lastLEDTime = now;

                ledState = !ledState;

                digitalWrite(LED2, ledState);
            }

            break;


        // --------------------------------------------------------
        // IMPELLER STARTING
        // --------------------------------------------------------

        case STATE_IMPELLER_STARTING:

            if (now - lastLEDTime >= 100)
            {
                lastLEDTime = now;

                ledState = !ledState;

                digitalWrite(LED1, ledState);
                digitalWrite(LED2, ledState);
            }

            break;


        // --------------------------------------------------------
        // VACUUM READY
        // --------------------------------------------------------

        case STATE_VACUUM_READY:

            ledsOn();

            break;


        case STATE_RUNNING:

            /*
               Slow heartbeat while PID is active.
            */

            if (now - lastLEDTime >= 600)
            {
                lastLEDTime = now;

                ledState = !ledState;

                digitalWrite(LED1, ledState);
                digitalWrite(LED2, ledState);
            }

            break;


        // --------------------------------------------------------
        // LINE LOST
        // --------------------------------------------------------

        case STATE_LINE_LOST:

            /*
               Rapid alternating LEDs.
            */

            if (now - lastLEDTime >= 80)
            {
                lastLEDTime = now;

                ledState = !ledState;

                digitalWrite(LED1, ledState);
                digitalWrite(LED2, !ledState);
            }

            break;


        // --------------------------------------------------------
        // STOPPED
        // --------------------------------------------------------

        case STATE_STOPPED:

            ledsOff();

            break;
    }
}


// ================================================================
// 19. CHANGE STATE
// ================================================================

void changeState(RobotState newState)
{
    state = newState;

    stateStartTime = millis();

    Serial.println();
    Serial.println(F("========================================"));

    switch (state)
    {
        case STATE_STARTUP:

            Serial.println(F("STATE: STARTUP"));

            break;


        case STATE_WAIT_CALIBRATION:

            Serial.println(F("STATE: READY"));
            Serial.println(F("LB = START VACUUM | LB+LB = CALIBRATE"));

            break;


        case STATE_CALIBRATION_COMPLETE:

            Serial.println(F("STATE: CALIBRATION COMPLETE"));
            Serial.println(F("SENSOR VALUES LOADED"));

            break;


        case STATE_WAIT_IMPELLER:

            Serial.println(F("STATE: WAITING FOR IMPELLER"));
            Serial.println(F("PRESS LB AGAIN"));

            break;


        case STATE_IMPELLER_STARTING:

            Serial.println(F("STATE: IMPELLER STARTING"));

            break;


        case STATE_VACUUM_READY:

            Serial.println(F("STATE: VACUUM READY"));
            Serial.println(F("PRESS RB FOR FINAL START - IMPELLER DISABLED"));

            break;


        case STATE_RUNNING:

            Serial.println(F("STATE: RUN MODE"));
            Serial.println(F("PID ACTIVE"));
            Serial.println(F("MOTORS ACTIVE"));
            Serial.println(F("IMPELLER DISABLED"));

            break;


        case STATE_LINE_LOST:

            Serial.println(F("STATE: LINE LOST"));
            Serial.println(F("RECOVERY ACTIVE"));

            break;


        case STATE_STOPPED:

            Serial.println(F("STATE: STOPPED"));

            break;
    }

    Serial.println(F("========================================"));
}


// ================================================================
// 20. LEFT BUTTON
// ================================================================

bool leftButtonPressed()
{
    bool current = digitalRead(LB);

    if (previousLB == HIGH &&
        current == LOW &&
        millis() - lastLBTime > BUTTON_DEBOUNCE)
    {
        lastLBTime = millis();

        previousLB = current;

        Serial.println(F("[BUTTON] LB"));

        return true;
    }

    previousLB = current;

    return false;
}


// ================================================================
// 21. RIGHT BUTTON
// ================================================================

bool rightButtonPressed()
{
    bool current = digitalRead(RB);

    if (previousRB == HIGH &&
        current == LOW &&
        millis() - lastRBTime > BUTTON_DEBOUNCE)
    {
        lastRBTime = millis();

        previousRB = current;

        Serial.println(F("[BUTTON] RB"));

        return true;
    }

    previousRB = current;

    return false;
}


// ================================================================
// REAL 20-SECOND SENSOR CALIBRATION
// ================================================================

void beginSensorCalibration()
{
    calibrationRunning = true;
    calibrationFinished = false;

    calibrationStartTime = millis();
    calibrationLastLEDTime = millis();
    calibrationLEDState = false;

    // Initialize min/max with the current raw reading.
    for (uint8_t i = 0; i < 8; i++)
    {
        int raw = analogRead(SENSOR_PINS[i]);

        calibrationMin[i] = raw;
        calibrationMax[i] = raw;
    }

    stopMotors();

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F("      SENSOR CALIBRATION STARTED"));
    Serial.println(F("========================================"));
    Serial.println(F("CALIBRATION TIME: 20 SECONDS"));
    Serial.println(F("MOVE SENSORS OVER WHITE AND BLACK"));
    Serial.println(F("LEDs ALTERNATING = CALIBRATING"));
    Serial.println(F("========================================"));

    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
}


void updateSensorCalibration()
{
    if (!calibrationRunning)
        return;

    const unsigned long now = millis();

    // Continuously record minimum and maximum raw value
    // for every sensor during the 20-second window.
    for (uint8_t i = 0; i < 8; i++)
    {
        int raw = analogRead(SENSOR_PINS[i]);

        if (raw < calibrationMin[i])
            calibrationMin[i] = raw;

        if (raw > calibrationMax[i])
            calibrationMax[i] = raw;
    }

    // Alternate LEDs every 150 ms while calibration is active.
    if (now - calibrationLastLEDTime >= 150UL)
    {
        calibrationLastLEDTime = now;
        calibrationLEDState = !calibrationLEDState;

        digitalWrite(LED1, calibrationLEDState);
        digitalWrite(LED2, !calibrationLEDState);
    }

    // Finish after 20 seconds.
    if (now - calibrationStartTime >= SENSOR_CALIBRATION_TIME_MS)
    {
        calibrationRunning = false;
        calibrationFinished = true;

        bool valid = true;

        Serial.println();
        Serial.println(F("========================================"));
        Serial.println(F("      SENSOR CALIBRATION COMPLETE"));
        Serial.println(F("========================================"));

        for (uint8_t i = 0; i < 8; i++)
        {
            int range = calibrationMax[i] - calibrationMin[i];

            if (range < 100)
                valid = false;

            sensorMin[i] = calibrationMin[i];
            sensorMax[i] = calibrationMax[i];

            Serial.print(F("S"));
            Serial.print(i);
            Serial.print(F(" MIN="));
            Serial.print(sensorMin[i]);
            Serial.print(F(" MAX="));
            Serial.print(sensorMax[i]);
            Serial.print(F(" RANGE="));
            Serial.println(range);
        }

        calibrationValid = valid;

        if (valid)
        {
            Serial.println(F("CALIBRATION STATUS: OK"));
            Serial.println(F("BOTH LEDs ON = CALIBRATION DONE"));
            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, HIGH);
        }
        else
        {
            Serial.println(F("CALIBRATION STATUS: INVALID"));
            Serial.println(F("LED1 ON / LED2 OFF = RETRY CALIBRATION"));
            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, LOW);
        }

        resetPID();
    }
}


bool handleLBDoubleClick()
{
    const unsigned long now = millis();

    if (calibrationRunning)
        return false;

    bool current = digitalRead(LB);

    // Detect a new LB press.
    if (previousLB == HIGH &&
        current == LOW &&
        now - lastLBTime > BUTTON_DEBOUNCE)
    {
        lastLBTime = now;
        previousLB = current;

        // Second press inside the double-click window.
        if (waitingForSecondLB &&
            now - firstLBPressTime <= LB_DOUBLE_CLICK_WINDOW_MS)
        {
            waitingForSecondLB = false;

            Serial.println(F("[BUTTON] LB DOUBLE-CLICK -> CALIBRATION"));

            beginSensorCalibration();

            return true;
        }

        // First press: wait for a second quick press.
        firstLBPressTime = now;
        waitingForSecondLB = true;

        Serial.println(F("[BUTTON] LB #1 - press LB again quickly"));

        return true;
    }

    previousLB = current;

    // Double-click window expired.
    // A single LB is the normal vacuum-start command.
    // We wait only for the double-click window so we can distinguish
    // a normal single press from LB + LB calibration.
    if (waitingForSecondLB &&
        now - firstLBPressTime > LB_DOUBLE_CLICK_WINDOW_MS)
    {
        waitingForSecondLB = false;

        Serial.println(F("[BUTTON] SINGLE LB -> RUN READY (IMPELLER DISABLED)"));

        // No impeller/vacuum is used in this version.
        impellerOff();
        changeState(STATE_VACUUM_READY);
    }

    return false;
}


// ================================================================
// 22. SENSOR SAMPLING + NORMALIZATION
// ================================================================

int normalizeRaw(uint8_t index, int raw)
{
    int minimum = sensorMin[index];
    int maximum = sensorMax[index];

    if (maximum <= minimum)
        return 0;

    long value = ((long)(raw - minimum) * 1000L) /
                 (maximum - minimum);

    return constrain((int)value, 0, 1000);
}

void sampleSensors()
{
    for (uint8_t i = 0; i < 8; i++)
    {
        int raw = analogRead(SENSOR_PINS[i]);
        int value = normalizeRaw(i, raw);

        if (!sensorFilterInitialized)
        {
            sensorFiltered[i] = value;
        }
        else
        {
            sensorFiltered[i] =
                (int)((SENSOR_FILTER_ALPHA * value) +
                      ((1.0 - SENSOR_FILTER_ALPHA) * sensorFiltered[i]) +
                      0.5);
        }

        // Hysteresis: once black, require a larger drop before turning off.
        if (sensorBlack[i])
        {
            if (sensorFiltered[i] < LINE_THRESHOLD_OFF)
                sensorBlack[i] = 0;
        }
        else
        {
            if (sensorFiltered[i] >= LINE_THRESHOLD_ON)
                sensorBlack[i] = 1;
        }
    }

    sensorFilterInitialized = true;
}

// Backward-compatible helper for any code that needs one current sensor.
int readNormalizedSensor(uint8_t index)
{
    return sensorFiltered[index];
}


// ================================================================
// SENSOR SERIAL MONITOR DEBUG
// ================================================================
// Prints RAW ADC, normalized value, filtered value and BLACK state.
// This is independent of PID telemetry, so sensor values are visible
// even when the robot is not running.
const unsigned long SENSOR_DEBUG_INTERVAL_MS = 100UL;
unsigned long lastSensorDebugTime = 0;

void printSensorReadings()
{
    unsigned long now = millis();

    if (now - lastSensorDebugTime < SENSOR_DEBUG_INTERVAL_MS)
        return;

    lastSensorDebugTime = now;

    // IMPORTANT: when the robot is stopped, runPID() is not called, so
    // sensorFiltered[] and sensorBlack[] would otherwise remain zero.
    // Take a real sensor/filter sample here whenever PID is not sampling.
    if (state != STATE_RUNNING && state != STATE_LINE_LOST)
    {
        sampleSensors();
    }

    Serial.print(F("SENSORS | RAW: "));

    for (uint8_t i = 0; i < 8; i++)
    {
        int raw = analogRead(SENSOR_PINS[i]);

        Serial.print(raw);

        if (i < 7)
            Serial.print(F(","));
    }

    Serial.print(F(" | NORM: "));

    for (uint8_t i = 0; i < 8; i++)
    {
        int raw = analogRead(SENSOR_PINS[i]);
        int normalized = normalizeRaw(i, raw);

        Serial.print(normalized);

        if (i < 7)
            Serial.print(F(","));
    }

    Serial.print(F(" | FILT: "));

    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.print(sensorFiltered[i]);

        if (i < 7)
            Serial.print(F(","));
    }

    Serial.print(F(" | BLACK: "));

    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.print(sensorBlack[i]);

        if (i < 7)
            Serial.print(F(","));
    }

    Serial.println();
}



// ================================================================
// 90-DEGREE TURN STATE
// ================================================================
bool sharpTurnActive = false;
bool sharpTurnRight = false;
unsigned long sharpTurnStartMs = 0;

bool detect90DegreeEdgeTurn(bool &turnRight)
{
    // A 90-degree corner normally puts the line strongly on one edge.
    // Require two adjacent edge sensors so a single noisy sensor cannot
    // start a turn. Also reject the both-sides-black pattern because
    // that pattern is handled as a straight track feature.
    const bool leftEdge = sensorBlack[0] && sensorBlack[1];
    const bool rightEdge = sensorBlack[6] && sensorBlack[7];
    const bool leftSupport = sensorBlack[2];
    const bool rightSupport = sensorBlack[5];

    if (leftEdge && !rightEdge && (leftSupport || sensorBlack[0]))
    {
        turnRight = false;
        return true;
    }

    if (rightEdge && !leftEdge && (rightSupport || sensorBlack[7]))
    {
        turnRight = true;
        return true;
    }

    return false;
}

void drive90DegreeTurn(bool turnRight)
{
    int inner = constrain(SHARP_TURN_MIN_SPEED, 0, MAX_SPEED);
    int outer = constrain(SHARP_TURN_OUTER_SPEED, MIN_FORWARD_SPEED, MAX_SPEED);

    if (turnRight)
    {
        motorAForward(outer);
        motorBForward(inner);
        lastMotorA = outer;
        lastMotorB = inner;
    }
    else
    {
        motorAForward(inner);
        motorBForward(outer);
        lastMotorA = inner;
        lastMotorB = outer;
    }

    haveLastDrive = true;
    integral = 0.0;
    filteredDerivative = 0.0;
    previousError = turnRight ? 2500.0 : -2500.0;
}

// ================================================================
// SPECIAL TRACK PATTERN
// BOTH SIDES BLACK + CENTER GAP/WHITE = STRAIGHT
// ================================================================
//
// Your track can produce patterns such as:
//
//     S0 S1 S2 S3 S4 S5 S6 S7
//      1  1  1  0  1  1  1  1
//      B  B  B  W  B  B  B  B
//
// The normal weighted centroid can see this as a large/ambiguous
// pattern. For this known track feature, treat it as a valid
// straight section and force equal motor speed.
//
// IMPORTANT:
// - Left side needs at least 2 of S0,S1,S2 BLACK.
// - Right side needs at least 2 of S5,S6,S7 BLACK.
// - At least ONE of the two center sensors S3/S4 must be WHITE.
// - This function is checked BEFORE sharp-turn/intersection/PID logic.
// ================================================================

bool isSideBlackCenterGapStraight()
{
    uint8_t leftBlack = 0;
    uint8_t rightBlack = 0;

    for (uint8_t i = 0; i <= 2; i++)
    {
        if (sensorBlack[i])
            leftBlack++;
    }

    for (uint8_t i = 5; i <= 7; i++)
    {
        if (sensorBlack[i])
            rightBlack++;
    }

    const bool centerGap =
        (!sensorBlack[3] || !sensorBlack[4]);

    return (
        leftBlack >= 2 &&
        rightBlack >= 2 &&
        centerGap
    );
}


void driveSideBlackCenterGapStraight()
{
    const int straightSpeed =
        constrain(BASE_SPEED,
                  MIN_FORWARD_SPEED,
                  MAX_SPEED);

    motorAForward(straightSpeed);
    motorBForward(straightSpeed);

    lastSteeringCorrection = 0.0;
    lastMotorA = straightSpeed;
    lastMotorB = straightSpeed;
    haveLastDrive = true;

    // Keep PID memory neutral so the ambiguous pattern cannot
    // immediately cause a turn on the next control cycle.
    previousError = 0.0;
    integral = 0.0;
    filteredDerivative = 0.0;
    lastValidPosition = CENTER_POSITION;
    lastLineDirection = 0;
}

// ================================================================
// 23. READ LINE POSITION
// ================================================================

bool getLinePosition(float &position, float &error)
{
    long weightedSum = 0;
    long total = 0;

    uint8_t strongCount = 0;
    int strongestValue = 0;
    int strongestIndex = lastStrongSensor;

    for (uint8_t i = 0; i < 8; i++)
    {
        int value = sensorFiltered[i];

        // Ignore weak responses. Hysteresis decides black/white state.
        if (!sensorBlack[i])
            value = 0;

        if (value > strongestValue)
        {
            strongestValue = value;
            strongestIndex = i;
        }

        if (sensorBlack[i])
            strongCount++;

        // Square weighting makes the actual black-line peak dominate
        // weaker reflected-light shoulders.
        long weight = value;
        if (POSITION_WEIGHT_POWER == 2)
            weight = (weight * weight) / 1000L;

        weightedSum += weight * SENSOR_POSITION[i];
        total += weight;
    }

    if (strongCount > 0)
        lastStrongSensor = strongestIndex;

    if (total < LINE_MIN_TOTAL)
        return false;

    position = (float)weightedSum / (float)total;
    error = position - CENTER_POSITION;

    lastValidPosition = (int)position;

    if (error > 100.0)
        lastLineDirection = +1;
    else if (error < -100.0)
        lastLineDirection = -1;

    return true;
}


// ================================================================
// 24. INTERSECTION DETECTION
// ================================================================

uint8_t countStrongBlackSensors()
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        if (sensorBlack[i])
            count++;
    }

    return count;
}


bool detectIntersection(float currentError)
{
    const unsigned long now = millis();

    // Read the array once. We deliberately use the physical pattern,
    // not only the weighted error. This is critical because a 90-degree
    // corner can also create a large sensor count for a short moment.
    uint8_t strong[8];
    uint8_t totalStrong = 0;
    uint8_t leftStrong = 0;
    uint8_t centerStrong = 0;
    uint8_t rightStrong = 0;

    for (uint8_t i = 0; i < 8; i++)
    {
        strong[i] = (sensorBlack[i]) ? 1 : 0;
        totalStrong += strong[i];

        if (i <= 2 && strong[i]) leftStrong++;
        if (i >= 2 && i <= 5 && strong[i]) centerStrong++;
        if (i >= 5 && strong[i]) rightStrong++;
    }

    // A crossing has black on both outer sides and in the middle.
    // A normal 20-mm line/curve usually does not satisfy all three.
    const bool crossingShape =
        totalStrong >= INTERSECTION_MIN_SENSORS &&
        leftStrong >= INTERSECTION_LEFT_MIN &&
        centerStrong >= INTERSECTION_CENTER_MIN &&
        rightStrong >= INTERSECTION_RIGHT_MIN &&
        fabs(currentError) <= INTERSECTION_MAX_CENTER_ERROR;

    if (crossingShape)
    {
        if (intersectionWideSince == 0)
            intersectionWideSince = now;

        if (!intersectionActive &&
            now - intersectionWideSince >= INTERSECTION_CONFIRM_MS &&
            now - lastIntersectionTime >= INTERSECTION_COOLDOWN_MS)
        {
            intersectionActive = true;
            lastIntersectionTime = now;
            intersectionStraightUntil = now + INTERSECTION_STRAIGHT_MS;
            intersectionClearSince = 0;

            Serial.print(F("[TRACK] INTERSECTION CONFIRMED: "));
            Serial.print(totalStrong);
            Serial.println(F(" sensors -> STRAIGHT"));

            return true;
        }
    }
    else
    {
        intersectionWideSince = 0;
    }

    return false;
}

void driveStraightThroughIntersection()
{
    // Equal PWM is intentional here. The goal is to ignore the side
    // branch's sensor signal and physically cross the junction straight.
    // Keep the speed moderate so inertia does not carry the robot into
    // a branch before PID resumes.
    // No automatic speed reduction at intersections.
    // Use the configured base speed directly.
    const int intersectionSpeed =
        constrain(BASE_SPEED,
                  MIN_FORWARD_SPEED,
                  MAX_SPEED);

    motorAForward(intersectionSpeed);
    motorBForward(intersectionSpeed);

    lastMotorA = intersectionSpeed;
    lastMotorB = intersectionSpeed;
    haveLastDrive = true;
}


// ================================================================
// 25. SHARP / 90-DEGREE TURN
// ================================================================

bool runSharpTurn(float error, float derivative)
{
    const float absError = fabs(error);

    if (absError < SHARP_TURN_ERROR)
        return false;

    // A strong error indicates a 90-degree/very sharp turn.
    // Use maximum outer-wheel speed and minimum inner-wheel speed
    // to rotate aggressively onto the new path.
    float demand =
        (absError - SHARP_TURN_ERROR) /
        (SHARP_TURN_FULL_ERROR - SHARP_TURN_ERROR);

    demand = constrain(demand, 0.0, 1.0);

    // Maximum differential steering:
    // outer wheel stays at MAX/255 and inner wheel stays at MIN_FORWARD_SPEED.
    // This prevents the robot from appearing to stop at a 90-degree turn.
    int innerSpeed = SHARP_TURN_MIN_SPEED;
    int outerSpeed = SHARP_TURN_OUTER_SPEED;

    innerSpeed = constrain(innerSpeed,
                           MIN_FORWARD_SPEED,
                           MAX_SPEED);

    outerSpeed = constrain(outerSpeed,
                           MIN_FORWARD_SPEED,
                           MAX_SPEED);

    bool turnRight = (error > 0);
    if (INVERT_STEERING)
        turnRight = !turnRight;

    if (turnRight)
    {
        motorAForward(outerSpeed);
        motorBForward(innerSpeed);
    }
    else
    {
        motorAForward(innerSpeed);
        motorBForward(outerSpeed);
    }

    previousError = error;
    filteredDerivative = derivative;
    integral = 0.0;

    Serial.print(F("[TURN] SHARP "));
    Serial.print(turnRight ? F("RIGHT") : F("LEFT"));
    Serial.print(F(" ERR="));
    Serial.print(error, 0);
    Serial.print(F(" A="));
    Serial.print(turnRight ? outerSpeed : innerSpeed);
    Serial.print(F(" B="));
    Serial.println(turnRight ? innerSpeed : outerSpeed);

    return true;
}


// ================================================================
// 26. LINE RECOVERY
// ================================================================

const int RECOVERY_SPEED = 50;
const int RECOVERY_INNER_SPEED = 15;

void lineRecovery()
{
    /*
       IMPORTANT:
       The physical motor setup is forward-only. Never reverse a motor.

       Direction is latched from the LAST VALID line position rather than
       using an error of zero after the line disappears. This prevents the
       robot from randomly changing search direction and making a U-turn.
    */

    bool turnRight;

    if (lastLineDirection > 0)
        turnRight = true;
    else if (lastLineDirection < 0)
        turnRight = false;
    else
        turnRight = (lastValidPosition > CENTER_POSITION);

    if (INVERT_STEERING)
        turnRight = !turnRight;

    // For a short dashed-line gap, continue the previous steering arc.
    // After the gap timeout, reduce the recovery speed but keep the same
    // direction; do NOT alternate left/right.
    unsigned long lostFor =
        lineLostSince == 0 ? 0 : millis() - lineLostSince;

    int outer = (lostFor <= MAX_DASHED_GAP_MS)
                    ? RECOVERY_SPEED
                    : 35;

    int inner = RECOVERY_INNER_SPEED;

    if (turnRight)
    {
        motorAForward(outer);
        motorBForward(inner);
    }
    else
    {
        motorAForward(inner);
        motorBForward(outer);
    }

    lastMotorA = turnRight ? outer : inner;
    lastMotorB = turnRight ? inner : outer;
    haveLastDrive = true;
}


// ================================================================
// 25. PID CONTROL - V2
// ================================================================

void resetAdaptiveSpeed();

void resetPID()
{
    sharpTurnActive = false;
    previousError = 0.0;
    integral = 0.0;
    filteredDerivative = 0.0;
    resetAdaptiveSpeed();
    lastPIDTime = micros();
}


void printPIDSettings()
{
    Serial.println();
    Serial.println(F("================ PID SETTINGS ================"));

    Serial.print(F("Kp       = "));
    Serial.println(Kp, 4);

    Serial.print(F("Ki       = "));
    Serial.println(Ki, 4);

    Serial.print(F("Kd       = "));
    Serial.println(Kd, 4);

    Serial.print(F("Speed    = "));
    Serial.println(BASE_SPEED);

    Serial.print(F("MinSpeed = "));
    Serial.println(MIN_FORWARD_SPEED);

    Serial.print(F("MaxSpeed = "));
    Serial.println(MAX_SPEED);

    Serial.print(F("CurveMin = "));
    Serial.println(CURVE_MIN_SPEED);

    Serial.print(F("CurveErrStart = "));
    Serial.println(CURVE_ERROR_START, 1);

    Serial.print(F("CurveErrFull = "));
    Serial.println(CURVE_ERROR_FULL, 1);

    Serial.print(F("Sensor ON/OFF = "));
    Serial.print(LINE_THRESHOLD_ON);
    Serial.print(F("/"));
    Serial.println(LINE_THRESHOLD_OFF);

    Serial.print(F("PID dt   = "));
    Serial.print(PID_SAMPLE_US / 1000.0, 2);
    Serial.println(F(" ms"));

    Serial.print(F("D filter = "));
    Serial.println(derivativeFilterAlpha, 3);

    Serial.print(F("Run limit       = "));
    Serial.println(MAX_RUN_TIME_MS / 1000UL);
    Serial.println(F(" seconds"));

    Serial.print(F("Telemetry= "));
    Serial.println(telemetryEnabled ? F("ON") : F("OFF"));

    Serial.println(F("================================================"));
    Serial.println();
}


void printPIDHelp()
{
    Serial.println();
    Serial.println(F("================ PID COMMANDS ================="));
    Serial.println(F("KP 280      -> set Kp"));
    Serial.println(F("KI 0        -> set Ki"));
    Serial.println(F("KD 10       -> set Kd"));
    Serial.println(F("SPEED 40    -> set base speed"));
    Serial.println(F("MIN 5       -> set minimum forward speed"));
    Serial.println(F("MAX 80      -> set maximum speed"));
    Serial.println(F("DT 5        -> PID sample time in ms"));
    Serial.println(F("FILTER 0.25 -> derivative filter alpha"));
    Serial.println(F("TELEM ON    -> enable telemetry"));
    Serial.println(F("TELEM OFF   -> disable telemetry"));
    Serial.println(F("SHOW        -> show current settings"));
    Serial.println(F("RESET       -> reset PID memory"));
    Serial.println(F("HELP        -> show commands"));
    Serial.println(F("================================================"));
    Serial.println();
}


void processSerialCommand(char *command)
{
    char *token = strtok(command, " \t");

    if (token == nullptr)
        return;

    // ------------------------------------------------------------
    // KP
    // ------------------------------------------------------------

    if (strcasecmp(token, "KP") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            Kp = atof(token);
            Serial.print(F("[PID] Kp = "));
            Serial.println(Kp, 4);
            resetPID();
        }
        return;
    }

    // ------------------------------------------------------------
    // KI
    // ------------------------------------------------------------

    if (strcasecmp(token, "KI") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            Ki = atof(token);
            Serial.print(F("[PID] Ki = "));
            Serial.println(Ki, 4);
            resetPID();
        }
        return;
    }

    // ------------------------------------------------------------
    // KD
    // ------------------------------------------------------------

    if (strcasecmp(token, "KD") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            Kd = atof(token);
            Serial.print(F("[PID] Kd = "));
            Serial.println(Kd, 4);
            resetPID();
        }
        return;
    }

    // ------------------------------------------------------------
    // SPEED
    // ------------------------------------------------------------

    if (strcasecmp(token, "SPEED") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            BASE_SPEED = constrain(atoi(token), 0, 255);
            Serial.print(F("[PID] Base speed = "));
            Serial.println(BASE_SPEED);
        }
        return;
    }

    // ------------------------------------------------------------
    // MIN
    // ------------------------------------------------------------

    if (strcasecmp(token, "MIN") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            MIN_FORWARD_SPEED = constrain(atoi(token), 0, 255);
            Serial.print(F("[PID] Min speed = "));
            Serial.println(MIN_FORWARD_SPEED);
        }
        return;
    }

    // ------------------------------------------------------------
    // MAX
    // ------------------------------------------------------------

    if (strcasecmp(token, "MAX") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            MAX_SPEED = constrain(atoi(token), 0, 255);
            Serial.print(F("[PID] Max speed = "));
            Serial.println(MAX_SPEED);
        }
        return;
    }

    // ------------------------------------------------------------
    // DT
    // ------------------------------------------------------------

    if (strcasecmp(token, "DT") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            int dtMs = atoi(token);
            dtMs = constrain(dtMs, 2, 20);

            PID_SAMPLE_US = (unsigned long)dtMs * 1000UL;

            Serial.print(F("[PID] Sample time = "));
            Serial.print(dtMs);
            Serial.println(F(" ms"));

            resetPID();
        }
        return;
    }

    // ------------------------------------------------------------
    // FILTER
    // ------------------------------------------------------------

    if (strcasecmp(token, "FILTER") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            derivativeFilterAlpha = atof(token);
            derivativeFilterAlpha = constrain(derivativeFilterAlpha, 0.01, 1.0);

            Serial.print(F("[PID] D filter alpha = "));
            Serial.println(derivativeFilterAlpha, 3);

            resetPID();
        }
        return;
    }

    // ------------------------------------------------------------
    // TELEMETRY
    // ------------------------------------------------------------

    if (strcasecmp(token, "TELEM") == 0)
    {
        token = strtok(nullptr, " \t");

        if (token)
        {
            if (strcasecmp(token, "ON") == 0)
            {
                telemetryEnabled = true;
                Serial.println(F("[PID] Telemetry ON"));
            }
            else if (strcasecmp(token, "OFF") == 0)
            {
                telemetryEnabled = false;
                Serial.println(F("[PID] Telemetry OFF"));
            }
        }
        return;
    }

    // ------------------------------------------------------------
    // SHOW
    // ------------------------------------------------------------

    if (strcasecmp(token, "SHOW") == 0)
    {
        printPIDSettings();
        return;
    }

    // ------------------------------------------------------------
    // RESET
    // ------------------------------------------------------------

    if (strcasecmp(token, "RESET") == 0)
    {
        resetPID();
        Serial.println(F("[PID] PID memory reset"));
        return;
    }

    // ------------------------------------------------------------
    // HELP
    // ------------------------------------------------------------

    if (strcasecmp(token, "HELP") == 0)
    {
        printPIDHelp();
        return;
    }

    Serial.print(F("[PID] Unknown command: "));
    Serial.println(token);
    Serial.println(F("Type HELP"));
}


void processSerialInput()
{
    while (Serial.available() > 0)
    {
        char c = Serial.read();

        if (c == '\r' || c == '\n')
        {
            if (serialIndex > 0)
            {
                serialBuffer[serialIndex] = '\0';

                processSerialCommand(serialBuffer);

                serialIndex = 0;
            }
        }
        else
        {
            if (serialIndex < sizeof(serialBuffer) - 1)
            {
                serialBuffer[serialIndex++] = c;
            }
        }
    }
}


// ================================================================
// ADAPTIVE BASE SPEED
// ================================================================

int calculateCurveSpeed(float error, float derivative)
{
    float absError = fabs(error);
    float absDerivative = fabs(derivative);

    // 0 = straight, 1 = strong curve.
    float errorDemand = 0.0;
    if (absError > CURVE_ERROR_START)
    {
        errorDemand =
            (absError - CURVE_ERROR_START) /
            (CURVE_ERROR_FULL - CURVE_ERROR_START);
    }
    errorDemand = constrain(errorDemand, 0.0, 1.0);

    // A rapidly changing error means the robot is approaching a curve
    // quickly, so add a smaller anticipatory demand.
    float derivativeDemand =
        (absDerivative - CURVE_DERIVATIVE_START) /
        (CURVE_DERIVATIVE_FULL - CURVE_DERIVATIVE_START);
    derivativeDemand = constrain(derivativeDemand, 0.0, 1.0);

    float turnDemand =
        (errorDemand * (1.0 - DERIVATIVE_SPEED_WEIGHT)) +
        (derivativeDemand * DERIVATIVE_SPEED_WEIGHT);

    float targetSpeed =
        BASE_SPEED -
        ((BASE_SPEED - CURVE_MIN_SPEED) * turnDemand);

    targetSpeed = constrain(
        targetSpeed,
        (float)CURVE_MIN_SPEED,
        (float)BASE_SPEED
    );

    // Smooth speed changes. This keeps the robot stable when the line
    // briefly moves between two adjacent sensors.
    adaptiveBaseSpeed +=
        SPEED_SMOOTHING * (targetSpeed - adaptiveBaseSpeed);

    adaptiveBaseSpeed = constrain(
        adaptiveBaseSpeed,
        (float)CURVE_MIN_SPEED,
        (float)BASE_SPEED
    );

    return (int)(adaptiveBaseSpeed + 0.5);
}


void resetAdaptiveSpeed()
{
    adaptiveBaseSpeed = BASE_SPEED;
}


void runPID()
{
    // ------------------------------------------------------------
    // FIXED PID SAMPLE TIME
    // ------------------------------------------------------------

    unsigned long nowMicros = micros();

    if ((unsigned long)(nowMicros - lastPIDTime) < PID_SAMPLE_US)
        return;

    lastPIDTime = nowMicros;


    // One coherent sensor snapshot per PID cycle.
    sampleSensors();

    float position = 0;
    float error = 0;
    int curveSpeed = BASE_SPEED;

    bool lineFound = getLinePosition(position, error);

    // ============================================================
    // 90-DEGREE TURN DETECTOR -- HIGHEST PRIORITY
    // ============================================================
    // Detect the physical edge pattern BEFORE the special straight
    // pattern. At a 90-degree corner the weighted centroid can change
    // too quickly or disappear, so relying only on error is unreliable.
    // Once detected, hold the turn for a short time and continue turning
    // until the new line is reacquired.
    bool detectedTurnRight = false;

    if (!sharpTurnActive && detect90DegreeEdgeTurn(detectedTurnRight))
    {
        sharpTurnActive = true;
        sharpTurnRight = detectedTurnRight;
        sharpTurnStartMs = millis();

        Serial.print(F("[TURN] 90-DEGREE "));
        Serial.println(sharpTurnRight ? F("RIGHT") : F("LEFT"));
    }

    if (sharpTurnActive)
    {
        unsigned long turnAge = millis() - sharpTurnStartMs;

        drive90DegreeTurn(sharpTurnRight);

        // Never finish the turn during the initial hold.
        if (turnAge < SHARP_TURN_HOLD_MS)
            return;

        // After the hold, allow the turn to finish when the center of the
        // line is reacquired. A small error is considered centered.
        bool centeredAgain = lineFound && fabs(error) < 500.0;

        if (centeredAgain || turnAge >= SHARP_TURN_MAX_MS)
        {
            sharpTurnActive = false;
            resetPID();
            Serial.println(F("[TURN] 90-DEGREE COMPLETE"));
        }
        else
        {
            return;
        }
    }

    // ============================================================
    // SPECIAL TRACK PATTERN:
    // BOTH SIDES BLACK + CENTER GAP/WHITE
    // ============================================================
    if (isSideBlackCenterGapStraight())
    {
        driveSideBlackCenterGapStraight();
        return;
    }

    // Legacy error-based sharp turn for very large continuous error.
    if (runSharpTurn(error, filteredDerivative))
    {
        return;
    }

    // First finish an active straight-through crossing. The weighted
    // position is intentionally ignored while crossing because the
    // side branch makes the error misleading.
    if (intersectionStraightUntil != 0)
    {
        const unsigned long now = millis();
        const uint8_t strongCountNow = countStrongBlackSensors();

        if (strongCountNow >= INTERSECTION_MIN_SENSORS)
            intersectionClearSince = 0;
        else if (intersectionClearSince == 0)
            intersectionClearSince = now;

        if (now < intersectionStraightUntil ||
            (intersectionClearSince != 0 &&
             now - intersectionClearSince < INTERSECTION_CLEAR_MS))
        {
            driveStraightThroughIntersection();
            return;
        }

        intersectionStraightUntil = 0;
        intersectionClearSince = 0;
        intersectionActive = false;
        intersectionWideSince = 0;
        resetPID();
    }

    // Check for a new crossing only when the line itself is visible.
    // This prevents line-loss recovery from accidentally creating a
    // junction event.
    if (lineFound && detectIntersection(error))
    {
        driveStraightThroughIntersection();
        return;
    }


    // ------------------------------------------------------------
    // LINE LOST
    // ------------------------------------------------------------

    if (!lineFound)
    {
        if (state != STATE_LINE_LOST)
        {
            lineLostSince = millis();
            changeState(STATE_LINE_LOST);
        }

        // Do not allow integral/derivative memory to grow while
        // the line is unavailable.
        integral = 0.0;
        filteredDerivative = 0.0;

        lineRecovery();

        return;
    }


    // ------------------------------------------------------------
    // LINE FOUND AGAIN
    // ------------------------------------------------------------

    if (state == STATE_LINE_LOST)
    {
        Serial.println(F("[RECOVERY] LINE FOUND AGAIN"));

        lineLostSince = 0;
        resetPID();
        changeState(STATE_RUNNING);
    }


    // ------------------------------------------------------------
    // TIME-BASED INTEGRAL
    // ------------------------------------------------------------

    const float dt = PID_SAMPLE_US / 1000000.0;

    integral += error * dt;

    integral =
        constrain(integral,
                  -INTEGRAL_LIMIT,
                  INTEGRAL_LIMIT);


    // ------------------------------------------------------------
    // TIME-BASED DERIVATIVE
    // ------------------------------------------------------------

    float rawDerivative =
        (error - previousError) / dt;


    // Low-pass filter the derivative to reduce sensor noise.
    filteredDerivative =
        (derivativeFilterAlpha * rawDerivative)
        +
        ((1.0 - derivativeFilterAlpha) * filteredDerivative);


    // ------------------------------------------------------------
    // PID
    // ------------------------------------------------------------

    float pTerm =
        Kp * error / 1000.0;

    float iTerm =
        Ki * integral / 1000.0;

    float dTerm =
        Kd * filteredDerivative / 1000.0;

    float correction =
        pTerm + iTerm + dTerm;

    // Never let one noisy derivative sample demand more than the complete
    // available differential. The motors remain forward-only.
    correction = constrain(
        correction,
        -(float)(curveSpeed - MIN_FORWARD_SPEED),
        (float)(curveSpeed - MIN_FORWARD_SPEED));


    previousError = error;


    // ------------------------------------------------------------
    // DIFFERENTIAL DRIVE
    // ------------------------------------------------------------

    /*
       Positive error:
           line is toward sensor right.

       Motor A = faster
       Motor B = slower

       Negative error:
           Motor A = slower
           Motor B = faster

       If the robot physically turns in the wrong direction,
       invert correction by changing:

           BASE_SPEED + correction
           BASE_SPEED - correction

       to:

           BASE_SPEED - correction
           BASE_SPEED + correction
    */

    // ------------------------------------------------------------
    // ADAPTIVE SPEED
    // ------------------------------------------------------------
    // Speed is reduced by BOTH lateral error and how quickly the error is
    // changing. This lets the robot brake before a sharp curve instead of
    // discovering the curve after it has already overshot.
    curveSpeed = calculateCurveSpeed(error, filteredDerivative);

    int motorASpeed;
    int motorBSpeed;

    if (INVERT_STEERING)
    {
        motorASpeed = curveSpeed - correction;
        motorBSpeed = curveSpeed + correction;
    }
    else
    {
        motorASpeed = curveSpeed + correction;
        motorBSpeed = curveSpeed - correction;
    }


    // ------------------------------------------------------------
    // CONSTRAIN
    // ------------------------------------------------------------

    motorASpeed =
        constrain(motorASpeed,
                  MIN_FORWARD_SPEED,
                  MAX_SPEED);

    motorBSpeed =
        constrain(motorBSpeed,
                  MIN_FORWARD_SPEED,
                  MAX_SPEED);


    // ------------------------------------------------------------
    // DRIVE
    // ------------------------------------------------------------

    motorAForward(motorASpeed);
    motorBForward(motorBSpeed);

    // Remember the latest valid steering command so an intersection can
    // temporarily hold this heading instead of recalculating from the
    // ambiguous crossing pattern.
    lastSteeringCorrection = correction;
    lastMotorA = motorASpeed;
    lastMotorB = motorBSpeed;
    haveLastDrive = true;


    // ------------------------------------------------------------
    // SERIAL TELEMETRY
    // ------------------------------------------------------------

    if (telemetryEnabled &&
        millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)
    {
        lastTelemetryTime = millis();

        Serial.print(F("DATA"));
        Serial.print(F(",TIME="));
        Serial.print(millis());

        Serial.print(F(",POS="));
        Serial.print(position, 1);

        Serial.print(F(",ERR="));
        Serial.print(error, 1);

        Serial.print(F(",P="));
        Serial.print(pTerm, 2);

        Serial.print(F(",I="));
        Serial.print(iTerm, 2);

        Serial.print(F(",D="));
        Serial.print(dTerm, 2);

        Serial.print(F(",PID="));
        Serial.print(correction, 2);

        Serial.print(F(",A="));
        Serial.print(motorASpeed);

        Serial.print(F(",B="));
        Serial.print(motorBSpeed);

        Serial.print(F(",CURVEBASE="));
        Serial.print(curveSpeed);

        Serial.print(F(",BLACK="));
        Serial.print(countStrongBlackSensors());

        Serial.print(F(",CONF="));
        Serial.print(lineFound ? 1 : 0);

        Serial.print(F(",XING="));
        Serial.print(intersectionStraightUntil != 0 ? 1 : 0);

        Serial.print(F(",SIDEGAP="));
        Serial.print(isSideBlackCenterGapStraight() ? 1 : 0);

        Serial.print(F(",INV="));
        Serial.print(INVERT_STEERING ? 1 : 0);

        Serial.print(F(",TURN="));
        Serial.print(fabs(error) >= SHARP_TURN_ERROR ? 1 : 0);

        Serial.print(F(",VAC=ON"));

        Serial.println();
    }
}


// ================================================================
// 26. RUN CONTROL
// ================================================================

void startRunning(const __FlashStringHelper *reason)
{
    resetPID();
    runStartTime = millis();
    intersectionStraightUntil = 0;
    intersectionClearSince = 0;
    intersectionWideSince = 0;
    intersectionActive = false;
    lineLostSince = 0;
    lastLineDirection = 0;
    sensorFilterInitialized = false;

    // Clear hysteresis state so the new run starts from a fresh
    // sensor snapshot rather than stale BLACK states.
    for (uint8_t i = 0; i < 8; i++)
    {
        sensorFiltered[i] = 0;
        sensorBlack[i] = 0;
    }

    changeState(STATE_RUNNING);

    Serial.println();
    Serial.println(F("########################################"));
    Serial.println(F("                 RUN"));
    Serial.print(F("START REASON: "));
    Serial.println(reason);
    Serial.println(F("########################################"));

    motorAForward(BASE_SPEED);
    motorBForward(BASE_SPEED);
}


// ================================================================
// 27. SETUP
// ================================================================

void setup()
{
    Serial.begin(115200);

    serialIndex = 0;
    resetPID();


    // ------------------------------------------------------------
    // SENSOR PINS
    // ------------------------------------------------------------

    for (uint8_t i = 0; i < 8; i++)
        pinMode(SENSOR_PINS[i], INPUT);

    // ------------------------------------------------------------
    // MOTOR PINS
    // ------------------------------------------------------------

    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);


    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);


    // ------------------------------------------------------------
    // BUTTONS
    // ------------------------------------------------------------

    pinMode(LB, INPUT_PULLUP);
    pinMode(RB, INPUT_PULLUP);


    // ------------------------------------------------------------
    // LEDs
    // ------------------------------------------------------------

    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);


    // ------------------------------------------------------------
    // IMPELLER
    // ------------------------------------------------------------

    setupImpellerPWM();


    // ------------------------------------------------------------
    // SAFE STARTUP
    // ------------------------------------------------------------

    stopMotors();

    impellerOff();

    ledsOff();


    delay(500);


    // ------------------------------------------------------------
    // START MESSAGE
    // ------------------------------------------------------------

    Serial.println();
    Serial.println(F("=============================================="));
    Serial.println(F("       TECHGEEKS SPIRIT SUPERFAST LFR"));
    Serial.println(F("             PID CONTROLLER V4.1 SIDE-GAP STRAIGHT + IMPELLER OFF"));
    Serial.println(F("=============================================="));


    Serial.println();
    Serial.println(F("HARDWARE"));
    Serial.println(F("----------------------------------------------"));

    Serial.println(F("Arduino Nano       : READY"));
    Serial.println(F("ARC-8              : READY"));
    Serial.println(F("TB6612FNG          : READY"));
    Serial.println(F("Motor A            : READY"));
    Serial.println(F("Motor B            : READY"));
    Serial.println(F("Impeller Timer2    : DISABLED"));
    Serial.println(F("LB                 : READY"));
    Serial.println(F("RB                 : READY"));


    Serial.println();
    Serial.println(F("VERIFIED DRIVE"));
    Serial.println(F("----------------------------------------------"));

    Serial.println(F("Motor A D1         : FORWARD"));
    Serial.println(F("Motor B D1         : FORWARD"));


    Serial.println();
    Serial.println(F("SENSOR CALIBRATION"));
    Serial.println(F("----------------------------------------------"));

    for (uint8_t i = 0; i < 8; i++)
    {
        Serial.print(F("S"));
        Serial.print(i);

        Serial.print(F(" MIN="));
        Serial.print(sensorMin[i]);

        Serial.print(F(" MAX="));
        Serial.println(sensorMax[i]);
    }


    Serial.println();
    Serial.println(F("IMPELLER"));
    Serial.println(F("----------------------------------------------"));

    Serial.println(F("D11 / OC2A         : DISABLED"));
    Serial.println(F("Timer2             : DISABLED"));
    Serial.println(F("Frequency          : OFF"));
    Serial.println(F("Start duty         : 0/255"));


    Serial.println();
    Serial.println(F("BUTTON / START SEQUENCE"));
    Serial.println(F("----------------------------------------------"));

    Serial.println(F("LB SINGLE = START VACUUM"));
    Serial.println(F("LB DOUBLE-CLICK = 20s SENSOR CALIBRATION"));
    Serial.println(F("LB AFTER CALIBRATION = CONTINUE (IMPELLER DISABLED)"));
    Serial.println(F("RB    = START RUN"));
    Serial.println(F("RB DURING RUN = STOP"));
    Serial.println(F("RB DURING RUN = STOP"));
    Serial.println(F("RB AFTER STOP = RUN AGAIN"));
    Serial.println(F("NO START/END BOX LOGIC"));


    Serial.println();
    Serial.println(F("PID TUNING SERIAL COMMANDS"));
    Serial.println(F("----------------------------------------------"));
    Serial.println(F("KP 20 | KI 0 | KD 1"));
    Serial.println(F("SPEED 40 | MIN 5 | MAX 80"));
    Serial.println(F("DT 5 | FILTER 0.25"));
    Serial.println(F("TELEM ON/OFF | SHOW | RESET | HELP"));
    Serial.println(F("Open Serial Monitor at 115200 baud."));

    printPIDSettings();

    Serial.println();
    Serial.println(F("=============================================="));


    changeState(STATE_STARTUP);


    delay(1500);


    changeState(STATE_WAIT_CALIBRATION);
}


// ================================================================
// 28. MAIN LOOP
// ================================================================

void loop()
{
    // Continuous sensor diagnostics for Serial Monitor.
    // Open Serial Monitor at 115200 baud.
    printSensorReadings();

    processSerialInput();

    updateLEDs();


    // ============================================================
    // GLOBAL RB SAFETY
    // ============================================================

    /*
       RB is intentionally NOT treated as emergency stop
       before vacuum is ready.

       After vacuum is ready it starts the robot.

       During RUNNING, RB immediately stops the robot.
    */


    // ============================================================
    // DOUBLE-LB SENSOR CALIBRATION
    // ============================================================

    if (state == STATE_WAIT_CALIBRATION)
    {
        if (calibrationRunning)
        {
            updateSensorCalibration();
            return;
        }

        handleLBDoubleClick();

        // Calibration has finished. Both LEDs have already indicated
        // the result. Keep that indication visible for 2 seconds.
        if (calibrationFinished)
        {
            changeState(STATE_CALIBRATION_COMPLETE);

            delay(2000);

            calibrationFinished = false;

            if (calibrationValid)
            {
                // Valid calibration -> go directly to RUN-ready state.
                // The impeller is disabled in this version.
                changeState(STATE_VACUUM_READY);
            }
            else
            {
                // Invalid calibration -> return to calibration-ready state.
                Serial.println(F("CALIBRATION FAILED - DOUBLE-CLICK LB TO RETRY"));
                changeState(STATE_WAIT_CALIBRATION);
            }
        }

        return;
    }


    // ============================================================
    // WAITING FOR VACUUM START
    // ============================================================

    if (state == STATE_WAIT_IMPELLER)
    {
        // Compatibility state: impeller is disabled.
        changeState(STATE_VACUUM_READY);
        return;
    }


    // ============================================================
    // RB: START RUN
    // ============================================================

    if (state == STATE_VACUUM_READY)
    {
        if (rightButtonPressed())
        {
            Serial.println(F("[BUTTON] RB ACCEPTED - START"));
            startRunning(F("RB START"));
            return;
        }

        return;
    }


    // ============================================================
    // RUNNING
    // ============================================================

    if (state == STATE_RUNNING ||
        state == STATE_LINE_LOST)
    {
        // RB is an edge-triggered RUN/STOP toggle.
        // Holding RB does not repeatedly start/stop the robot.
        if (rightButtonPressed())
        {
            stopMotors();

            Serial.println();
            Serial.println(F("########################################"));
            Serial.println(F("              RB STOPPED"));
            Serial.println(F("########################################"));
            Serial.println(F("Press RB again to RUN again."));

            changeState(STATE_STOPPED);
            return;
        }

        // Official rulebook time limit: 180 seconds.
        if (millis() - runStartTime >= MAX_RUN_TIME_MS)
        {
            stopMotors();

            Serial.println();
            Serial.println(F("########################################"));
            Serial.println(F("            180s TIME LIMIT"));
            Serial.println(F("########################################"));

            changeState(STATE_STOPPED);
            return;
        }

        // No START/END black-box detection in V3.5.
        // RB is the only manual stop/restart control.

        runPID();
        return;
    }


    // ============================================================
    // STOPPED / RB RESTART
    // ============================================================

    if (state == STATE_STOPPED)
    {
        stopMotors();

        if (rightButtonPressed())
        {
            // Requested repeated cycle: RB -> RUN -> RB -> STOP.
            startRunning(F("RB RESTART"));
        }

        return;
    }
}