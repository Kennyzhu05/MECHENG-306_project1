#include <Arduino.h>
#include "Motor.h"
#include "Encoder.h"

/*
  BoundaryTest.ino

  Purpose:
  1. Home X to the LEFT limit switch.
  2. Home Y to the BOTTOM limit switch.
  3. Reset encoders at the bottom-left corner.
  4. Travel RIGHT until the right switch is triggered and record X-span encoder counts.
  5. Return LEFT.
  6. Reset encoders again.
  7. Travel UP until the top switch is triggered and record Y-span encoder counts.
  8. Return BOTTOM.
  9. Print the measured X and Y encoder spans.

  IMPORTANT:
  - Replace the four limit-switch pin numbers below with the pins used by your hardware.
  - The code assumes a pressed switch reads LOW using INPUT_PULLUP.
    If your existing switch test shows the opposite, change SWITCH_ACTIVE_STATE to HIGH.
  - The direction helper functions follow the supplied ME306 motor-direction table:
      Right:  Motor A +, Motor B +
      Left:   Motor A -, Motor B -
      Up:     Motor A -, Motor B +
      Down:   Motor A +, Motor B -
    If your driveMotorA()/driveMotorB() sign convention is reversed, change the signs
    in moveRightSlow(), moveLeftSlow(), moveUpSlow(), and moveDownSlow().
*/

// -----------------------------------------------------------------------------
// USER SETTINGS
// -----------------------------------------------------------------------------

// TODO: replace these with the actual Arduino Mega pins used for the four switches.
const uint8_t LEFT_LIMIT_PIN   = 22;   // CHANGE THIS
const uint8_t RIGHT_LIMIT_PIN  = 23;   // CHANGE THIS
const uint8_t TOP_LIMIT_PIN    = 24;   // CHANGE THIS
const uint8_t BOTTOM_LIMIT_PIN = 25;   // CHANGE THIS

// Common wiring arrangement: INPUT_PULLUP -> pressed switch reads LOW.
// Change to HIGH if your actual switch test shows pressed = HIGH.
const uint8_t SWITCH_ACTIVE_STATE = LOW;

// Use a deliberately low speed for boundary testing.
const int TEST_PWM = 70;

// Debounce and safety timeout.
const unsigned long DEBOUNCE_MS = 20;
const unsigned long STATE_TIMEOUT_MS = 15000;

// -----------------------------------------------------------------------------
// SIMPLE NON-BLOCKING SWITCH DEBOUNCE
// -----------------------------------------------------------------------------

struct DebouncedSwitch
{
    uint8_t pin;
    bool rawPressed;
    bool stablePressed;
    unsigned long lastChangeTime;
};

DebouncedSwitch leftSwitch   = {LEFT_LIMIT_PIN, false, false, 0};
DebouncedSwitch rightSwitch  = {RIGHT_LIMIT_PIN, false, false, 0};
DebouncedSwitch topSwitch    = {TOP_LIMIT_PIN, false, false, 0};
DebouncedSwitch bottomSwitch = {BOTTOM_LIMIT_PIN, false, false, 0};

bool readPressed(uint8_t pin)
{
    return digitalRead(pin) == SWITCH_ACTIVE_STATE;
}

void updateOneSwitch(DebouncedSwitch &sw)
{
    bool newRawPressed = readPressed(sw.pin);
    unsigned long now = millis();

    if (newRawPressed != sw.rawPressed)
    {
        sw.rawPressed = newRawPressed;
        sw.lastChangeTime = now;
    }

    if ((now - sw.lastChangeTime) >= DEBOUNCE_MS)
    {
        sw.stablePressed = sw.rawPressed;
    }
}

void updateSwitches()
{
    updateOneSwitch(leftSwitch);
    updateOneSwitch(rightSwitch);
    updateOneSwitch(topSwitch);
    updateOneSwitch(bottomSwitch);
}

// -----------------------------------------------------------------------------
// DIRECTION HELPERS
// -----------------------------------------------------------------------------

void moveRightSlow()
{
    driveMotorA(+TEST_PWM);
    driveMotorB(+TEST_PWM);
}

void moveLeftSlow()
{
    driveMotorA(-TEST_PWM);
    driveMotorB(-TEST_PWM);
}

void moveUpSlow()
{
    driveMotorA(-TEST_PWM);
    driveMotorB(+TEST_PWM);
}

void moveDownSlow()
{
    driveMotorA(+TEST_PWM);
    driveMotorB(-TEST_PWM);
}

// -----------------------------------------------------------------------------
// TEST STATE MACHINE
// -----------------------------------------------------------------------------

enum TestState
{
    WAITING_TO_START,
    HOMING_X_LEFT,
    HOMING_Y_BOTTOM,
    TESTING_X_RIGHT,
    RETURNING_X_LEFT,
    TESTING_Y_UP,
    RETURNING_Y_BOTTOM,
    TEST_COMPLETE,
    TEST_FAULT
};

TestState state = WAITING_TO_START;
unsigned long stateStartTime = 0;

long measuredXSpanCounts = 0;
long measuredYSpanCounts = 0;

void enterState(TestState newState)
{
    stopMotors();
    state = newState;
    stateStartTime = millis();

    switch (state)
    {
        case HOMING_X_LEFT:
            Serial.println(F("Homing X: moving LEFT..."));
            break;

        case HOMING_Y_BOTTOM:
            Serial.println(F("Homing Y: moving DOWN..."));
            break;

        case TESTING_X_RIGHT:
            Serial.println(F("Testing X: moving RIGHT until right limit..."));
            break;

        case RETURNING_X_LEFT:
            Serial.println(F("Returning X: moving LEFT until left limit..."));
            break;

        case TESTING_Y_UP:
            Serial.println(F("Testing Y: moving UP until top limit..."));
            break;

        case RETURNING_Y_BOTTOM:
            Serial.println(F("Returning Y: moving DOWN until bottom limit..."));
            break;

        case TEST_COMPLETE:
            Serial.println();
            Serial.println(F("===== BOUNDARY TEST COMPLETE ====="));
            Serial.print(F("Measured X span = "));
            Serial.print(measuredXSpanCounts);
            Serial.println(F(" encoder-coordinate counts"));

            Serial.print(F("Measured Y span = "));
            Serial.print(measuredYSpanCounts);
            Serial.println(F(" encoder-coordinate counts"));
            Serial.println(F("=================================="));
            break;

        case TEST_FAULT:
            Serial.println();
            Serial.println(F("FAULT: movement timed out. Motors stopped."));
            Serial.println(F("Check motor direction, switch wiring, encoder wiring, or mechanical blockage."));
            break;

        default:
            break;
    }
}

bool stateTimedOut()
{
    return (millis() - stateStartTime) >= STATE_TIMEOUT_MS;
}

// -----------------------------------------------------------------------------
// ARDUINO SETUP
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
    pinMode(TOP_LIMIT_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_LIMIT_PIN, INPUT_PULLUP);

    stopMotors();

    Serial.println();
    Serial.println(F("ME306 XY Plotter Boundary Test"));
    Serial.println(F("--------------------------------"));
    Serial.println(F("Before starting, verify the four switch pin numbers."));
    Serial.println(F("The test will move slowly to all four boundaries."));
    Serial.println(F("Send 's' in Serial Monitor to start."));
    Serial.println(F("Send 'q' at any time to stop."));
}

// -----------------------------------------------------------------------------
// MAIN NON-BLOCKING LOOP
// -----------------------------------------------------------------------------

void loop()
{
    updateEncoders();
    updateSwitches();

    // Manual emergency stop from Serial Monitor.
    if (Serial.available())
    {
        char command = Serial.read();

        if (command == 'q' || command == 'Q')
        {
            stopMotors();
            enterState(TEST_FAULT);
            return;
        }

        if ((command == 's' || command == 'S') && state == WAITING_TO_START)
        {
            enterState(HOMING_X_LEFT);
        }
    }

    // Safety timeout for every moving state.
    if (state != WAITING_TO_START &&
        state != TEST_COMPLETE &&
        state != TEST_FAULT &&
        stateTimedOut())
    {
        enterState(TEST_FAULT);
        return;
    }

    switch (state)
    {
        case WAITING_TO_START:
            stopMotors();
            break;

        // 1) Find X = 0 using the left limit switch.
        case HOMING_X_LEFT:
            if (leftSwitch.stablePressed)
            {
                stopMotors();
                Serial.println(F("Left limit reached -> X home found."));
                enterState(HOMING_Y_BOTTOM);
            }
            else
            {
                moveLeftSlow();
            }
            break;

        // 2) Find Y = 0 using the bottom limit switch.
        case HOMING_Y_BOTTOM:
            if (bottomSwitch.stablePressed)
            {
                stopMotors();
                Serial.println(F("Bottom limit reached -> Y home found."));
                resetEncoders();
                Serial.println(F("Encoders reset at bottom-left machine zero."));
                enterState(TESTING_X_RIGHT);
            }
            else
            {
                moveDownSlow();
            }
            break;

        // 3) Move across the full X range.
        case TESTING_X_RIGHT:
            if (rightSwitch.stablePressed)
            {
                stopMotors();

                long encoderA = getLeftEncoderCount();
                long encoderB = getRightEncoderCount();

                // From the supplied kinematics:
                // X = (A + B) / 2
                measuredXSpanCounts = labs((encoderA + encoderB) / 2);

                Serial.println(F("Right limit reached."));
                Serial.print(F("Encoder A = "));
                Serial.println(encoderA);
                Serial.print(F("Encoder B = "));
                Serial.println(encoderB);
                Serial.print(F("Calculated full X span = "));
                Serial.println(measuredXSpanCounts);

                enterState(RETURNING_X_LEFT);
            }
            else
            {
                moveRightSlow();
            }
            break;

        // 4) Return to X = 0 before testing Y.
        case RETURNING_X_LEFT:
            if (leftSwitch.stablePressed)
            {
                stopMotors();
                Serial.println(F("Returned to left limit."));
                resetEncoders();
                Serial.println(F("Encoders reset before Y test."));
                enterState(TESTING_Y_UP);
            }
            else
            {
                moveLeftSlow();
            }
            break;

        // 5) Move across the full Y range.
        case TESTING_Y_UP:
            if (topSwitch.stablePressed)
            {
                stopMotors();

                long encoderA = getLeftEncoderCount();
                long encoderB = getRightEncoderCount();

                // From the supplied kinematics:
                // Y = (A - B) / 2
                measuredYSpanCounts = labs((encoderA - encoderB) / 2);

                Serial.println(F("Top limit reached."));
                Serial.print(F("Encoder A = "));
                Serial.println(encoderA);
                Serial.print(F("Encoder B = "));
                Serial.println(encoderB);
                Serial.print(F("Calculated full Y span = "));
                Serial.println(measuredYSpanCounts);

                enterState(RETURNING_Y_BOTTOM);
            }
            else
            {
                moveUpSlow();
            }
            break;

        // 6) Return to Y = 0 so all four boundaries have been exercised.
        case RETURNING_Y_BOTTOM:
            if (bottomSwitch.stablePressed)
            {
                stopMotors();
                Serial.println(F("Returned to bottom limit."));
                enterState(TEST_COMPLETE);
            }
            else
            {
                moveDownSlow();
            }
            break;

        case TEST_COMPLETE:
            stopMotors();
            break;

        case TEST_FAULT:
            stopMotors();
            break;
    }
}
