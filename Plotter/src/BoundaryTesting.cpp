#include <Arduino.h>

#include "BoundaryTesting.h"
#include "Encoder.h"
#include "Homing.h"
#include "Motor.h"
#include "Switch.h"

namespace
{
// This is the same test speed used by the existing RoundTesting.cpp.
constexpr int TEST_PWM = 120;

// Stops a stage if a limit switch fails to trigger.
constexpr unsigned long STAGE_TIMEOUT_MS = 60000UL;

enum class BoundaryState
{
    NOT_STARTED,
    HOMING_FOR_TOP_TEST,
    MOVING_TO_TOP,
    HOMING_FOR_RIGHT_TEST,
    MOVING_TO_RIGHT,
    COMPLETE,
    FAULT
};

BoundaryState state = BoundaryState::NOT_STARTED;
unsigned long stageStartTime_ms = 0;

long topLeftCount = 0;
long topRightCount = 0;
long rightLeftCount = 0;
long rightRightCount = 0;

void startStageTimer()
{
    stageStartTime_ms = millis();
}

bool stageTimedOut()
{
    return millis() - stageStartTime_ms >= STAGE_TIMEOUT_MS;
}

void stopWithFault(const __FlashStringHelper* reason)
{
    stopMotors();
    Serial.println();
    Serial.println(F("BOUNDARY TEST STOPPED"));
    Serial.println(reason);
    state = BoundaryState::FAULT;
}

void printTopCounts()
{
    Serial.println();
    Serial.println(F("===== TOP BOUNDARY REACHED ====="));
    Serial.print(F("Left encoder count, origin to top:  "));
    Serial.println(topLeftCount);
    Serial.print(F("Right encoder count, origin to top: "));
    Serial.println(topRightCount);
    Serial.println(F("================================"));
}

void printRightCounts()
{
    Serial.println();
    Serial.println(F("===== RIGHT BOUNDARY REACHED ====="));
    Serial.print(F("Left encoder count, origin to right:  "));
    Serial.println(rightLeftCount);
    Serial.print(F("Right encoder count, origin to right: "));
    Serial.println(rightRightCount);
    Serial.println(F("=================================="));
}
} // namespace

void beginBoundaryTest()
{
    setupMotors();
    setupSwitches();
    setupEncoders();
    stopMotors();

    Serial.println(F("Starting plotter boundary test."));
    Serial.println(F("Homing to the bottom-left origin for the top test."));

    resetHoming();
    startHoming();
    startStageTimer();
    state = BoundaryState::HOMING_FOR_TOP_TEST;
}

void updateBoundaryTest()
{
    if (state == BoundaryState::NOT_STARTED)
    {
        return;
    }

    // Keep the measured switch states and encoder counts current.
    updateLimitSwitches();
    updateEncoders();

    switch (state)
    {
        case BoundaryState::HOMING_FOR_TOP_TEST:
            updateHoming();

            if (isHomingComplete())
            {
                stopMotors();
                resetEncoders();

                Serial.println(F("Origin reached; encoder counts reset to zero."));
                Serial.println(F("Moving straight up."));

                startStageTimer();
                state = BoundaryState::MOVING_TO_TOP;
            }
            else if (stageTimedOut())
            {
                stopWithFault(F("Timeout while homing for the top test."));
            }
            break;

        case BoundaryState::MOVING_TO_TOP:
            // Test the switch before issuing another movement command. This
            // stops both motors on the first loop that detects the top switch.
            if (topLimitReached())
            {
                stopMotors();
                updateEncoders();

                topLeftCount = getLeftEncoderCount();
                topRightCount = getRightEncoderCount();
                printTopCounts();

                Serial.println(F("Homing again for the right-boundary test."));
                resetHoming();
                startHoming();
                startStageTimer();
                state = BoundaryState::HOMING_FOR_RIGHT_TEST;
            }
            else if (stageTimedOut())
            {
                stopWithFault(F("Top switch did not trigger before timeout."));
            }
            else
            {
                moveTop(TEST_PWM);
            }
            break;

        case BoundaryState::HOMING_FOR_RIGHT_TEST:
            updateHoming();

            if (isHomingComplete())
            {
                stopMotors();
                resetEncoders();

                Serial.println(F("Origin reached; encoder counts reset to zero."));
                Serial.println(F("Moving straight right."));

                startStageTimer();
                state = BoundaryState::MOVING_TO_RIGHT;
            }
            else if (stageTimedOut())
            {
                stopWithFault(F("Timeout while homing for the right test."));
            }
            break;

        case BoundaryState::MOVING_TO_RIGHT:
            if (rightLimitReached())
            {
                stopMotors();
                updateEncoders();

                rightLeftCount = getLeftEncoderCount();
                rightRightCount = getRightEncoderCount();
                printRightCounts();

                Serial.println();
                Serial.println(F("Boundary test complete. Motors stopped."));
                state = BoundaryState::COMPLETE;
            }
            else if (stageTimedOut())
            {
                stopWithFault(F("Right switch did not trigger before timeout."));
            }
            else
            {
                moveRight(TEST_PWM);
            }
            break;

        case BoundaryState::COMPLETE:
        case BoundaryState::FAULT:
            stopMotors();
            break;

        case BoundaryState::NOT_STARTED:
            break;
    }
}

bool isBoundaryTestComplete()
{
    return state == BoundaryState::COMPLETE;
}

bool hasBoundaryTestFault()
{
    return state == BoundaryState::FAULT;
}
