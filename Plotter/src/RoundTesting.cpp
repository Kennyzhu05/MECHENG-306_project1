#include "RoundTesting.h"
#include "Motor.h"
#include "Switch.h"

enum TestState
{
    MOVE_RIGHT,
    MOVE_UP,
    MOVE_LEFT,
    MOVE_DOWN
};

TestState testState = MOVE_RIGHT;

const int TEST_SPEED = 120;

void runSwitchTest()
{
    updateLimitSwitches();

    switch (testState)
    {
        case MOVE_RIGHT:
            moveRight(TEST_SPEED);

            if (rightLimitReached())
            {
                stopMotors();
                delay(300);
                testState = MOVE_UP;
            }
            break;

        case MOVE_UP:
            moveTop(TEST_SPEED);

            if (topLimitReached())
            {
                stopMotors();
                delay(300);
                testState = MOVE_LEFT;
            }
            break;

        case MOVE_LEFT:
            moveLeft(TEST_SPEED);

            if (leftLimitReached())
            {
                stopMotors();
                delay(300);
                testState = MOVE_DOWN;
            }
            break;

        case MOVE_DOWN:
            moveBottom(TEST_SPEED);

            if (bottomLimitReached())
            {
                stopMotors();
                delay(300);
                testState = MOVE_RIGHT;
            }
            break;
    }
}