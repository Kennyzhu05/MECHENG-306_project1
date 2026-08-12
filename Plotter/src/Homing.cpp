#include <Arduino.h>

#include "Homing.h"
#include "Motor.h"
#include "Switch.h"


// =======================================================
// HOMING CONFIGURATION
// =======================================================

const int HOMING_PWM = 150;


// =======================================================
// HOMING STATES
// =======================================================

enum class HomingState
{
    NOT_HOMING,
    MOVING_BOTTOM_LEFT,
    MOVING_BOTTOM,
    MOVING_LEFT,
    COMPLETE
};


static HomingState homingState = HomingState::NOT_HOMING;


// =======================================================
// START HOMING
// =======================================================

void startHoming()
{
    // Make sure the latest switch states are available.
    updateLimitSwitches();

    // If we are already at the bottom-left position,
    // there is nothing to do.
    if (homeReached())
    {
        stopMotors();

        homingState = HomingState::COMPLETE;

        Serial.println("Homing: Already at HOME.");

        return;
    }


    Serial.println("Homing started.");

    // Start by moving diagonally toward bottom-left.
    moveBottomLeft(HOMING_PWM);

    homingState = HomingState::MOVING_BOTTOM_LEFT;
}


// =======================================================
// UPDATE HOMING
// =======================================================

void updateHoming()
{
    // Don't do anything if homing has not been started.
    if (homingState == HomingState::NOT_HOMING)
    {
        return;
    }


    // Don't keep commanding the motors after completion.
    if (homingState == HomingState::COMPLETE)
    {
        stopMotors();
        return;
    }


    // Read the physical switches.
    updateLimitSwitches();


    // ---------------------------------------------------
    // Check whether we have reached HOME.
    // ---------------------------------------------------

    if (homeReached())
    {
        stopMotors();

        homingState = HomingState::COMPLETE;

        Serial.println("Homing complete.");
        Serial.println("Position = Bottom Left.");

        return;
    }


    // ---------------------------------------------------
    // MOVING BOTTOM-LEFT
    // ---------------------------------------------------

    if (homingState == HomingState::MOVING_BOTTOM_LEFT)
    {
        bool leftReached = leftLimitReached();
        bool bottomReached = bottomLimitReached();


        // Both switches reached.
        if (leftReached && bottomReached)
        {
            stopMotors();

            homingState = HomingState::COMPLETE;

            Serial.println("Homing complete.");

            return;
        }


        // Left switch reached first.
        // Stop moving left and continue toward bottom.
        if (leftReached)
        {
            Serial.println("Left limit reached.");
            Serial.println("Continuing toward bottom.");

            moveBottom(HOMING_PWM);

            homingState = HomingState::MOVING_BOTTOM;

            return;
        }


        // Bottom switch reached first.
        // Stop moving bottom and continue toward left.
        if (bottomReached)
        {
            Serial.println("Bottom limit reached.");
            Serial.println("Continuing toward left.");

            moveLeft(HOMING_PWM);

            homingState = HomingState::MOVING_LEFT;

            return;
        }


        // Neither switch reached.
        // Continue moving toward bottom-left.
        moveBottomLeft(HOMING_PWM);

        return;
    }


    // ---------------------------------------------------
    // MOVING BOTTOM
    // ---------------------------------------------------

    if (homingState == HomingState::MOVING_BOTTOM)
    {
        // We already reached the left switch.
        // Now only the bottom switch needs to be reached.

        if (bottomLimitReached())
        {
            stopMotors();

            homingState = HomingState::COMPLETE;

            Serial.println("Homing complete.");

            return;
        }


        // Continue moving toward bottom.
        moveBottom(HOMING_PWM);

        return;
    }


    // ---------------------------------------------------
    // MOVING LEFT
    // ---------------------------------------------------

    if (homingState == HomingState::MOVING_LEFT)
    {
        // We already reached the bottom switch.
        // Now only the left switch needs to be reached.

        if (leftLimitReached())
        {
            stopMotors();

            homingState = HomingState::COMPLETE;

            Serial.println("Homing complete.");

            return;
        }


        // Continue moving toward left.
        moveLeft(HOMING_PWM);

        return;
    }
}


// =======================================================
// HOMING COMPLETE
// =======================================================

bool isHomingComplete()
{
    return homingState == HomingState::COMPLETE;
}


// =======================================================
// RESET HOMING
// =======================================================

void resetHoming()
{
    stopMotors();

    homingState = HomingState::NOT_HOMING;

    Serial.println("Homing reset.");
}