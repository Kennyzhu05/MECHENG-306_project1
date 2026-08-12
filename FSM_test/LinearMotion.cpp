#include <Arduino.h>
#include <math.h>

#include "LinearMotion.h"


// =======================================================
// STORED COMMAND
// =======================================================

static MotionCommand activeCommand = {0.0f, 0.0f};

static bool commandAvailable = false;


// =======================================================
// TRAJECTORY REFERENCE
// =======================================================

static MotionReference reference = {
    0.0f,
    0.0f,
    0.0f,
    0.0f
};


// Starting X-Y position for the current move
static float startX_mm = 0.0f;
static float startY_mm = 0.0f;


// Required Cartesian displacement
static float deltaX_mm = 0.0f;
static float deltaY_mm = 0.0f;


// Total straight-line distance
static float pathLength_mm = 0.0f;


// =======================================================
// CONFIGURATION
// =======================================================

static float linearSpeed_mm_s = 0.0f;
static float positionTolerance_mm = 0.0f;

static unsigned long timeoutMargin_ms = 0;

static bool configured = false;


// =======================================================
// TIMING
// =======================================================

static unsigned long moveStartTime_ms = 0;

static unsigned long plannedMoveTime_ms = 0;


// =======================================================
// STATUS
// =======================================================

static bool moveActive = false;

static bool moveComplete = false;

static bool moveTimedOut = false;


// =======================================================
// CONFIGURATION FUNCTION
// =======================================================

bool configureLinearMotion(
    float speed_mm_s,
    float tolerance_mm,
    unsigned long newTimeoutMargin_ms
)
{
    // Reject invalid configuration
    if (speed_mm_s <= 0.0f)
    {
        configured = false;
        return false;
    }

    if (tolerance_mm <= 0.0f)
    {
        configured = false;
        return false;
    }

    linearSpeed_mm_s = speed_mm_s;

    positionTolerance_mm = tolerance_mm;

    timeoutMargin_ms = newTimeoutMargin_ms;

    configured = true;

    return true;
}


// =======================================================
// STORE G1 TARGET
// =======================================================

void setLinearTarget(
    float targetX_mm,
    float targetY_mm
)
{
    activeCommand.targetX_mm = targetX_mm;

    activeCommand.targetY_mm = targetY_mm;

    commandAvailable = true;
}


MotionCommand getLinearTarget()
{
    return activeCommand;
}


bool linearTargetAvailable()
{
    return commandAvailable;
}


void clearLinearTarget()
{
    commandAvailable = false;
}


// =======================================================
// START A LINEAR MOVE
// =======================================================

bool startLinearMove(
    float currentX_mm,
    float currentY_mm
)
{
    // Motion generator must first be configured.
    if (!configured)
    {
        return false;
    }

    // A G1 target must already have been stored.
    if (!commandAvailable)
    {
        return false;
    }


    // Store where this movement begins.
    startX_mm = currentX_mm;

    startY_mm = currentY_mm;


    // Calculate required displacement.
    deltaX_mm =
        activeCommand.targetX_mm - startX_mm;

    deltaY_mm =
        activeCommand.targetY_mm - startY_mm;


    // Straight-line distance from start to target.
    pathLength_mm =
        sqrt(
            deltaX_mm * deltaX_mm +
            deltaY_mm * deltaY_mm
        );


    // Reset status from any previous move.
    moveComplete = false;

    moveTimedOut = false;


    // ---------------------------------------------------
    // Target is already reached
    // ---------------------------------------------------

    if (pathLength_mm <= positionTolerance_mm)
    {
        reference.x_mm =
            activeCommand.targetX_mm;

        reference.y_mm =
            activeCommand.targetY_mm;


        /*
         * Platform kinematics:
         *
         * A = M2 = X + Y
         * B = M1 = X - Y
         */

        reference.motor2_mm =
            reference.x_mm +
            reference.y_mm;

        reference.motor1_mm =
            reference.x_mm -
            reference.y_mm;


        moveActive = false;

        moveComplete = true;

        return true;
    }


    // ---------------------------------------------------
    // Calculate planned motion duration
    // ---------------------------------------------------

    float moveTime_s =
        pathLength_mm / linearSpeed_mm_s;

    plannedMoveTime_ms =
        (unsigned long)(moveTime_s * 1000.0f);


    // Prevent a zero-duration move caused by rounding.
    if (plannedMoveTime_ms == 0)
    {
        plannedMoveTime_ms = 1;
    }


    moveStartTime_ms = millis();

    moveActive = true;


    // Start trajectory exactly at current position.
    reference.x_mm = startX_mm;

    reference.y_mm = startY_mm;


    reference.motor2_mm =
        reference.x_mm +
        reference.y_mm;

    reference.motor1_mm =
        reference.x_mm -
        reference.y_mm;


    return true;
}


// =======================================================
// UPDATE THE LINEAR TRAJECTORY
// =======================================================

void updateLinearMove(
    float currentX_mm,
    float currentY_mm
)
{
    if (!moveActive)
    {
        return;
    }


    unsigned long elapsedTime_ms =
        millis() - moveStartTime_ms;


    // ---------------------------------------------------
    // Calculate progress along the commanded straight line
    // ---------------------------------------------------

    float progress =
        (float)elapsedTime_ms /
        (float)plannedMoveTime_ms;


    // Do not allow reference to go past target.
    if (progress > 1.0f)
    {
        progress = 1.0f;
    }


    /*
     * Linear interpolation:
     *
     * Xref = Xstart + progress * deltaX
     * Yref = Ystart + progress * deltaY
     *
     * Both coordinates therefore advance using the SAME
     * progress variable, producing a straight-line reference.
     */

    reference.x_mm =
        startX_mm +
        progress * deltaX_mm;

    reference.y_mm =
        startY_mm +
        progress * deltaY_mm;


    // ---------------------------------------------------
    // Convert Cartesian reference to motor coordinates
    // ---------------------------------------------------

    /*
     * From supplied kinematics:
     *
     * A = X + Y
     * B = X - Y
     *
     * For our motor mapping:
     *
     * A = M2
     * B = M1
     */

    reference.motor2_mm =
        reference.x_mm +
        reference.y_mm;

    reference.motor1_mm =
        reference.x_mm -
        reference.y_mm;


    // ---------------------------------------------------
    // Wait until reference reaches final target
    // ---------------------------------------------------

    if (progress < 1.0f)
    {
        return;
    }


    // ---------------------------------------------------
    // Check actual position error
    // ---------------------------------------------------

    float errorX_mm =
        activeCommand.targetX_mm -
        currentX_mm;

    float errorY_mm =
        activeCommand.targetY_mm -
        currentY_mm;


    float positionError_mm =
        sqrt(
            errorX_mm * errorX_mm +
            errorY_mm * errorY_mm
        );


    // Actual machine has reached the target.
    if (positionError_mm <= positionTolerance_mm)
    {
        moveActive = false;

        moveComplete = true;

        return;
    }


    // ---------------------------------------------------
    // Timeout
    // ---------------------------------------------------

    if (elapsedTime_ms >
        plannedMoveTime_ms + timeoutMargin_ms)
    {
        moveActive = false;

        moveTimedOut = true;
    }
}


// =======================================================
// GET CURRENT REFERENCE
// =======================================================

MotionReference getLinearReference()
{
    return reference;
}


// =======================================================
// STATUS FUNCTIONS
// =======================================================

bool linearMoveActive()
{
    return moveActive;
}


bool linearMoveComplete()
{
    return moveComplete;
}


bool linearMoveTimedOut()
{
    return moveTimedOut;
}