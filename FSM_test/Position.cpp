#include "Position.h"

static CartesianPosition currentPosition = {0.0f, 0.0f};

static long motor1OriginCount = 0;
static long motor2OriginCount = 0;

// No fake default calibration value.
static float motor1CountsPerMm = 0.0f;
static float motor2CountsPerMm = 0.0f;

static bool configured = false;


bool configurePositionScale(
    float newMotor1CountsPerMm,
    float newMotor2CountsPerMm
)
{
    if (newMotor1CountsPerMm <= 0.0f ||
        newMotor2CountsPerMm <= 0.0f)
    {
        configured = false;
        return false;
    }

    motor1CountsPerMm = newMotor1CountsPerMm;
    motor2CountsPerMm = newMotor2CountsPerMm;

    configured = true;
    return true;
}


bool positionScaleConfigured()
{
    return configured;
}


void setPositionOrigin(
    long motor1Count,
    long motor2Count
)
{
    motor1OriginCount = motor1Count;
    motor2OriginCount = motor2Count;

    currentPosition.x_mm = 0.0f;
    currentPosition.y_mm = 0.0f;
}


bool updatePosition(
    long motor1Count,
    long motor2Count
)
{
    if (!configured)
    {
        return false;
    }

    long motor1RelativeCount =
        motor1Count - motor1OriginCount;

    long motor2RelativeCount =
        motor2Count - motor2OriginCount;


    // Convert encoder counts into motor displacement.
    float motor1MovementMm =
        motor1RelativeCount / motor1CountsPerMm;

    float motor2MovementMm =
        motor2RelativeCount / motor2CountsPerMm;


    /*
     * Course kinematics:
     *
     * A = left motor  = M2
     * B = right motor = M1
     *
     * ΔX = (ΔA + ΔB) / 2
     * ΔY = (ΔA - ΔB) / 2
     *
     * Therefore:
     *
     * X = (M2 + M1) / 2
     * Y = (M2 - M1) / 2
     */

    currentPosition.x_mm =
        0.5f * (motor2MovementMm +
                motor1MovementMm);

    currentPosition.y_mm =
        0.5f * (motor2MovementMm -
                motor1MovementMm);

    return true;
}


CartesianPosition getCurrentPosition()
{
    return currentPosition;
}


float getCurrentX()
{
    return currentPosition.x_mm;
}


float getCurrentY()
{
    return currentPosition.y_mm;
}