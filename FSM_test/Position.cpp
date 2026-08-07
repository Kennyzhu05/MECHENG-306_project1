#include "Position.h"

// Latest calculated pen position
static CartesianPosition currentPosition = {0.0f, 0.0f};

// Encoder values recorded at the home position
static long motor1OriginCount = 0;
static long motor2OriginCount = 0;

// Temporary calibration values.
//
// These must be replaced later with measured or calculated
// counts-per-millimetre values.
static float motor1CountsPerMm = 1.0f;
static float motor2CountsPerMm = 1.0f;


// -------------------------------------------------------
// Configure encoder-to-distance conversion
// -------------------------------------------------------

void configurePositionScale(
    float newMotor1CountsPerMm,
    float newMotor2CountsPerMm
)
{
    // Prevent division by zero or invalid negative scales
    if (newMotor1CountsPerMm > 0.0f)
    {
        motor1CountsPerMm = newMotor1CountsPerMm;
    }

    if (newMotor2CountsPerMm > 0.0f)
    {
        motor2CountsPerMm = newMotor2CountsPerMm;
    }
}


// -------------------------------------------------------
// Establish the Cartesian origin
// -------------------------------------------------------

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


// -------------------------------------------------------
// Convert encoder counts into X-Y position
// -------------------------------------------------------

void updatePosition(
    long motor1Count,
    long motor2Count
)
{
    // Calculate the change in encoder counts since homing
    long motor1RelativeCount =
        motor1Count - motor1OriginCount;

    long motor2RelativeCount =
        motor2Count - motor2OriginCount;

    // Convert each motor's encoder movement into millimetres
    float motor1MovementMm =
        motor1RelativeCount / motor1CountsPerMm;

    float motor2MovementMm =
        motor2RelativeCount / motor2CountsPerMm;

    /*
       XY plotter kinematic conversion:

       Right:
           M1 positive, M2 positive

       Left:
           M1 negative, M2 negative

       Up:
           M1 positive, M2 negative

       Down:
           M1 negative, M2 positive
    */

    currentPosition.x_mm =
        0.5f * (motor1MovementMm + motor2MovementMm);

    currentPosition.y_mm =
        0.5f * (motor1MovementMm - motor2MovementMm);
}


// -------------------------------------------------------
// Position getters
// -------------------------------------------------------

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