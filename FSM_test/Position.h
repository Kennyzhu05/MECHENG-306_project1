#ifndef POSITION_H
#define POSITION_H

// Cartesian position of the pen
struct CartesianPosition
{
    float x_mm;
    float y_mm;
};

// Set the encoder calibration values.
//
// These values describe how many encoder counts correspond
// to one millimetre of belt movement for each motor.
void configurePositionScale(
    float motor1CountsPerMm,
    float motor2CountsPerMm
);

// Set the current encoder counts as X = 0 mm, Y = 0 mm.
//
// This should normally be called after homing is complete.
void setPositionOrigin(
    long motor1Count,
    long motor2Count
);

// Calculate the current X-Y position using the latest
// encoder counts.
void updatePosition(
    long motor1Count,
    long motor2Count
);

// Return the latest calculated position.
CartesianPosition getCurrentPosition();

// Optional convenience functions.
float getCurrentX();
float getCurrentY();

#endif