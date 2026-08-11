#ifndef POSITION_H
#define POSITION_H

struct CartesianPosition
{
    float x_mm;
    float y_mm;
};

// Set experimentally determined encoder scale.
// These will be measured during a later hardware test.
bool configurePositionScale(
    float motor1CountsPerMm,
    float motor2CountsPerMm
);

// Define the current encoder position as machine zero.
void setPositionOrigin(
    long motor1Count,
    long motor2Count
);

// Calculate current X-Y position.
bool updatePosition(
    long motor1Count,
    long motor2Count
);

CartesianPosition getCurrentPosition();

float getCurrentX();
float getCurrentY();

bool positionScaleConfigured();

#endif