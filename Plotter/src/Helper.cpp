#include <Arduino.h>
#include <math.h>
#include "Helper.h"
#include "Motion.h"
#include "Motor.h"
#include "Encoder.h"
#include "Switch.h"

const float X_COUNTS_PER_MM = 20041.8 / 220.0;
const float Y_COUNTS_PER_MM = 12339.2 / 130.0;

void resetBottomLeft()
{
    Serial.println("Resetting to bottom-left corner...");

    // Go bottom
    while (true)
    {
        updateLimitSwitches();

        if (bottomLimitReached())
        {
            stopMotors();
            Serial.println("Bottom reached");
            break;
        }

        moveBottom(150);
    }

    delay(200);

    // Go left
    while (true)
    {
        updateLimitSwitches();

        if (leftLimitReached())
        {
            stopMotors();
            Serial.println("Left reached");
            break;
        }

        moveLeft(150);
    }

    stopMotors();

    Serial.println("Homed.");
}

long mmToXCounts(float mm)
{
    return (long)round(mm * X_COUNTS_PER_MM);
}

long mmToYCounts(float mm)
{
    return (long)round(mm * Y_COUNTS_PER_MM);
}

float xCountsToMm(long counts)
{
    return counts / X_COUNTS_PER_MM;
}

float yCountsToMm(long counts)
{
    return counts / Y_COUNTS_PER_MM;
}