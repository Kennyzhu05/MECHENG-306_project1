#include <Arduino.h>
#include <math.h>

#include "Motion.h"
#include "Motor.h"
#include "Encoder.h"


// =======================================================
// PI CONTROL SETTINGS
// =======================================================

float KP = 0.8f;
float KI = 0.05f;

// Corrects one motor when its proportional progress is ahead
// of the other motor. Start conservatively and tune experimentally.
float SYNC_KP = 0.5f;

int MIN_PWM = 60;
int MAX_PWM = 255;

long POSITION_TOLERANCE = 5;
int SETTLE_SAMPLES = 5;

unsigned long MOVE_TIMEOUT_MS = 8000;


// =======================================================
// INDIVIDUAL MOTOR CALIBRATION
// =======================================================

/*
 * Boundary-test results:
 *
 * Rightward movement over 220 mm:
 * Left motor  = 20276.4 counts
 * Right motor = 19807.8 counts
 *
 * Upward movement over 130 mm:
 * Left motor  = 12393.6 counts
 * Right motor = -12285.6 counts
 */

const float LEFT_X_COUNTS_PER_MM =
    20276.4f / 220.0f;

const float RIGHT_X_COUNTS_PER_MM =
    19807.8f / 220.0f;

const float LEFT_Y_COUNTS_PER_MM =
    12393.6f / 130.0f;

const float RIGHT_Y_COUNTS_PER_MM =
    12285.6f / 130.0f;


// =======================================================
// HELPER FUNCTIONS
// =======================================================

static int clampInt(int value, int minimum, int maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static long absoluteLong(long value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}


static float motorDirection(long target)
{
    if (target > 0)
    {
        return 1.0f;
    }

    if (target < 0)
    {
        return -1.0f;
    }

    return 0.0f;
}


static int toMotorCommand(float controllerOutput)
{
    int pwm = static_cast<int>(controllerOutput);

    pwm = clampInt(pwm, -MAX_PWM, MAX_PWM);

    if (pwm > 0 && pwm < MIN_PWM)
    {
        pwm = MIN_PWM;
    }
    else if (pwm < 0 && pwm > -MIN_PWM)
    {
        pwm = -MIN_PWM;
    }

    return pwm;
}


// =======================================================
// CLOSED-LOOP MOTOR MOVEMENT
// =======================================================

static void moveMotorTargets(
    long targetA,
    long targetB
)
{
    resetEncoders();

    float integralA = 0.0f;
    float integralB = 0.0f;

    unsigned long startTime = millis();
    unsigned long lastTime = startTime;

    int settledCount = 0;
    bool timedOut = false;

    while (true)
    {
        updateEncoders();

        long measuredA = getLeftEncoderCount();
        long measuredB = getRightEncoderCount();

        long errorA = targetA - measuredA;
        long errorB = targetB - measuredB;

        unsigned long now = millis();

        float dt =
            static_cast<float>(now - lastTime) / 1000.0f;

        if (dt <= 0.0f)
        {
            dt = 0.001f;
        }

        lastTime = now;


        // -----------------------------------------------
        // Normal PI position control
        // -----------------------------------------------

        float outputA =
            KP * static_cast<float>(errorA) +
            KI * integralA;

        float outputB =
            KP * static_cast<float>(errorB) +
            KI * integralB;


        // -----------------------------------------------
        // Coordinated-motion synchronization
        // -----------------------------------------------

        /*
         * For a straight path, both motors should complete
         * the same proportion of their movements at the
         * same time.
         *
         * For example:
         *
         * progressA = 0.50
         * progressB = 0.45
         *
         * means Motor A is ahead of Motor B.
         */

        long targetMagnitudeA = absoluteLong(targetA);
        long targetMagnitudeB = absoluteLong(targetB);

        if (targetMagnitudeA > POSITION_TOLERANCE &&
            targetMagnitudeB > POSITION_TOLERANCE)
        {
            float progressA =
                static_cast<float>(measuredA) /
                static_cast<float>(targetA);

            float progressB =
                static_cast<float>(measuredB) /
                static_cast<float>(targetB);

            float progressDifference =
                progressA - progressB;

            long smallerTargetMagnitude;

            if (targetMagnitudeA < targetMagnitudeB)
            {
                smallerTargetMagnitude = targetMagnitudeA;
            }
            else
            {
                smallerTargetMagnitude = targetMagnitudeB;
            }

            /*
             * Convert the normalized progress difference
             * into an approximate count difference.
             */
            float synchronizationErrorCounts =
                progressDifference *
                static_cast<float>(smallerTargetMagnitude);

            float synchronizationOutput =
                SYNC_KP * synchronizationErrorCounts;

            /*
             * If A is ahead:
             *
             * - reduce A in its movement direction;
             * - increase B in its movement direction.
             *
             * motorDirection() makes this work even when
             * one motor has a negative target.
             */
            outputA -=
                motorDirection(targetA) *
                synchronizationOutput;

            outputB +=
                motorDirection(targetB) *
                synchronizationOutput;
        }


        // -----------------------------------------------
        // Anti-windup
        // -----------------------------------------------

        bool saturatedPositiveA =
            outputA > MAX_PWM && errorA > 0;

        bool saturatedNegativeA =
            outputA < -MAX_PWM && errorA < 0;

        if (!saturatedPositiveA &&
            !saturatedNegativeA)
        {
            integralA +=
                static_cast<float>(errorA) * dt;
        }


        bool saturatedPositiveB =
            outputB > MAX_PWM && errorB > 0;

        bool saturatedNegativeB =
            outputB < -MAX_PWM && errorB < 0;

        if (!saturatedPositiveB &&
            !saturatedNegativeB)
        {
            integralB +=
                static_cast<float>(errorB) * dt;
        }


        // -----------------------------------------------
        // Convert controller outputs into PWM
        // -----------------------------------------------

        int pwmA = toMotorCommand(outputA);
        int pwmB = toMotorCommand(outputB);

        if (absoluteLong(errorA) <= POSITION_TOLERANCE)
        {
            pwmA = 0;
        }

        if (absoluteLong(errorB) <= POSITION_TOLERANCE)
        {
            pwmB = 0;
        }

        driveMotorA(pwmA);
        driveMotorB(pwmB);


        // -----------------------------------------------
        // Completion check
        // -----------------------------------------------

        if (absoluteLong(errorA) <= POSITION_TOLERANCE &&
            absoluteLong(errorB) <= POSITION_TOLERANCE)
        {
            settledCount++;

            if (settledCount >= SETTLE_SAMPLES)
            {
                break;
            }
        }
        else
        {
            settledCount = 0;
        }


        // -----------------------------------------------
        // Safety timeout
        // -----------------------------------------------

        if (now - startTime >= MOVE_TIMEOUT_MS)
        {
            timedOut = true;
            break;
        }
    }

    stopMotors();

    updateEncoders();

    Serial.println();
    Serial.println("Movement finished.");

    Serial.print("Target A: ");
    Serial.println(targetA);

    Serial.print("Final A:  ");
    Serial.println(getLeftEncoderCount());

    Serial.print("Target B: ");
    Serial.println(targetB);

    Serial.print("Final B:  ");
    Serial.println(getRightEncoderCount());

    if (timedOut)
    {
        Serial.println("WARNING: Movement timed out.");
    }
}


// =======================================================
// RAW ENCODER-COUNT INTERFACE
// =======================================================

void moveXY(
    long targetXCounts,
    long targetYCounts
)
{
    /*
     * Ideal kinematic conversion:
     *
     * A = X + Y
     * B = X - Y
     *
     * This function assumes X and Y have already been
     * expressed using a common encoder-count scale.
     */
    long targetA =
        targetXCounts + targetYCounts;

    long targetB =
        targetXCounts - targetYCounts;

    moveMotorTargets(targetA, targetB);
}


void moveX(long targetCounts)
{
    moveXY(targetCounts, 0);
}


void moveY(long targetCounts)
{
    moveXY(0, targetCounts);
}


// =======================================================
// CALIBRATED MILLIMETRE INTERFACE
// =======================================================

void moveXYmm(
    float targetXmm,
    float targetYmm
)
{
    /*
     * Apply the experimentally measured scale separately
     * to each motor.
     *
     * Motor A:
     * A = +X +Y
     *
     * Motor B:
     * B = +X -Y
     */

    long targetA = lroundf(
        targetXmm * LEFT_X_COUNTS_PER_MM +
        targetYmm * LEFT_Y_COUNTS_PER_MM
    );

    long targetB = lroundf(
        targetXmm * RIGHT_X_COUNTS_PER_MM -
        targetYmm * RIGHT_Y_COUNTS_PER_MM
    );

    moveMotorTargets(targetA, targetB);
}


void moveXmm(float targetMm)
{
    moveXYmm(targetMm, 0.0f);
}


void moveYmm(float targetMm)
{
    moveXYmm(0.0f, targetMm);
}