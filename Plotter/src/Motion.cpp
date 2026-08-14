#include <Arduino.h>
#include <math.h>

#include "Motion.h"
#include "Motor.h"
#include "Encoder.h"


// =======================================================
// CONTROLLER SETTINGS
// =======================================================

// Position PI gains
float KP = 0.8f;
float KI = 0.05f;

// Synchronisation gain
// Increase slightly if the path curves.
// Reduce if the motors oscillate or correct too aggressively.
float SYNC_KP = 0.5f;


// PWM limits
int MIN_PWM = 60;
int MAX_PWM = 255;


// Stopping conditions
long POSITION_TOLERANCE = 5;
int SETTLE_SAMPLES = 5;


// Safety timeout
unsigned long MOVE_TIMEOUT_MS = 8000UL;


// =======================================================
// ENCODER CALIBRATION
// =======================================================

/*
 * Boundary-test averages:
 *
 * X movement over 220 mm:
 *
 * Left motor  = 20276.4 counts
 * Right motor = 19807.8 counts
 *
 * Y movement over 130 mm:
 *
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
// MOTION STATE
// =======================================================

enum class MotionState
{
    IDLE,
    ACTIVE,
    COMPLETE,
    TIMED_OUT
};


static MotionState motionState =
    MotionState::IDLE;


// Motor-coordinate targets
static long targetA = 0;
static long targetB = 0;


// Whether a target has been stored but not started
static bool targetAvailable = false;


// PI controller memory
static float integralA = 0.0f;
static float integralB = 0.0f;


// Movement timing
static unsigned long motionStartTime = 0;
static unsigned long previousUpdateTime = 0;


// Number of consecutive samples within tolerance
static int settledCount = 0;


// =======================================================
// HELPER FUNCTIONS
// =======================================================

static int clampInt(
    int value,
    int minimum,
    int maximum
)
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


// Return the intended direction of a motor target.
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


// Convert controller output into signed PWM.
static int toMotorCommand(float controllerOutput)
{
    int pwm =
        static_cast<int>(controllerOutput);

    pwm =
        clampInt(pwm, -MAX_PWM, MAX_PWM);


    // Ensure a non-zero command is large enough to
    // overcome static friction.
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


// Store targets that are already expressed as encoder counts.
static void setMotionTargetCounts(
    long targetXCounts,
    long targetYCounts
)
{
    /*
     * Plotter kinematics:
     *
     * A = X + Y
     * B = X - Y
     */

    targetA =
        targetXCounts + targetYCounts;

    targetB =
        targetXCounts - targetYCounts;

    targetAvailable = true;
}


// =======================================================
// STORE A MILLIMETRE TARGET
// =======================================================

void setMotionTargetMm(
    float targetXmm,
    float targetYmm
)
{
    /*
     * Apply each motor's measured encoder scale separately.
     *
     * Motor A:
     *
     * A = +X +Y
     *
     * Motor B:
     *
     * B = +X -Y
     */

    targetA = lroundf(
        targetXmm * LEFT_X_COUNTS_PER_MM +
        targetYmm * LEFT_Y_COUNTS_PER_MM
    );

    targetB = lroundf(
        targetXmm * RIGHT_X_COUNTS_PER_MM -
        targetYmm * RIGHT_Y_COUNTS_PER_MM
    );

    targetAvailable = true;


    Serial.println();
    Serial.println("Motion target stored.");

    Serial.print("Target X: ");
    Serial.print(targetXmm);
    Serial.println(" mm");

    Serial.print("Target Y: ");
    Serial.print(targetYmm);
    Serial.println(" mm");

    Serial.print("Motor A target: ");
    Serial.println(targetA);

    Serial.print("Motor B target: ");
    Serial.println(targetB);
}


// =======================================================
// START MOTION
// =======================================================

bool startMotion()
{
    if (!targetAvailable)
    {
        Serial.println(
            "ERROR: Cannot start motion because no target is available."
        );

        return false;
    }


    // The movement is measured relative to the
    // current physical position.
    resetEncoders();


    // Reset controller memory.
    integralA = 0.0f;
    integralB = 0.0f;

    settledCount = 0;


    // Reset timing.
    motionStartTime = millis();
    previousUpdateTime = motionStartTime;


    motionState = MotionState::ACTIVE;


    // The target has now been consumed.
    targetAvailable = false;


    Serial.println("Motion started.");

    return true;
}


// =======================================================
// UPDATE MOTION
// =======================================================

void updateMotion()
{
    // Nothing should be calculated unless a movement
    // is currently active.
    if (motionState != MotionState::ACTIVE)
    {
        return;
    }


    // Read the latest encoder values.
    updateEncoders();

    long measuredA =
        getLeftEncoderCount();

    long measuredB =
        getRightEncoderCount();


    // Position errors
    long errorA =
        targetA - measuredA;

    long errorB =
        targetB - measuredB;


    // Calculate elapsed controller time.
    unsigned long now = millis();

    float dt =
        static_cast<float>(
            now - previousUpdateTime
        ) / 1000.0f;

    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }

    previousUpdateTime = now;


    // ===================================================
    // BASIC PI POSITION CONTROL
    // ===================================================

    float outputA =
        KP * static_cast<float>(errorA) +
        KI * integralA;

    float outputB =
        KP * static_cast<float>(errorB) +
        KI * integralB;


    // ===================================================
    // MOTOR SYNCHRONISATION
    // ===================================================

    /*
     * For a straight line, both motors should complete
     * the same proportion of their total movement at
     * approximately the same time.
     *
     * Example:
     *
     * progressA = 0.50
     * progressB = 0.40
     *
     * Motor A is ahead, so the synchronisation controller
     * reduces Motor A and increases Motor B.
     */

    long targetMagnitudeA =
        absoluteLong(targetA);

    long targetMagnitudeB =
        absoluteLong(targetB);


    // Synchronisation requires both motors to have
    // meaningful non-zero movement targets.
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

        if (targetMagnitudeA <
            targetMagnitudeB)
        {
            smallerTargetMagnitude =
                targetMagnitudeA;
        }
        else
        {
            smallerTargetMagnitude =
                targetMagnitudeB;
        }


        // Convert the proportional progress error into
        // an approximate encoder-count error.
        float synchronisationError =
            progressDifference *
            static_cast<float>(
                smallerTargetMagnitude
            );


        float synchronisationOutput =
            SYNC_KP *
            synchronisationError;


        /*
         * If Motor A is ahead:
         *
         * - reduce A in its movement direction;
         * - increase B in its movement direction.
         *
         * motorDirection() also handles movements where
         * one or both targets are negative.
         */

        outputA -=
            motorDirection(targetA) *
            synchronisationOutput;

        outputB +=
            motorDirection(targetB) *
            synchronisationOutput;
    }


    // ===================================================
    // ANTI-WINDUP
    // ===================================================

    bool saturatedPositiveA =
        outputA > MAX_PWM &&
        errorA > 0;

    bool saturatedNegativeA =
        outputA < -MAX_PWM &&
        errorA < 0;


    if (!saturatedPositiveA &&
        !saturatedNegativeA)
    {
        integralA +=
            static_cast<float>(errorA) *
            dt;
    }


    bool saturatedPositiveB =
        outputB > MAX_PWM &&
        errorB > 0;

    bool saturatedNegativeB =
        outputB < -MAX_PWM &&
        errorB < 0;


    if (!saturatedPositiveB &&
        !saturatedNegativeB)
    {
        integralB +=
            static_cast<float>(errorB) *
            dt;
    }


    // ===================================================
    // CONVERT OUTPUTS TO PWM
    // ===================================================

    int pwmA =
        toMotorCommand(outputA);

    int pwmB =
        toMotorCommand(outputB);


    // Stop controlling a motor once it is within its
    // final encoder-count tolerance.
    if (absoluteLong(errorA) <=
        POSITION_TOLERANCE)
    {
        pwmA = 0;
    }

    if (absoluteLong(errorB) <=
        POSITION_TOLERANCE)
    {
        pwmB = 0;
    }


    driveMotorA(pwmA);
    driveMotorB(pwmB);


    // ===================================================
    // COMPLETION CHECK
    // ===================================================

    if (absoluteLong(errorA) <=
            POSITION_TOLERANCE &&
        absoluteLong(errorB) <=
            POSITION_TOLERANCE)
    {
        settledCount++;


        // Require several consecutive successful readings.
        if (settledCount >= SETTLE_SAMPLES)
        {
            stopMotors();

            motionState =
                MotionState::COMPLETE;


            Serial.println();
            Serial.println("Motion complete.");

            Serial.print("Motor A target: ");
            Serial.println(targetA);

            Serial.print("Motor A final:  ");
            Serial.println(measuredA);

            Serial.print("Motor B target: ");
            Serial.println(targetB);

            Serial.print("Motor B final:  ");
            Serial.println(measuredB);

            return;
        }
    }
    else
    {
        settledCount = 0;
    }


    // ===================================================
    // TIMEOUT CHECK
    // ===================================================

    if (now - motionStartTime >=
        MOVE_TIMEOUT_MS)
    {
        stopMotors();

        motionState =
            MotionState::TIMED_OUT;


        Serial.println();
        Serial.println("ERROR: Motion timed out.");

        Serial.print("Motor A target: ");
        Serial.println(targetA);

        Serial.print("Motor A final:  ");
        Serial.println(measuredA);

        Serial.print("Motor B target: ");
        Serial.println(targetB);

        Serial.print("Motor B final:  ");
        Serial.println(measuredB);
    }
}


// =======================================================
// MOTION STATUS
// =======================================================

bool isMotionComplete()
{
    return motionState ==
           MotionState::COMPLETE;
}


bool hasMotionTimedOut()
{
    return motionState ==
           MotionState::TIMED_OUT;
}


bool isMotionActive()
{
    return motionState ==
           MotionState::ACTIVE;
}


// =======================================================
// RESET MOTION
// =======================================================

void resetMotion()
{
    stopMotors();

    integralA = 0.0f;
    integralB = 0.0f;

    settledCount = 0;

    targetA = 0;
    targetB = 0;

    targetAvailable = false;

    motionState =
        MotionState::IDLE;
}


// =======================================================
// CONVERSION FUNCTIONS
// =======================================================

long xMmToCounts(float distanceMm)
{
    /*
     * This returns an average Cartesian count value.
     * The calibrated millimetre interface below applies
     * the individual motor scales instead.
     */

    float averageXCountsPerMm =
        0.5f *
        (
            LEFT_X_COUNTS_PER_MM +
            RIGHT_X_COUNTS_PER_MM
        );

    return lroundf(
        distanceMm *
        averageXCountsPerMm
    );
}


long yMmToCounts(float distanceMm)
{
    float averageYCountsPerMm =
        0.5f *
        (
            LEFT_Y_COUNTS_PER_MM +
            RIGHT_Y_COUNTS_PER_MM
        );

    return lroundf(
        distanceMm *
        averageYCountsPerMm
    );
}


// =======================================================
// LEGACY BLOCKING FUNCTIONS
// =======================================================

/*
 * These functions are kept so older testing code still
 * compiles.
 *
 * Do not call them from FSM::movingUpdate(), because they
 * wait inside a while loop.
 *
 * The FSM must instead use:
 *
 * setMotionTargetMm()
 * startMotion()
 * updateMotion()
 */

void moveXY(
    long targetXCounts,
    long targetYCounts
)
{
    setMotionTargetCounts(
        targetXCounts,
        targetYCounts
    );


    if (!startMotion())
    {
        return;
    }


    while (isMotionActive())
    {
        updateMotion();
    }
}


void moveX(long targetCounts)
{
    moveXY(targetCounts, 0);
}


void moveY(long targetCounts)
{
    moveXY(0, targetCounts);
}


void moveXYmm(
    float targetXmm,
    float targetYmm
)
{
    setMotionTargetMm(
        targetXmm,
        targetYmm
    );


    if (!startMotion())
    {
        return;
    }


    while (isMotionActive())
    {
        updateMotion();
    }
}


void moveXmm(float targetMm)
{
    moveXYmm(targetMm, 0.0f);
}


void moveYmm(float targetMm)
{
    moveXYmm(0.0f, targetMm);
}