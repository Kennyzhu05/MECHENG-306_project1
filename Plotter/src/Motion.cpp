#include <Arduino.h>
#include <math.h>

#include "Motion.h"
#include "Motor.h"
#include "Encoder.h"


// =======================================================
// CONTROLLER SETTINGS
// =======================================================

float KP = 0.8f;
float KI = 0.05f;

// Synchronisation correction
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
 *
 * We use the magnitudes for distance calibration.
 * Direction is handled separately by targetA / targetB.
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


// Signed motor targets.
//
// Sign = motor direction
// Magnitude = required travel distance in encoder counts
static long targetA = 0;
static long targetB = 0;


// True when a target has been stored but not yet started.
static bool targetAvailable = false;


// PI controller memory.
// These integrate MAGNITUDE error only.
static float integralA = 0.0f;
static float integralB = 0.0f;


// Timing
static unsigned long motionStartTime = 0;
static unsigned long previousUpdateTime = 0;


// Consecutive samples inside tolerance
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


// Returns:
//
// +1 for positive target
// -1 for negative target
//  0 for zero target
static int motorDirection(long target)
{
    if (target > 0)
    {
        return 1;
    }

    if (target < 0)
    {
        return -1;
    }

    return 0;
}


// Convert a POSITIVE controller magnitude into a signed motor PWM.
static int makeSignedMotorCommand(
    float controllerMagnitude,
    long target
)
{
    // Motor has no requested movement.
    if (target == 0)
    {
        return 0;
    }


    int pwm =
        static_cast<int>(
            controllerMagnitude
        );


    pwm =
        clampInt(
            pwm,
            0,
            MAX_PWM
        );


    // Make sure the motor receives enough PWM
    // to overcome static friction.
    if (pwm > 0 && pwm < MIN_PWM)
    {
        pwm = MIN_PWM;
    }


    return motorDirection(target) * pwm;
}


// =======================================================
// STORE RAW ENCODER-COUNT TARGET
// =======================================================

static void setMotionTargetCounts(
    long targetXCounts,
    long targetYCounts
)
{
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Cannot change target while motion is active."
        );

        return;
    }


    /*
     * Plotter kinematics:
     *
     * A = X + Y
     * B = X - Y
     */

    targetA =
        targetXCounts +
        targetYCounts;

    targetB =
        targetXCounts -
        targetYCounts;


    targetAvailable = true;
}


// =======================================================
// STORE MILLIMETRE TARGET
// =======================================================

void setMotionTargetMm(
    float targetXmm,
    float targetYmm
)
{
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Cannot change target while motion is active."
        );

        return;
    }


    /*
     * Motor-coordinate convention used by the existing
     * Motor.cpp:
     *
     * RIGHT:
     * A+
     * B+
     *
     * LEFT:
     * A-
     * B-
     *
     * UP:
     * A+
     * B-
     *
     * DOWN:
     * A-
     * B+
     *
     * Therefore:
     *
     * A = +X +Y
     * B = +X -Y
     */


    targetA =
        lroundf(
            targetXmm *
                LEFT_X_COUNTS_PER_MM
            +
            targetYmm *
                LEFT_Y_COUNTS_PER_MM
        );


    targetB =
        lroundf(
            targetXmm *
                RIGHT_X_COUNTS_PER_MM
            -
            targetYmm *
                RIGHT_Y_COUNTS_PER_MM
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

    Serial.print("Motor A signed target: ");
    Serial.println(targetA);

    Serial.print("Motor B signed target: ");
    Serial.println(targetB);

    Serial.print("Motor A travel magnitude: ");
    Serial.println(
        absoluteLong(targetA)
    );

    Serial.print("Motor B travel magnitude: ");
    Serial.println(
        absoluteLong(targetB)
    );
}


// =======================================================
// START MOTION
// =======================================================

bool startMotion()
{
    /*
     * Prevent accidental repeated starting.
     *
     * This is important because resetting the encoders
     * repeatedly would make the plotter keep travelling.
     */
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: startMotion() called while motion is already active."
        );

        return false;
    }


    if (!targetAvailable)
    {
        Serial.println(
            "ERROR: Cannot start motion - no target available."
        );

        return false;
    }


    // Motion is relative to current physical position.
    resetEncoders();


    // Reset controller memory.
    integralA = 0.0f;
    integralB = 0.0f;


    settledCount = 0;


    // Reset timing.
    motionStartTime =
        millis();

    previousUpdateTime =
        motionStartTime;


    motionState =
        MotionState::ACTIVE;


    // Target has now been consumed.
    targetAvailable =
        false;


    Serial.println();
    Serial.println("Motion started.");


    return true;
}


// =======================================================
// UPDATE MOTION
// =======================================================

void updateMotion()
{
    if (motionState != MotionState::ACTIVE)
    {
        return;
    }


    unsigned long now =
        millis();


    // ===================================================
    // TIMEOUT
    // ===================================================

    if (
        now - motionStartTime >=
        MOVE_TIMEOUT_MS
    )
    {
        stopMotors();

        motionState =
            MotionState::TIMED_OUT;


        Serial.println();
        Serial.println(
            "ERROR: Motion timed out."
        );


        updateEncoders();

        Serial.print(
            "Final raw encoder A: "
        );
        Serial.println(
            getLeftEncoderCount()
        );

        Serial.print(
            "Final raw encoder B: "
        );
        Serial.println(
            getRightEncoderCount()
        );


        return;
    }


    // ===================================================
    // READ ENCODERS
    // ===================================================

    updateEncoders();


    long rawMeasuredA =
        getLeftEncoderCount();

    long rawMeasuredB =
        getRightEncoderCount();


    /*
     * CRITICAL CHANGE:
     *
     * We care about HOW FAR each motor has travelled.
     *
     * We do NOT rely on the encoder sign to determine
     * whether the motor moved forward or backward.
     *
     * Direction is already known from targetA / targetB.
     */

    long travelledA =
        absoluteLong(
            rawMeasuredA
        );

    long travelledB =
        absoluteLong(
            rawMeasuredB
        );


    long targetMagnitudeA =
        absoluteLong(
            targetA
        );

    long targetMagnitudeB =
        absoluteLong(
            targetB
        );


    // Remaining travel magnitude.
    long errorA =
        targetMagnitudeA -
        travelledA;

    long errorB =
        targetMagnitudeB -
        travelledB;


    // ===================================================
    // COMPLETION CHECK
    // ===================================================

    bool motorAReached =
        absoluteLong(errorA) <=
        POSITION_TOLERANCE;

    bool motorBReached =
        absoluteLong(errorB) <=
        POSITION_TOLERANCE;


    /*
     * A zero target should already count as complete.
     */
    if (
        targetMagnitudeA <=
        POSITION_TOLERANCE
    )
    {
        motorAReached = true;
    }


    if (
        targetMagnitudeB <=
        POSITION_TOLERANCE
    )
    {
        motorBReached = true;
    }


    if (
        motorAReached &&
        motorBReached
    )
    {
        stopMotors();

        settledCount++;


        if (
            settledCount >=
            SETTLE_SAMPLES
        )
        {
            motionState =
                MotionState::COMPLETE;


            Serial.println();
            Serial.println(
                "Motion complete."
            );


            Serial.print(
                "Target magnitude A: "
            );
            Serial.println(
                targetMagnitudeA
            );

            Serial.print(
                "Travelled A:        "
            );
            Serial.println(
                travelledA
            );


            Serial.print(
                "Target magnitude B: "
            );
            Serial.println(
                targetMagnitudeB
            );

            Serial.print(
                "Travelled B:        "
            );
            Serial.println(
                travelledB
            );


            Serial.print(
                "Raw encoder A:      "
            );
            Serial.println(
                rawMeasuredA
            );

            Serial.print(
                "Raw encoder B:      "
            );
            Serial.println(
                rawMeasuredB
            );


            return;
        }


        return;
    }
    else
    {
        settledCount = 0;
    }


    // ===================================================
    // CONTROLLER TIME STEP
    // ===================================================

    float dt =
        static_cast<float>(
            now -
            previousUpdateTime
        ) / 1000.0f;


    if (dt <= 0.0f)
    {
        dt = 0.001f;
    }


    previousUpdateTime =
        now;


    // ===================================================
    // PI CONTROL USING DISTANCE MAGNITUDE
    // ===================================================

    /*
     * errorA / errorB are remaining DISTANCES.
     *
     * Direction is applied later using targetA / targetB.
     */

    float outputMagnitudeA =
        KP *
        static_cast<float>(
            errorA
        )
        +
        KI *
        integralA;


    float outputMagnitudeB =
        KP *
        static_cast<float>(
            errorB
        )
        +
        KI *
        integralB;


    /*
     * If a motor has passed the target slightly,
     * do not command it farther in the same direction.
     *
     * This controller is intended to stop at the target,
     * rather than reverse after overshoot.
     */
    if (errorA <= 0)
    {
        outputMagnitudeA = 0.0f;
    }

    if (errorB <= 0)
    {
        outputMagnitudeB = 0.0f;
    }


    // ===================================================
    // MOTOR SYNCHRONISATION
    // ===================================================

    /*
     * Compare how much of each requested travel has
     * been completed.
     *
     * This works regardless of motor direction because
     * travelledA/B and targetMagnitudeA/B are positive.
     */

    if (
        targetMagnitudeA >
            POSITION_TOLERANCE &&
        targetMagnitudeB >
            POSITION_TOLERANCE
    )
    {
        float progressA =
            static_cast<float>(
                travelledA
            ) /
            static_cast<float>(
                targetMagnitudeA
            );


        float progressB =
            static_cast<float>(
                travelledB
            ) /
            static_cast<float>(
                targetMagnitudeB
            );


        float progressDifference =
            progressA -
            progressB;


        long smallerTargetMagnitude;

        if (
            targetMagnitudeA <
            targetMagnitudeB
        )
        {
            smallerTargetMagnitude =
                targetMagnitudeA;
        }
        else
        {
            smallerTargetMagnitude =
                targetMagnitudeB;
        }


        float synchronisationError =
            progressDifference *
            static_cast<float>(
                smallerTargetMagnitude
            );


        float synchronisationCorrection =
            SYNC_KP *
            synchronisationError;


        /*
         * If A is ahead:
         *
         * decrease A magnitude
         * increase B magnitude
         */

        outputMagnitudeA -=
            synchronisationCorrection;

        outputMagnitudeB +=
            synchronisationCorrection;
    }


    // Do not allow negative speed magnitudes.
    if (outputMagnitudeA < 0.0f)
    {
        outputMagnitudeA = 0.0f;
    }

    if (outputMagnitudeB < 0.0f)
    {
        outputMagnitudeB = 0.0f;
    }


    // ===================================================
    // ANTI-WINDUP
    // ===================================================

    bool saturatedA =
        outputMagnitudeA >
        MAX_PWM;

    bool saturatedB =
        outputMagnitudeB >
        MAX_PWM;


    if (
        !saturatedA &&
        errorA > 0
    )
    {
        integralA +=
            static_cast<float>(
                errorA
            ) *
            dt;
    }


    if (
        !saturatedB &&
        errorB > 0
    )
    {
        integralB +=
            static_cast<float>(
                errorB
            ) *
            dt;
    }


    // ===================================================
    // MOTOR COMMANDS
    // ===================================================

    int pwmA =
        makeSignedMotorCommand(
            outputMagnitudeA,
            targetA
        );


    int pwmB =
        makeSignedMotorCommand(
            outputMagnitudeB,
            targetB
        );


    // Stop motors that have already reached target.
    if (motorAReached)
    {
        pwmA = 0;
    }


    if (motorBReached)
    {
        pwmB = 0;
    }


    driveMotorA(pwmA);
    driveMotorB(pwmB);
}


// =======================================================
// MOTION STATUS
// =======================================================

bool isMotionActive()
{
    return
        motionState ==
        MotionState::ACTIVE;
}


bool isMotionComplete()
{
    return
        motionState ==
        MotionState::COMPLETE;
}


bool hasMotionTimedOut()
{
    return
        motionState ==
        MotionState::TIMED_OUT;
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


    targetAvailable =
        false;


    motionStartTime = 0;
    previousUpdateTime = 0;


    motionState =
        MotionState::IDLE;
}


// =======================================================
// CONVERSION FUNCTIONS
// =======================================================

long xMmToCounts(float distanceMm)
{
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
// BLOCKING TEST FUNCTIONS
// =======================================================

/*
 * These are useful for direct hardware tests.
 *
 * Do NOT call them inside FSM::movingUpdate().
 *
 * FSM use:
 *
 * setMotionTargetMm(...)
 * startMotion()
 * updateMotion()
 */

void moveXY(
    long targetXCounts,
    long targetYCounts
)
{
    if (isMotionActive())
    {
        Serial.println(
            "ERROR: Another motion is already active."
        );

        return;
    }


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

        delay(1);
    }
}


void moveX(long targetCounts)
{
    moveXY(
        targetCounts,
        0
    );
}


void moveY(long targetCounts)
{
    moveXY(
        0,
        targetCounts
    );
}


void moveXYmm(
    float targetXmm,
    float targetYmm
)
{
    if (isMotionActive())
    {
        Serial.println(
            "ERROR: Another motion is already active."
        );

        return;
    }


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

        delay(1);
    }
}


void moveXmm(float targetMm)
{
    moveXYmm(
        targetMm,
        0.0f
    );
}


void moveYmm(float targetMm)
{
    moveXYmm(
        0.0f,
        targetMm
    );
}