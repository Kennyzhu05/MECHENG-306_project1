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

// Synchronisation correction between the two motors
float SYNC_KP = 0.5f;


// =======================================================
// PWM SETTINGS
// =======================================================

int MIN_PWM = 60;
int MAX_PWM = 255;


// =======================================================
// STOPPING SETTINGS
// =======================================================

// Motor is considered to have reached the target
// when it reaches or crosses target -/+ this tolerance.
long POSITION_TOLERANCE = 5;

// Kept for compatibility with Motion.h.
// The new reached-target logic does not require
// consecutive samples because completion is latched.
int SETTLE_SAMPLES = 5;


// =======================================================
// SAFETY TIMEOUT
// =======================================================

unsigned long MOVE_TIMEOUT_MS = 8000UL;


// =======================================================
// CALIBRATION
// =======================================================

/*
 * Measured boundary-test results:
 *
 * X = 220 mm:
 *
 * Left encoder  = +20276.4
 * Right encoder = +19807.8
 *
 *
 * Y = 130 mm upward:
 *
 * Left encoder  = +12393.6
 * Right encoder = -12285.6
 *
 *
 * Therefore:
 *
 * RIGHT:
 *     Left  +
 *     Right +
 *
 * LEFT:
 *     Left  -
 *     Right -
 *
 * UP:
 *     Left  +
 *     Right -
 *
 * DOWN:
 *     Left  -
 *     Right +
 *
 *
 * This matches:
 *
 * Motor A = X + Y
 * Motor B = X - Y
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


// Signed encoder targets for the two motors
static long targetA = 0;
static long targetB = 0;


// Has a target been stored but not started yet?
static bool targetAvailable = false;


// =======================================================
// REACHED FLAGS
// =======================================================

/*
 * IMPORTANT:
 *
 * Once one motor reaches or passes its target,
 * its reached flag becomes true.
 *
 * It then stays true until the next movement.
 *
 * That motor will NEVER be driven again during
 * the current movement.
 */

static bool motorAReached = false;
static bool motorBReached = false;


// =======================================================
// PI CONTROLLER MEMORY
// =======================================================

static float integralA = 0.0f;
static float integralB = 0.0f;


// =======================================================
// TIMING
// =======================================================

static unsigned long motionStartTime = 0;
static unsigned long previousUpdateTime = 0;


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


// =======================================================
// PWM CONVERSION
// =======================================================

static int toMotorCommand(float controllerOutput)
{
    int pwm =
        static_cast<int>(controllerOutput);


    pwm =
        clampInt(
            pwm,
            -MAX_PWM,
            MAX_PWM
        );


    /*
     * Ensure a non-zero motor command has enough
     * PWM to overcome static friction.
     */

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
// TARGET REACHED CHECK
// =======================================================

/*
 * This is the critical stopping logic.
 *
 * POSITIVE target:
 *
 * target = +7000
 *
 * Stop when:
 *
 * measured >= 6995
 *
 *
 * NEGATIVE target:
 *
 * target = -7000
 *
 * Stop when:
 *
 * measured <= -6995
 *
 *
 * Therefore it doesn't matter if the encoder jumps
 * slightly past the target.
 */

static bool hasReachedTarget(
    long measured,
    long target
)
{
    // No movement required
    if (target == 0)
    {
        return true;
    }


    // Positive movement
    if (target > 0)
    {
        return
            measured >=
            (target - POSITION_TOLERANCE);
    }


    // Negative movement
    return
        measured <=
        (target + POSITION_TOLERANCE);
}


// =======================================================
// STORE RAW X/Y COUNT TARGET
// =======================================================

static void setMotionTargetCounts(
    long targetXCounts,
    long targetYCounts
)
{
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Cannot change motion target while moving."
        );

        return;
    }


    /*
     * XY -> motor coordinates:
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
// STORE MM TARGET
// =======================================================

void setMotionTargetMm(
    float targetXmm,
    float targetYmm
)
{
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Cannot change motion target while moving."
        );

        return;
    }


    /*
     * Use each motor's measured calibration separately.
     *
     *
     * Motor A / left:
     *
     * +X -> positive
     * +Y -> positive
     *
     *
     * Motor B / right:
     *
     * +X -> positive
     * +Y -> negative
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

    Serial.print("X target: ");
    Serial.print(targetXmm);
    Serial.println(" mm");

    Serial.print("Y target: ");
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
    /*
     * Prevent the FSM or another part of the program
     * from accidentally restarting an active movement.
     */

    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: startMotion() called while already moving."
        );

        return false;
    }


    if (!targetAvailable)
    {
        Serial.println(
            "ERROR: Cannot start motion because no target is available."
        );

        return false;
    }


    // ---------------------------------------------------
    // Reset encoder displacement
    // ---------------------------------------------------

    resetEncoders();


    // ---------------------------------------------------
    // Reset controller
    // ---------------------------------------------------

    integralA = 0.0f;
    integralB = 0.0f;


    // ---------------------------------------------------
    // Reset reached flags
    // ---------------------------------------------------

    motorAReached =
        (targetA == 0);

    motorBReached =
        (targetB == 0);


    // ---------------------------------------------------
    // Reset timer
    // ---------------------------------------------------

    motionStartTime =
        millis();

    previousUpdateTime =
        motionStartTime;


    motionState =
        MotionState::ACTIVE;


    // Target has been consumed
    targetAvailable = false;


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


    // ===================================================
    // TIME
    // ===================================================

    unsigned long now =
        millis();


    // ===================================================
    // SAFETY TIMEOUT
    // ===================================================

    /*
     * Check timeout before issuing another motor command.
     *
     * Even with completely incorrect encoder readings,
     * motors must stop after MOVE_TIMEOUT_MS.
     */

    if (
        now - motionStartTime >=
        MOVE_TIMEOUT_MS
    )
    {
        stopMotors();

        motionState =
            MotionState::TIMED_OUT;


        Serial.println();
        Serial.println("ERROR: Motion timed out.");

        Serial.print("Target A: ");
        Serial.println(targetA);

        Serial.print("Encoder A: ");
        Serial.println(
            getLeftEncoderCount()
        );

        Serial.print("Target B: ");
        Serial.println(targetB);

        Serial.print("Encoder B: ");
        Serial.println(
            getRightEncoderCount()
        );


        return;
    }


    // ===================================================
    // UPDATE ENCODERS
    // ===================================================

    updateEncoders();


    long measuredA =
        getLeftEncoderCount();

    long measuredB =
        getRightEncoderCount();


    // ===================================================
    // CHECK WHETHER EACH MOTOR HAS REACHED ITS TARGET
    // ===================================================

    /*
     * Once true, these flags NEVER become false during
     * this movement.
     */

    if (!motorAReached)
    {
        if (
            hasReachedTarget(
                measuredA,
                targetA
            )
        )
        {
            motorAReached = true;

            integralA = 0.0f;

            // Stop Motor A immediately
            driveMotorA(0);


            Serial.print(
                "Motor A reached target at: "
            );

            Serial.println(measuredA);
        }
    }


    if (!motorBReached)
    {
        if (
            hasReachedTarget(
                measuredB,
                targetB
            )
        )
        {
            motorBReached = true;

            integralB = 0.0f;

            // Stop Motor B immediately
            driveMotorB(0);


            Serial.print(
                "Motor B reached target at: "
            );

            Serial.println(measuredB);
        }
    }


    // ===================================================
    // BOTH MOTORS FINISHED
    // ===================================================

    if (
        motorAReached &&
        motorBReached
    )
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


    // ===================================================
    // CALCULATE DT
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
    // SIGNED POSITION ERRORS
    // ===================================================

    /*
     * IMPORTANT:
     *
     * Encoder.cpp already provides signed counts.
     *
     * Therefore use:
     *
     * error = target - measured
     *
     * directly.
     */

    long errorA =
        targetA -
        measuredA;

    long errorB =
        targetB -
        measuredB;


    // ===================================================
    // PI CONTROL
    // ===================================================

    float outputA = 0.0f;
    float outputB = 0.0f;


    if (!motorAReached)
    {
        outputA =
            KP *
                static_cast<float>(
                    errorA
                )
            +
            KI *
                integralA;
    }


    if (!motorBReached)
    {
        outputB =
            KP *
                static_cast<float>(
                    errorB
                )
            +
            KI *
                integralB;
    }


    // ===================================================
    // SYNCHRONISATION
    // ===================================================

    /*
     * Only synchronise while BOTH motors are still moving.
     *
     * Once one motor reaches its target, we completely stop
     * controlling that motor and let the other one finish.
     */

    if (
        !motorAReached &&
        !motorBReached &&
        absoluteLong(targetA) >
            POSITION_TOLERANCE &&
        absoluteLong(targetB) >
            POSITION_TOLERANCE
    )
    {
        /*
         * Because encoder counts and targets are both signed,
         *
         * measured / target
         *
         * gives positive progress for both positive and
         * negative movement.
         */

        float progressA =
            static_cast<float>(
                measuredA
            ) /
            static_cast<float>(
                targetA
            );


        float progressB =
            static_cast<float>(
                measuredB
            ) /
            static_cast<float>(
                targetB
            );


        float progressDifference =
            progressA -
            progressB;


        long smallerTargetMagnitude;

        if (
            absoluteLong(targetA) <
            absoluteLong(targetB)
        )
        {
            smallerTargetMagnitude =
                absoluteLong(targetA);
        }
        else
        {
            smallerTargetMagnitude =
                absoluteLong(targetB);
        }


        float syncErrorCounts =
            progressDifference *
            static_cast<float>(
                smallerTargetMagnitude
            );


        float syncCorrection =
            SYNC_KP *
            syncErrorCounts;


        /*
         * Determine direction of each target.
         */

        float directionA =
            (targetA > 0)
                ? 1.0f
                : -1.0f;

        float directionB =
            (targetB > 0)
                ? 1.0f
                : -1.0f;


        /*
         * A ahead:
         *
         * reduce A command in its travel direction
         * increase B command in its travel direction
         */

        outputA -=
            directionA *
            syncCorrection;

        outputB +=
            directionB *
            syncCorrection;
    }


    // ===================================================
    // ANTI-WINDUP
    // ===================================================

    if (!motorAReached)
    {
        bool saturatedPositiveA =
            outputA > MAX_PWM &&
            errorA > 0;

        bool saturatedNegativeA =
            outputA < -MAX_PWM &&
            errorA < 0;


        if (
            !saturatedPositiveA &&
            !saturatedNegativeA
        )
        {
            integralA +=
                static_cast<float>(
                    errorA
                ) *
                dt;
        }
    }


    if (!motorBReached)
    {
        bool saturatedPositiveB =
            outputB > MAX_PWM &&
            errorB > 0;

        bool saturatedNegativeB =
            outputB < -MAX_PWM &&
            errorB < 0;


        if (
            !saturatedPositiveB &&
            !saturatedNegativeB
        )
        {
            integralB +=
                static_cast<float>(
                    errorB
                ) *
                dt;
        }
    }


    // ===================================================
    // PWM COMMANDS
    // ===================================================

    int pwmA = 0;
    int pwmB = 0;


    if (!motorAReached)
    {
        pwmA =
            toMotorCommand(
                outputA
            );
    }


    if (!motorBReached)
    {
        pwmB =
            toMotorCommand(
                outputB
            );
    }


    // ===================================================
    // DRIVE MOTORS
    // ===================================================

    driveMotorA(pwmA);
    driveMotorB(pwmB);
}


// =======================================================
// STATUS FUNCTIONS
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


    targetA = 0;
    targetB = 0;


    targetAvailable = false;


    motorAReached = false;
    motorBReached = false;


    integralA = 0.0f;
    integralB = 0.0f;


    motionStartTime = 0;
    previousUpdateTime = 0;


    motionState =
        MotionState::IDLE;
}


// =======================================================
// CONVERSION HELPERS
// =======================================================

long xMmToCounts(float distanceMm)
{
    /*
     * This is only for the old raw-count interface.
     * Use average X calibration.
     */

    float averageCountsPerMm =
        (
            LEFT_X_COUNTS_PER_MM +
            RIGHT_X_COUNTS_PER_MM
        ) /
        2.0f;


    return
        lroundf(
            distanceMm *
            averageCountsPerMm
        );
}


long yMmToCounts(float distanceMm)
{
    /*
     * Use average magnitude of the Y calibration.
     */

    float averageCountsPerMm =
        (
            LEFT_Y_COUNTS_PER_MM +
            RIGHT_Y_COUNTS_PER_MM
        ) /
        2.0f;


    return
        lroundf(
            distanceMm *
            averageCountsPerMm
        );
}


// =======================================================
// BLOCKING TEST FUNCTIONS
// =======================================================

/*
 * These functions are useful for direct testing.
 *
 * DO NOT call moveXYmm() inside FSM::movingUpdate().
 *
 *
 * FSM operation should be:
 *
 * setMotionTargetMm(...)
 *
 * process G1
 *
 * startMotion() once
 *
 * then repeatedly:
 *
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
            "ERROR: Motion already active."
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
            "ERROR: Motion already active."
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