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
 * Measured boundary-test values:
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


// True after setMotionTarget...()
// False once startMotion() consumes the target.
static bool targetAvailable = false;


// PI controller memory
static float integralA = 0.0f;
static float integralB = 0.0f;


// Timing
static unsigned long motionStartTime = 0;
static unsigned long previousUpdateTime = 0;


// Number of consecutive samples inside tolerance
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
    int pwm =
        static_cast<int>(controllerOutput);

    pwm =
        clampInt(
            pwm,
            -MAX_PWM,
            MAX_PWM
        );


    // Make sure a commanded motor receives enough
    // PWM to overcome static friction.
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
// STORE TARGET IN ENCODER COUNTS
// =======================================================

static void setMotionTargetCounts(
    long targetXCounts,
    long targetYCounts
)
{
    /*
     * IMPORTANT:
     *
     * Never change the target while a movement is active.
     */
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Tried to change target while motion is active."
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
// STORE TARGET IN MILLIMETRES
// =======================================================

void setMotionTargetMm(
    float targetXmm,
    float targetYmm
)
{
    /*
     * Do not allow the FSM or another piece of code
     * to replace the target halfway through a move.
     */
    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: Tried to change target while motion is active."
        );

        return;
    }


    /*
     * Motor A:
     *
     * A = +X +Y
     *
     * Motor B:
     *
     * B = +X -Y
     *
     * Each motor uses its own experimentally measured
     * counts/mm calibration.
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
    /*
     * CRITICAL PROTECTION:
     *
     * If startMotion() is accidentally called repeatedly
     * by the FSM, DO NOT reset the encoders or timer.
     *
     * Previously, repeatedly starting the move could make
     * an 80 mm command effectively become:
     *
     *   move another 80 mm
     *   move another 80 mm
     *   move another 80 mm
     *   ...
     */

    if (motionState == MotionState::ACTIVE)
    {
        Serial.println(
            "WARNING: startMotion() called while motion already active."
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


    // Movement is relative to the current position.
    resetEncoders();


    // Reset PI controller memory.
    integralA = 0.0f;
    integralB = 0.0f;


    // Reset completion counter.
    settledCount = 0;


    // Start timeout and controller timing.
    motionStartTime = millis();
    previousUpdateTime = motionStartTime;


    // Motion is now running.
    motionState =
        MotionState::ACTIVE;


    // The stored target has now been consumed.
    targetAvailable = false;


    Serial.println();
    Serial.println("Motion started.");

    Serial.print("Motor A target: ");
    Serial.println(targetA);

    Serial.print("Motor B target: ");
    Serial.println(targetB);


    return true;
}


// =======================================================
// UPDATE MOTION
// =======================================================

void updateMotion()
{
    /*
     * updateMotion() is called repeatedly by:
     *
     *      FSM::movingUpdate()
     *
     * It NEVER starts a new movement.
     */
    if (motionState != MotionState::ACTIVE)
    {
        return;
    }


    unsigned long now =
        millis();


    // ===================================================
    // SAFETY TIMEOUT
    // ===================================================

    /*
     * Check timeout BEFORE issuing another motor command.
     *
     * This guarantees that an active movement cannot
     * continue beyond MOVE_TIMEOUT_MS.
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
        Serial.println(
            "ERROR: Motion timed out."
        );

        return;
    }


    // ===================================================
    // READ ENCODERS
    // ===================================================

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


    // ===================================================
    // COMPLETION CHECK
    // ===================================================

    bool motorAReached =
        absoluteLong(errorA) <=
        POSITION_TOLERANCE;

    bool motorBReached =
        absoluteLong(errorB) <=
        POSITION_TOLERANCE;


    if (
        motorAReached &&
        motorBReached
    )
    {
        /*
         * Both motors are already at the destination.
         *
         * Stop them immediately while checking whether
         * they remain there for SETTLE_SAMPLES readings.
         */
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
                "Motor A target: "
            );
            Serial.println(targetA);

            Serial.print(
                "Motor A final:  "
            );
            Serial.println(measuredA);

            Serial.print(
                "Motor B target: "
            );
            Serial.println(targetB);

            Serial.print(
                "Motor B final:  "
            );
            Serial.println(measuredB);

            return;
        }


        /*
         * Do not calculate another PI output while
         * both motors are inside the tolerance.
         */
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
    // PI POSITION CONTROL
    // ===================================================

    float outputA =
        KP *
        static_cast<float>(errorA) +
        KI *
        integralA;


    float outputB =
        KP *
        static_cast<float>(errorB) +
        KI *
        integralB;


    // ===================================================
    // MOTOR SYNCHRONISATION
    // ===================================================

    long targetMagnitudeA =
        absoluteLong(targetA);

    long targetMagnitudeB =
        absoluteLong(targetB);


    /*
     * Synchronisation only makes sense when both motors
     * have significant non-zero movement.
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


        float synchronisationOutput =
            SYNC_KP *
            synchronisationError;


        /*
         * If A is ahead:
         *
         * reduce A and increase B.
         *
         * motorDirection() makes this work for
         * positive and negative movements.
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


    if (
        !saturatedPositiveA &&
        !saturatedNegativeA
    )
    {
        integralA +=
            static_cast<float>(
                errorA
            ) * dt;
    }


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
            ) * dt;
    }


    // ===================================================
    // CONVERT TO PWM
    // ===================================================

    int pwmA =
        toMotorCommand(outputA);

    int pwmB =
        toMotorCommand(outputB);


    /*
     * If one motor has reached its final position,
     * stop that motor while the other finishes.
     */

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


    targetAvailable = false;


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
 * These functions are useful for simple hardware tests.
 *
 * DO NOT use these functions from FSM::movingUpdate().
 *
 * The FSM must use:
 *
 *      setMotionTargetMm()
 *      startMotion()
 *      updateMotion()
 *
 * separately.
 */

void moveXY(
    long targetXCounts,
    long targetYCounts
)
{
    if (isMotionActive())
    {
        Serial.println(
            "ERROR: Cannot start blocking move - another move is active."
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

        // Prevent an unnecessarily tight busy loop.
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
            "ERROR: Cannot start blocking move - another move is active."
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