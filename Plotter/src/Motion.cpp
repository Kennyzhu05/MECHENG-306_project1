#include <Arduino.h>
#include "Motion.h"
#include "Motor.h"
#include "Encoder.h"

// ---- Tunable PI gains and limits ----
float KP = 0.8;
float KI = 0.05;
int MIN_PWM = 60;      // below this, motors may not overcome static friction
int MAX_PWM = 255;
long POSITION_TOLERANCE = 5;   // encoder counts considered "close enough"
int SETTLE_SAMPLES = 5;        // consecutive in-tolerance loops before stopping
unsigned long MOVE_TIMEOUT_MS = 8000;   // safety cutoff

// ---- Persistent motion state (replaces the old while-loop locals) ----
static long targetA = 0;
static long targetB = 0;
static float integralA = 0;
static float integralB = 0;
static unsigned long moveStartTime = 0;
static unsigned long lastTickTime = 0;
static int settledCount = 0;
static bool motionActive = false;
static MotionResult lastResult = MotionResult::NONE;

static int clampInt(int value, int lo, int hi)
{
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

// Converts a raw PI output into a signed PWM command: capped at
// MAX_PWM, and bumped up to MIN_PWM if it's small-but-nonzero so the
// motor doesn't stall in the "should be moving but isn't" zone.
static int toMotorCommand(float piOutput)
{
    int pwm = (int)piOutput;
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

void startMotion(long targetXCounts, long targetYCounts)
{
    // Kinematics: convert the desired X/Y move into per-motor targets
    targetA = targetXCounts + targetYCounts;
    targetB = targetXCounts - targetYCounts;

    resetEncoders();

    integralA = 0;
    integralB = 0;
    settledCount = 0;

    moveStartTime = millis();
    lastTickTime = moveStartTime;

    lastResult = MotionResult::NONE;
    motionActive = true;
}

void abortMotion()
{
    motionActive = false;
    stopMotors();
}

bool isMotionActive()
{
    return motionActive;
}

bool isMotionTimedOut()
{
    if (!motionActive)
    {
        return false;
    }

    unsigned long now = millis();
    return (now - moveStartTime >= MOVE_TIMEOUT_MS);
}

bool isMotionDone()
{
    return !motionActive;
}

MotionResult getLastMotionResult()
{
    return lastResult;
}

// Runs exactly one PI iteration. This is the old while-loop body,
// unchanged in its control logic -- only the looping mechanism moved
// out to the caller (loop() calls this repeatedly instead of a
// blocking while(true) here).
void updateMotion()
{
    if (!motionActive)
    {
        return;
    }

    updateEncoders();

    unsigned long now = millis();
    float dt = (now - lastTickTime) / 1000.0;
    if (dt <= 0) dt = 0.001;   // guard against a zero dt on fast loops
    lastTickTime = now;

    long errorA = targetA - getLeftEncoderCount();
    long errorB = targetB - getRightEncoderCount();

    // --- Motor A ---
    float outputA = KP * errorA + KI * integralA;
    int pwmA = toMotorCommand(outputA);
    bool satPosA = (outputA > MAX_PWM && errorA > 0);
    bool satNegA = (outputA < -MAX_PWM && errorA < 0);
    if (!satPosA && !satNegA)
    {
        integralA += errorA * dt;
    }

    // --- Motor B ---
    float outputB = KP * errorB + KI * integralB;
    int pwmB = toMotorCommand(outputB);
    bool satPosB = (outputB > MAX_PWM && errorB > 0);
    bool satNegB = (outputB < -MAX_PWM && errorB < 0);
    if (!satPosB && !satNegB)
    {
        integralB += errorB * dt;
    }

    // Once a motor is within tolerance, stop driving it so it
    // doesn't hunt back and forth while the other axis finishes
    if (abs(errorA) <= POSITION_TOLERANCE) pwmA = 0;
    if (abs(errorB) <= POSITION_TOLERANCE) pwmB = 0;

    driveMotorA(pwmA);
    driveMotorB(pwmB);

    // Both motors settled -> done
    if (abs(errorA) <= POSITION_TOLERANCE && abs(errorB) <= POSITION_TOLERANCE)
    {
        settledCount++;
        if (settledCount >= SETTLE_SAMPLES)
        {
            stopMotors();
            motionActive = false;
            lastResult = MotionResult::SETTLED;
            return;
        }
    }
    else
    {
        settledCount = 0;
    }

    if (now - moveStartTime >= MOVE_TIMEOUT_MS)
    {
        // Safety timeout: stall, disconnected encoder, sign inversion, etc.
        stopMotors();
        motionActive = false;
        lastResult = MotionResult::TIMED_OUT;
        return;
    }
}

// ---- Blocking wrapper kept for bench testing outside the FSM ----
void moveXY(long targetXCounts, long targetYCounts)
{
    startMotion(targetXCounts, targetYCounts);
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