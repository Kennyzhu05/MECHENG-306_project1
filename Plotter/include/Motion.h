#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

// ---- Tunable PI gains and limits (still adjustable at runtime) ----
extern float KP;
extern float KI;
extern int MIN_PWM;
extern int MAX_PWM;
extern long POSITION_TOLERANCE;
extern int SETTLE_SAMPLES;
extern unsigned long MOVE_TIMEOUT_MS;

// Result of the most recently completed move, so the FSM can tell a
// clean settle apart from a safety timeout.
enum class MotionResult
{
    NONE,       // no move has completed yet since startMotion()
    SETTLED,    // both axes reached tolerance and stayed there
    TIMED_OUT   // safety cutoff hit before settling
};

// Begin a new move to the given target encoder counts (X/Y space).
// Resets encoders, integrators, timers, and the settle counter.
void startMotion(long targetXCounts, long targetYCounts);

// Call this every loop() iteration. Does ONE PI update tick if a
// motion is active; does nothing if idle. Non-blocking.
void updateMotion();

// True once updateMotion() has settled or timed out (i.e. motors
// are no longer being actively driven toward a target).
bool isMotionDone();

// True while a motion is actively running (equivalent to !isMotionDone()
// but reads more naturally from FSM code, e.g. isBusy()).
bool isMotionActive();

// True if the current motion has exceeded the safety timeout.
bool isMotionTimedOut();

// Why the last motion ended. Valid once isMotionDone() is true.
MotionResult getLastMotionResult();

// Immediately halts any active motion and stops the motors.
// Use for e-stop / limit-switch triggers, not for normal completion.
void abortMotion();

// Blocking convenience wrapper kept for standalone bench testing
// (bypasses the FSM). Internally just spins calling updateMotion().
void moveXY(long targetXCounts, long targetYCounts);
void moveX(long targetCounts);
void moveY(long targetCounts);

#endif