#ifndef LINEAR_MOTION_H
#define LINEAR_MOTION_H


// =======================================================
// DATA STRUCTURES
// =======================================================

// Final X-Y coordinate requested by G1
struct MotionCommand
{
    float targetX_mm;
    float targetY_mm;
};


// Instantaneous reference position along the straight line
struct MotionReference
{
    float x_mm;
    float y_mm;

    // Equivalent motor displacement coordinates
    float motor1_mm;
    float motor2_mm;
};


// =======================================================
// CONFIGURATION
// =======================================================

/*
 * Configure the linear motion generator.
 *
 * speed_mm_s:
 *      Desired Cartesian travel speed.
 *
 * tolerance_mm:
 *      How close the measured pen position must be
 *      to the target before MOVE_COMPLETE is declared.
 *
 * timeoutMargin_ms:
 *      Additional time allowed after the planned trajectory
 *      has finished before declaring a timeout.
 *
 * No arbitrary defaults are used. The values must be chosen
 * explicitly by the team.
 */
bool configureLinearMotion(
    float speed_mm_s,
    float tolerance_mm,
    unsigned long timeoutMargin_ms
);


// =======================================================
// TARGET COMMAND
// =======================================================

// Store the target produced by the G-code parser.
void setLinearTarget(
    float targetX_mm,
    float targetY_mm
);

MotionCommand getLinearTarget();

bool linearTargetAvailable();

void clearLinearTarget();


// =======================================================
// MOVE CONTROL
// =======================================================

// Called once when MOVING begins.
bool startLinearMove(
    float currentX_mm,
    float currentY_mm
);

// Called repeatedly while the FSM is in MOVING.
void updateLinearMove(
    float currentX_mm,
    float currentY_mm
);


// =======================================================
// OUTPUT / STATUS
// =======================================================

MotionReference getLinearReference();

bool linearMoveActive();

bool linearMoveComplete();

bool linearMoveTimedOut();

#endif