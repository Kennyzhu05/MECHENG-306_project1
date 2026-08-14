#ifndef MOTION_H
#define MOTION_H


// =======================================================
// CONTROLLER SETTINGS
// =======================================================

extern float KP;
extern float KI;

// Synchronisation gain used to keep both motors coordinated.
extern float SYNC_KP;


// =======================================================
// PWM LIMITS
// =======================================================

extern int MIN_PWM;
extern int MAX_PWM;


// =======================================================
// STOPPING CONDITIONS
// =======================================================

extern long POSITION_TOLERANCE;
extern int SETTLE_SAMPLES;


// =======================================================
// SAFETY
// =======================================================

extern unsigned long MOVE_TIMEOUT_MS;


// =======================================================
// ENCODER CALIBRATION
// =======================================================

extern const float X_COUNTS_PER_MM;
extern const float Y_COUNTS_PER_MM;


// Convert Cartesian millimetres into average encoder counts.
long xMmToCounts(float distanceMm);
long yMmToCounts(float distanceMm);


// =======================================================
// NON-BLOCKING FSM MOTION INTERFACE
// =======================================================

// Store a relative XY target in millimetres.
// This function stores the target but does not start movement.
void setMotionTargetMm(
    float targetXmm,
    float targetYmm
);


// Start the previously stored target.
// Returns false if no target has been stored.
bool startMotion();


// Perform one iteration of encoder reading and PI control.
// Call this repeatedly from FSM::movingUpdate().
void updateMotion();


// Motion status functions
bool isMotionActive();
bool isMotionComplete();
bool hasMotionTimedOut();


// Stop the motors and reset the motion controller.
void resetMotion();


// =======================================================
// LEGACY BLOCKING INTERFACE
// =======================================================

/*
 * These functions perform the complete movement before
 * returning.
 *
 * Do not call them from FSM::movingUpdate().
 */

// Raw encoder-count movement
void moveXY(
    long targetXCounts,
    long targetYCounts
);

void moveX(long targetCounts);
void moveY(long targetCounts);


// Calibrated millimetre movement
void moveXYmm(
    float targetXmm,
    float targetYmm
);

void moveXmm(float targetMm);
void moveYmm(float targetMm);


#endif