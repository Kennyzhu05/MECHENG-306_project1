#ifndef MOTION_H
#define MOTION_H

// ---- PI gains (tune for your platform) ----
extern float KP;
extern float KI;

// ---- PWM limits ----
extern int MIN_PWM;   // minimum PWM to overcome static friction
extern int MAX_PWM;   // maximum output speed cap

// ---- Stopping criteria ----
extern long POSITION_TOLERANCE;  // encoder counts considered "close enough"
extern int SETTLE_SAMPLES;       // consecutive in-tolerance loops before stopping

// ---- Safety ----
extern unsigned long MOVE_TIMEOUT_MS;

// ---- Experimentally measured encoder scale ----
// Boundary-test results:
//   X: 20042.1 Cartesian counts over 220 mm
//   Y: 12339.6 Cartesian counts over 130 mm
extern const float X_COUNTS_PER_MM;
extern const float Y_COUNTS_PER_MM;

// Convert a Cartesian distance in millimetres into encoder counts.
long xMmToCounts(float distanceMm);
long yMmToCounts(float distanceMm);

// Move the platform by targetXCounts / targetYCounts (encoder counts
// along the X and Y axes). Converts to per-motor targets via the
// kinematics (targetA = targetX + targetY, targetB = targetX - targetY)
// and drives each motor independently under closed-loop PI control, so
// the two motors stay synchronized even if their speeds don't match.
void moveXY(long targetXCounts, long targetYCounts);

// Convenience wrappers for single-axis moves
void moveX(long targetCounts);
void moveY(long targetCounts);

// Millimetre versions. These convert X and Y separately using the measured
// axis scales above, then call the existing encoder-count motion functions.
// Distances are relative to the position at which the function is called.
void moveXYmm(float targetXmm, float targetYmm);
void moveXmm(float targetMm);
void moveYmm(float targetMm);

#endif