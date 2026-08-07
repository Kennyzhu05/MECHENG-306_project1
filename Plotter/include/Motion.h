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

// Move the platform by targetXCounts / targetYCounts (encoder counts
// along the X and Y axes). Converts to per-motor targets via the
// kinematics (targetA = targetX + targetY, targetB = targetX - targetY)
// and drives each motor independently under closed-loop PI control, so
// the two motors stay synchronized even if their speeds don't match.
void moveXY(long targetXCounts, long targetYCounts);

// Convenience wrappers for single-axis moves
void moveX(long targetCounts);
void moveY(long targetCounts);

#endif