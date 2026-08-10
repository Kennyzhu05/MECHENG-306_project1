#ifndef LINEAR_MOTION_H
#define LINEAR_MOTION_H

// Information required for one G1 movement
struct MotionCommand
{
    float targetX_mm;
    float targetY_mm;
};

// Store a new target
void setLinearTarget(float targetX_mm, float targetY_mm);

// Read the currently stored target
MotionCommand getLinearTarget();

// Check whether a target has been stored
bool linearTargetAvailable();

// Remove the target after movement is complete
void clearLinearTarget();

#endif