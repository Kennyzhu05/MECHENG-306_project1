#include "LinearMotion.h"

// The currently stored G1 target
static MotionCommand activeCommand = {0.0f, 0.0f};

// Whether a valid command is currently available
static bool commandAvailable = false;

void setLinearTarget(float targetX_mm, float targetY_mm)
{
    activeCommand.targetX_mm = targetX_mm;
    activeCommand.targetY_mm = targetY_mm;

    commandAvailable = true;
}

MotionCommand getLinearTarget()
{
    return activeCommand;
}

bool linearTargetAvailable()
{
    return commandAvailable;
}

void clearLinearTarget()
{
    commandAvailable = false;
}