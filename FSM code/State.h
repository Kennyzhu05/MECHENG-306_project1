#ifndef STATE_H
#define STATE_H

enum class State
{
    INITIALIZING,
    IDLE,
    HOMING,
    MOVING,
    FAULT
};

#endif