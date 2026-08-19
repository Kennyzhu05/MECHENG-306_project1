#ifndef EVENT_H
#define EVENT_H

enum class Event
{
    NONE,

    // System startup
    INIT_SUCCESS,
    INIT_FAILED,

    // Commands
    G28_RECEIVED,
    G1_RECEIVED,

    // Motion feedback
    HOME_COMPLETE,
    MOVE_COMPLETE,

    // Faults
    HOMING_ERROR,
    LIMIT_TRIGGERED,
    ENCODER_ERROR,
    TIMEOUT, 

    // Recovery
    RESET
};

#endif