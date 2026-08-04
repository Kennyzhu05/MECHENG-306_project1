#ifndef EVENT_H
#define EVENT_H

enum class Event
{
    NONE,

    G28_RECEIVED,
    G1_RECEIVED,

    HOME_COMPLETE,
    MOVE_COMPLETE,

    LIMIT_TRIGGERED,

    ENCODER_ERROR,

    TIMEOUT,

    RESET
};

#endif