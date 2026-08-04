#ifndef FSM_H
#define FSM_H


#include <iostream>
#include "State.h"
#include "Event.h"

class FSM
{
public:

    FSM();

    void begin();

    void update();

    void processEvent(Event event);

    bool isIdle() const;

    bool isBusy() const;

    bool isFault() const;

    State getCurrentState() const;

    
    
private:

    State currentState;

    //--------------------------------
    // State transition
    //--------------------------------

    void changeState(State newState);

    //--------------------------------
    // State lifecycle
    //--------------------------------

    void enterState(State state);

    void updateState();

    void exitState(State state);

    //--------------------------------
    // Individual state updates
    //--------------------------------

    void idleUpdate();

    void homingUpdate();

    void movingUpdate();

    void faultUpdate();

    //--------------------------------
    // Event processing
    //--------------------------------

   
};

#endif