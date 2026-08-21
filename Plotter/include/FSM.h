#ifndef FSM_H
#define FSM_H


#include "Arduino.h"
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

    void initializingUpdate();

    bool motorsReady();
    
    bool switchesReady();

    bool encoderReady();

    //--------------------------------
    // Event processing
    //--------------------------------

    // Store the initial states of the switches before it starts moving
    bool startTopLimitActive = false;
    bool startBottomLimitActive = false;
    bool startLeftLimitActive = false;
    bool startRightLimitActive = false;
};

extern FSM fsm;

#endif