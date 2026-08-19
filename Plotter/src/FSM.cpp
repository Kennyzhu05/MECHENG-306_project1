#include "FSM.h"
#include "Homing.h"
#include "Motion.h"
#include "Motor.h"
#include "Switch.h"
#include "GcodeParser.h"
#include "Helper.h"

// Start with initialzing state
FSM::FSM()
{
    currentState = State::INITIALIZING;
}

void FSM::begin()
{
    enterState(currentState);
}

void FSM::update()
{
    updateState();
}

State FSM::getCurrentState() const
{
    return currentState;
}

bool FSM::isIdle() const
{
    if (currentState == State::IDLE)
    {
        return true;
    }

    return false;
}

bool FSM::isBusy() const
{
    if (currentState == State::MOVING ||
        currentState == State::HOMING ||
        currentState == State::INITIALIZING)
    {
        return true;
    }

    return false;
}

bool FSM::isFault() const
{
    return currentState == State::FAULT;
}

void FSM::updateState()
{
    switch(currentState)
    {
        case State::IDLE:

            idleUpdate();

            break;

        case State::HOMING:

            homingUpdate();

            break;

        case State::MOVING:

            movingUpdate();

            break;

        case State::FAULT:

            faultUpdate();

            break;
    }
}

void FSM::changeState(State newState)
{
    if(currentState == newState)
        return;

    exitState(currentState);

    currentState = newState;

    enterState(currentState);
}

void FSM::enterState(State state)
{
    switch(state)
    {
        case State::INITIALIZING:

            Serial.println("Entering INITIALIZING:");

            break;

        case State::IDLE:

            Serial.println("Entering IDLE");

            break;

        case State::HOMING:

            Serial.println("Entering HOMING");

            break;

        case State::MOVING:

            Serial.println("Entering MOVING");

            break;

        case State::FAULT:

            Serial.println("Entering FAULT");

            break;
    }
}

void FSM::exitState(State state)
{
    switch(state)
    {
        case State::INITIALIZING:

            Serial.println("Leaving INITIALIZING:");

            break;

        case State::IDLE:

            Serial.println("Leaving IDLE");

            break;

        case State::HOMING:

            Serial.println("Leaving HOMING");

            break;

        case State::MOVING:

            Serial.println("Leaving MOVING");

            // Make sure motors are stopped and
            // the motion controller is reset when
            // leaving the MOVING state.
            abortMotion();

            break;

        case State::FAULT:

            Serial.println("Leaving FAULT");

            break;
    }
}


//--------------------------------
// State update functions
//--------------------------------

void FSM::idleUpdate()
{
}

void FSM::homingUpdate()
{
    updateHoming();

    if (isHomingComplete())
    {
        processEvent(Event::HOME_COMPLETE);
    }
}

void FSM::movingUpdate()
{
    // Continue the movement that was started
    // when G1_RECEIVED was processed.
    updateMotion();

    // If the requested position has been reached,
    // return to IDLE.
    if (isMotionDone())
    {
        processEvent(Event::MOVE_COMPLETE);
        return;
    }

    // If the movement takes too long,
    // move the FSM into FAULT.
    if (isMotionTimedOut())
    {
        processEvent(Event::TIMEOUT);
        return;
    }
}

void FSM::faultUpdate()
{
    stopMotors();
    updateLimitSwitches();
}


//--------------------------------
// Event processing
//--------------------------------

void FSM::processEvent(Event event)
{
    switch(currentState)
    {
        case State::INITIALIZING:

            if (event == Event::INIT_SUCCESS)
            {
                changeState(State::HOMING);
            }

            else if (event == Event::INIT_FAILED)
            {
                changeState(State::FAULT);
            }

            break;


        case State::IDLE:

            if(event == Event::G28_RECEIVED)
            {
                changeState(State::HOMING);

                startHoming();
            }

            else if(event == Event::G1_RECEIVED)
            {
                long targetX = gcodeParser.getTargetX();
                long targetY = gcodeParser.getTargetY();

                changeState(State::MOVING);

                startMotion(mmToXCounts(targetX), mmToYCounts(targetY));
            }

            // else if (event == Event::LIMIT_TRIGGERED)
            // {
            //     changeState(State::FAULT);
            // }

            break;


        case State::HOMING:

            if(event == Event::HOME_COMPLETE)
            {
                changeState(State::IDLE);
            }

            else if(event == Event::HOMING_ERROR)
            {
                changeState(State::FAULT);
            }

            else if (event == Event::TIMEOUT)
            {
                changeState(State::FAULT);
            }

            break;


        case State::MOVING:

            if(event == Event::MOVE_COMPLETE)
            {
                changeState(State::IDLE);
            }

            else if(event == Event::LIMIT_TRIGGERED)
            {
                changeState(State::FAULT);
            }

            else if(event == Event::TIMEOUT)
            {
                changeState(State::FAULT);
            }

            break;


        case State::FAULT:

            if(event == Event::RESET)
            {
                changeState(State::IDLE);
            }

            // Or maybe return to INITIALIZATION

            break;
    }
}