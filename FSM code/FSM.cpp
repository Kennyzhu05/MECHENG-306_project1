#include "FSM.h"

FSM::FSM()
{
    currentState = State::IDLE;
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
        case State::IDLE:
            std::cout << "Entering IDLE\n";
            //Serial.println("Entering IDLE");
            //restore the serial print while testing on Arduino. 
            break;

        case State::HOMING:
            std::cout << "Entering HOMING\n";
            //Serial.println("Entering HOMING");
            //restore the serial print while testing on Arduino. 
        

            break;

        case State::MOVING:
            std::cout << "Entering MOVING\n";
            //Serial.println("Entering MOVING");
            //restore the serial print while testing on Arduino. 
    

            break;

        case State::FAULT:

            std::cout << "Entering FAULT\n";
            //Serial.println("Entering FAULT");
            //restore the serial print while testing on Arduino. 

            break;
    }
}

void FSM::exitState(State state)
{
    switch(state)
    {
        case State::IDLE:
            std::cout << "Leaving IDLE\n";
            //Serial.println("Leaving IDLE");
            //restore the serial print while testing on Arduino.

            break;

        case State::HOMING:
            std::cout << "Leaving HOMING\n";
            //Serial.println("Leaving HOMING");
            //restore the serial print while testing on Arduino.

            break;

        case State::MOVING:
            std::cout << "Leaving MOVING\n";
            //Serial.println("Leaving MOVING");
            //restore the serial print while testing on Arduino.

            break;

        case State::FAULT:
            std::cout << "Leaving FAULT\n";
            //Serial.println("Leaving FAULT");
            //restore the serial print while testing on Arduino.

            break;
    }
}


    //--------------------------------
    // Initially placeholders
    //--------------------------------

void FSM::idleUpdate()
{
}

void FSM::homingUpdate()
{
}

void FSM::movingUpdate()
{
}

void FSM::faultUpdate()
{
}

void FSM::processEvent(Event event){
    switch(currentState)
    {
        case State::IDLE:

            if(event == Event::G28_RECEIVED)
                changeState(State::HOMING);

            else if(event == Event::G1_RECEIVED)
                changeState(State::MOVING);

            break;

        case State::HOMING:

            if(event == Event::HOME_COMPLETE)
                changeState(State::IDLE);

            else if(event == Event::LIMIT_TRIGGERED)
                changeState(State::FAULT);

            break;

        case State::MOVING:

            if(event == Event::MOVE_COMPLETE)
                changeState(State::IDLE);

            else if(event == Event::LIMIT_TRIGGERED)
                changeState(State::FAULT);

            else if(event == Event::TIMEOUT)
                changeState(State::FAULT);

            break;

        case State::FAULT:

            if(event == Event::RESET)
                changeState(State::IDLE);

            break;
    }
}