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
        case State::INITIALIZING:

            initializingUpdate();

            break;

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
// State readiness checks during INITIALIZING
//--------------------------------
bool FSM::motorsReady()
{
    // Check motor driver / motor control initialization
    return true;
}

bool FSM::switchesReady()
{
    // Check limit switches are configured
    return true;
}

bool FSM::encoderReady()
{
    // Check encoder is giving a valid reading
    return true;
}


//--------------------------------
// State update functions
//--------------------------------
void FSM::initializingUpdate()
{
    if (motorsReady() &&
        switchesReady() &&
        encoderReady())
    {
        processEvent(Event::INIT_SUCCESS);
    }
    else
    {
        processEvent(Event::INIT_FAILED);
    }
}

void FSM::idleUpdate()
{
    updateLimitSwitches();
}

void FSM::homingUpdate()
{
   
    // Always get the latest physical switch states
    updateLimitSwitches();


    // =================================================
    // 1. RE-ARM switches that were active at homing start
    // =================================================

    if (startRightLimitActive && !rightLimitReached())
    {
        startRightLimitActive = false;
        Serial.println("RIGHT limit re-armed during homing");
    }

    if (startTopLimitActive && !topLimitReached())
    {
        startTopLimitActive = false;
        Serial.println("TOP limit re-armed during homing");
    }


    // =================================================
    // 2. Check for WRONG limit switches during homing
    // =================================================
    //
    // Assuming homing moves toward LEFT + BOTTOM:
    //
    // LEFT   = expected
    // BOTTOM = expected
    // RIGHT  = FAULT
    // TOP    = FAULT
    //
    // If RIGHT/TOP was already pressed when homing started,
    // ignore it until it has first been released.

    if (rightLimitReached() && !startRightLimitActive)
    {
        Serial.println("HOMING ERROR: RIGHT limit triggered");

        stopMotors();
        processEvent(Event::HOMING_ERROR);
        return;
    }

    if (topLimitReached() && !startTopLimitActive)
    {
        Serial.println("HOMING ERROR: TOP limit triggered");

        stopMotors();
        processEvent(Event::HOMING_ERROR);
        return;
    }


    // =================================================
    // 3. No fault -> continue normal homing
    // =================================================

    updateHoming();


    // =================================================
    // 4. Check whether normal homing has completed
    // =================================================

    if (isHomingComplete())
    {
        processEvent(Event::HOME_COMPLETE);
        return;
    }
}

void FSM::movingUpdate()
{   
    // The following section of the code is responsible to check whether
    // the previous pressed switch has been released

    // Always refresh the physical switch states first
    updateLimitSwitches();


    // =================================================
    // 1. RE-ARM switches that were active at move start
    // This means: now the switch is released, meaning that
    // the plotter has left the starting point where the switch
    // was pressed. So if the switch is triggered, it is
    // indicating an error. 
    // =================================================
    //
    // start_____LimitActive == true means:
    // "This switch was already pressed when motion began,
    //  so ignore it until it gets released."
    //
    // Once released, set the flag false.
    // From then on it is armed again.

    if (startLeftLimitActive && !leftLimitReached())
    {
        startLeftLimitActive = false;
        Serial.println("LEFT limit re-armed");
    }

    if (startRightLimitActive && !rightLimitReached())
    {
        startRightLimitActive = false;
        Serial.println("RIGHT limit re-armed");
    }

     if (startBottomLimitActive && !bottomLimitReached())
    {
        startBottomLimitActive = false;
        Serial.println("BOTTOM limit re-armed");
    }

    if (startTopLimitActive && !topLimitReached())
    {
        startTopLimitActive = false;
        Serial.println("TOP limit re-armed");
    }

    // =================================================
    // 2. Check all ARMED switches
    // =================================================
    //
    // If:
    //      switch is physically pressed
    // AND
    //      it is NOT one of the initially ignored switches
    //
    // then this is a new limit hit -> FAULT.

    if (leftLimitReached() && !startLeftLimitActive)
    {
        Serial.println("FAULT: LEFT limit triggered");

        processEvent(Event::LIMIT_TRIGGERED);
        return;
    }

    if (rightLimitReached() && !startRightLimitActive)
    {
        Serial.println("FAULT: RIGHT limit triggered");

        processEvent(Event::LIMIT_TRIGGERED);
        return;
    }

    if (bottomLimitReached() && !startBottomLimitActive)
    {
        Serial.println("FAULT: BOTTOM limit triggered");

        processEvent(Event::LIMIT_TRIGGERED);
        return;
    }

    if (topLimitReached() && !startTopLimitActive)
    {
        Serial.println("FAULT: TOP limit triggered");

        processEvent(Event::LIMIT_TRIGGERED);
        return;
    }

    // =================================================
    // 3. No limit fault -> continue normal motion
    // =================================================
    // Continue the movement that was started
    // when G1_RECEIVED was processed.
    updateMotion();

    // Check whether Motion.cpp rejected the target
    if (getLastMotionResult() == MotionResult::INVALID_TARGET)
    {
        Serial.println("FAULT: Invalid motion target");
        processEvent(Event::LIMIT_TRIGGERED); // Use LIMIT_TRIGGERED to enter FAULT state
        return;
    }

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
    
    // =====================================================
    // GLOBAL M999 OVERRIDE
    // =====================================================

    if (event == Event::M999_RECEIVED)
    {
        Serial.println(
            "M999: stopping current operation"
        );

        // Immediately stop any motor movement
        abortMotion();

        // Return to IDLE regardless of current state
        if (currentState != State::IDLE)
        {
            changeState(State::IDLE);
        }

        return;
    }

    switch(currentState)
    {
        case State::INITIALIZING:

            if (event == Event::INIT_SUCCESS)
            {
                updateLimitSwitches();
                startTopLimitActive = topLimitReached();
                startBottomLimitActive = bottomLimitReached();
                startLeftLimitActive = leftLimitReached();
                startRightLimitActive = rightLimitReached();
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
                if(event == Event::G28_RECEIVED)
                   if(event == Event::G28_RECEIVED)
                    {
                        // Get current physical switch states
                        updateLimitSwitches();

                        // Remember which switches were already pressed
                        // BEFORE homing starts
                        startTopLimitActive = topLimitReached();
                        startBottomLimitActive = bottomLimitReached();
                        startLeftLimitActive = leftLimitReached();
                        startRightLimitActive = rightLimitReached();

                        changeState(State::HOMING);

                        startHoming();
                    }
            }

            else if(event == Event::G1_RECEIVED)
            {
                long targetX = gcodeParser.getTargetX();
                long targetY = gcodeParser.getTargetY();
                long targetFeedRate = gcodeParser.getTargetFeedRate();
                            
                            // Make sure we are checking the latest switch states
                updateLimitSwitches();

                // -------------------------------------------------
                // Do not allow motion further into a pressed limit
                // -------------------------------------------------

                // Left limit already pressed -> cannot move left
                if (leftLimitReached() && targetX < 0)
                {
                    Serial.println("G1 rejected: LEFT limit already reached");
                    return;
                }

                // Right limit already pressed -> cannot move right
                if (rightLimitReached() && targetX > 0)
                {
                    Serial.println("G1 rejected: RIGHT limit already reached");
                    return;
                }

                // Bottom limit already pressed -> cannot move down
                if (bottomLimitReached() && targetY < 0)
                {
                    Serial.println("G1 rejected: BOTTOM limit already reached");
                    return;
                }

                // Top limit already pressed -> cannot move up
                if (topLimitReached() && targetY > 0)
                {
                    Serial.println("G1 rejected: TOP limit already reached");
                    return;
                }
                
                // ----------------------------------------
                // Record if the switches are triggered BEFORE movement
                // ----------------------------------------

                startTopLimitActive = topLimitReached();
                startBottomLimitActive = bottomLimitReached();
                startLeftLimitActive = leftLimitReached();
                startRightLimitActive = rightLimitReached();

                // If none of the above unsafe cases occurred,
                // the movement is allowed.
                changeState(State::MOVING);

                startMotion(mmToXCounts(targetX), mmToYCounts(targetY), targetFeedRate);
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