#include "FSM.h"

FSM fsm;

//--------------------------------------------------
// Convert State enum to text
//--------------------------------------------------

const char* stateToString(State state)
{
    switch (state)
    {
        case State::INITIALIZING: return "INITIALIZING";
        case State::IDLE:         return "IDLE";
        case State::HOMING:       return "HOMING";
        case State::MOVING:       return "MOVING";
        case State::FAULT:        return "FAULT";
        default:                  return "UNKNOWN";
    }
}
//--------------------------------------------------
// Print command list
//--------------------------------------------------

void printHelp()
{
    Serial.println();
    Serial.println("========== FSM TEST ==========");
    Serial.println("Available commands:");
    Serial.println("INIT_OK");
    Serial.println("INIT_FAIL");
    Serial.println("G28");
    Serial.println("G1");
    Serial.println("HOME_DONE");
    Serial.println("MOVE_DONE");
    Serial.println("LIMIT");
    Serial.println("TIMEOUT");
    Serial.println("RESET");
    Serial.println("STATE");
    Serial.println("HELP");
    Serial.println("==============================");
    Serial.println();
}



void setup() {
   Serial.begin(115200);

    while (!Serial);

    fsm.begin();

    Serial.println();
    Serial.println("===== FSM Test Started =====");

    printHelp();

    Serial.print("Current State: ");
    Serial.println(stateToString(fsm.getCurrentState()));

}

void loop() {
  if (Serial.available())
    {
        String command = Serial.readStringUntil('\n');

        command.trim();

        command.toUpperCase();

        //--------------------------------------

        if (command == "INIT_SUCCESS")
        {
            fsm.processEvent(Event::INIT_SUCCESS);
        }

        else if (command == "INIT_FAIL")
        {
            fsm.processEvent(Event::INIT_FAILED);
        }

        else if (command == "G28")
        {
            fsm.processEvent(Event::G28_RECEIVED);
        }

        else if (command == "G1")
        {
            fsm.processEvent(Event::G1_RECEIVED);
        }

        else if (command == "HOME_DONE")
        {
            fsm.processEvent(Event::HOME_COMPLETE);
        }

        else if (command == "MOVE_DONE")
        {
            fsm.processEvent(Event::MOVE_COMPLETE);
        }

        else if (command == "LIMIT")
        {
            fsm.processEvent(Event::LIMIT_TRIGGERED);
        }

        else if (command == "TIMEOUT")
        {
            fsm.processEvent(Event::TIMEOUT);
        }

        else if (command == "RESET")
        {
            fsm.processEvent(Event::RESET);
        }

        else if (command == "STATE")
        {
            // Just print state
        }

        else if (command == "HELP")
        {
            printHelp();
        }

        else
        {
            Serial.println("Unknown command.");
        }

        //--------------------------------------

        // fsm.update();

        Serial.print("Current State: ");

        Serial.println(stateToString(fsm.getCurrentState()));

        Serial.println();
    }

}
