#include "FSM.h"

FSM fsm;

void setup()
{
    Serial.begin(115200);

    fsm.begin();

    Serial.println("FSM Test");
    Serial.println("--------------------");
}

void loop()
{
    if (Serial.available())
    {
        char c = Serial.read();

        switch (c)
        {
        case 'i':

            fsm.processEvent(Event::INIT_SUCCESS);

            break;

        case 'f':

            fsm.processEvent(Event::INIT_FAILED);

            break;

        case 'h':

            fsm.processEvent(Event::G28_RECEIVED);

            break;

        case 'm':

            fsm.processEvent(Event::G1_RECEIVED);

            break;

        case 'H':

            fsm.processEvent(Event::HOME_COMPLETE);

            break;

        case 'M':

            fsm.processEvent(Event::MOVE_COMPLETE);

            break;

        case 'l':

            fsm.processEvent(Event::LIMIT_TRIGGERED);

            break;

        case 't':

            fsm.processEvent(Event::TIMEOUT);

            break;

        case 'r':

            fsm.processEvent(Event::RESET);

            break;
        }

        Serial.print("Current State: ");

        Serial.println(stateToString(fsm.getCurrentState()));
    }

    fsm.update();
}