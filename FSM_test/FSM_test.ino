#include "FSM.h"
#include "FSM.h"
#include "LinearMotion.h"

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
        {

            // Temporary test target:
            // Equivalent to receiving G1 X50 Y30
            setLinearTarget(50.0f, 30.0f);

            // Tell the FSM that the G1 command is ready
            fsm.processEvent(Event::G1_RECEIVED);

            MotionCommand command = getLinearTarget();

            Serial.print("Target X: ");
            Serial.print(command.targetX_mm);
            Serial.print(" mm, Target Y: ");
            Serial.print(command.targetY_mm);
            Serial.println(" mm");

            break;
        }

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