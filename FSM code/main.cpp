#include <iostream>
#include <string>

#include "FSM.h"
const char* stateToString(State state)
{
    switch (state)
    {
        case State::IDLE:    return "IDLE";
        case State::HOMING:  return "HOMING";
        case State::MOVING:  return "MOVING";
        case State::FAULT:   return "FAULT";
        default:             return "UNKNOWN";
    }
}

int main()
{
    FSM fsm;

    fsm.begin();

    std::string command;

    while (true)
    {
        std::cout << "\nCurrent State: "
                  << stateToString(fsm.getCurrentState())
                  << '\n';

        std::cout << "Command> ";

        std::getline(std::cin, command);

        if (command == "quit")
            break;

        if (command == "G28")
            fsm.processEvent(Event::G28_RECEIVED);

        else if (command == "G1")
            fsm.processEvent(Event::G1_RECEIVED);

        else if (command == "HOME_DONE")
            fsm.processEvent(Event::HOME_COMPLETE);

        else if (command == "MOVE_DONE")
            fsm.processEvent(Event::MOVE_COMPLETE);

        else if (command == "LIMIT")
            fsm.processEvent(Event::LIMIT_TRIGGERED);

        else if (command == "RESET")
            fsm.processEvent(Event::RESET);

        else
            std::cout << "Unknown command.\n";

        fsm.update();
    }

    return 0;
}