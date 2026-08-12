#ifndef GCODEPARSER_H
#define GCODEPARSER_H
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include "FSM.h"

class GCodeParser
{
public:
    // Constructor
    GCodeParser(FSM& fsmReference);

    // Call continuously inside loop()
    void updateSerialCommands();

    // Get parsed G1 values
    float getTargetX() const;
    float getTargetY() const;
    float getTargetFeedRate() const;

private:
    // Reference to the main FSM object
    FSM& fsm;

    // -----------------------------
    // Serial command buffer
    // -----------------------------

    static constexpr size_t COMMAND_BUFFER_SIZE = 64;

    char commandBuffer[COMMAND_BUFFER_SIZE];
    size_t commandIndex;

    // -----------------------------
    // Parsed G1 data
    // -----------------------------

    float targetX;
    float targetY;

    float lastFeedRate;
    float targetFeedRate;

    // -----------------------------
    // Internal parsing functions
    // -----------------------------

    void parseGCode(char* command);

    bool parseG1(const char* command);

    bool parseNumber(
        const char*& cursor,
        float& result
    );
};

#endif