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

    // Check queue and execute next command when FSM is IDLE
    void updateCommandQueue();

    // Get parsed G1 values
    float getTargetX() const;
    float getTargetY() const;
    float getTargetFeedRate() const;


private:

    // Reference to the main FSM object
    FSM& fsm;


    // =====================================================
    // Serial command buffer
    // =====================================================

    static constexpr size_t COMMAND_BUFFER_SIZE = 64;

    char commandBuffer[COMMAND_BUFFER_SIZE];
    size_t commandIndex;


    // =====================================================
    // Parsed G1 data
    // =====================================================

    float targetX;
    float targetY;

    float lastFeedRate;
    float targetFeedRate;


    // =====================================================
    // G1 command queue
    // =====================================================

    /*
      Each queued G1 command stores:

      X offset
      Y offset
      Feed rate
    */
    struct G1Command
    {
        float x;
        float y;
        float f;
    };


    /*
      Fixed-size circular queue.

      Arduino memory is limited, so the queue
      cannot grow indefinitely.
    */
    static constexpr size_t QUEUE_SIZE = 10;

    G1Command commandQueue[QUEUE_SIZE];

    // Index of the next command to execute
    size_t queueHead;

    // Index where the next new command will be stored
    size_t queueTail;

    // Number of commands currently stored
    size_t queueCount;


    // =====================================================
    // Internal parsing functions
    // =====================================================

    void parseGCode(char* command);

    bool parseG1(const char* command);

    bool parseNumber(
        const char*& cursor,
        float& result
    );


    // =====================================================
    // Queue functions
    // =====================================================

    /*
      Add a G1 command to the end of the queue.

      Returns false if the queue is full.
    */
    bool enqueueG1(
        float x,
        float y,
        float f
    );


    /*
      Remove the oldest G1 command from the queue.

      Returns false if the queue is empty.
    */
    bool dequeueG1(
        G1Command& command
    );
};

extern GCodeParser gcodeParser;

#endif