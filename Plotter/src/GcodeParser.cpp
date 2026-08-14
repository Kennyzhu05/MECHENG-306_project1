#include "GCodeParser.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>


// =====================================================
// Constructor
// =====================================================

GCodeParser::GCodeParser(FSM& fsmReference)
    : fsm(fsmReference),
      commandIndex(0),
      targetX(0.0f),
      targetY(0.0f),
      lastFeedRate(100.0f),
      targetFeedRate(100.0f),
      queueHead(0),
      queueTail(0),
      queueCount(0)
{
    commandBuffer[0] = '\0';
}


// =====================================================
// Read serial commands
// =====================================================

void GCodeParser::updateSerialCommands()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();

        // End of one command
        if (receivedChar == '\n' || receivedChar == '\r')
        {
            if (commandIndex > 0)
            {
                // Terminate C-string
                commandBuffer[commandIndex] = '\0';

                // Parse complete command
                parseGCode(commandBuffer);

                // Reset buffer
                commandIndex = 0;
                commandBuffer[0] = '\0';
            }
        }
        else
        {
            if (commandIndex < COMMAND_BUFFER_SIZE - 1)
            {
                commandBuffer[commandIndex] = receivedChar;
                commandIndex++;
            }
            else
            {
                Serial.println("ERROR: command too long");

                // Reset current command
                commandIndex = 0;
                commandBuffer[0] = '\0';
            }
        }
    }
}


// =====================================================
// Parse complete G-code command
// =====================================================

void GCodeParser::parseGCode(char* command)
{
    // Convert lowercase letters to uppercase
    for (size_t i = 0; command[i] != '\0'; i++)
    {
        command[i] =
            static_cast<char>(toupper(command[i]));
    }

    // -----------------------------
    // RESET
    // -----------------------------

    if (strcmp(command, "RESET") == 0)
    {
        fsm.processEvent(Event::RESET);
        return;
    }

    // -----------------------------
    // G28
    // -----------------------------

    if (strcmp(command, "G28") == 0)
    {
      Serial.println("G28 accepted: Homing command received");
        fsm.processEvent(Event::G28_RECEIVED);
        return;
    }

    // -----------------------------
    // G1 / G01
    // -----------------------------

    if (parseG1(command))
{
    // If machine is IDLE, execute G1 immediately
    if (fsm.isIdle())
    {
        Serial.println("G1 accepted: executing now");

        fsm.processEvent(Event::G1_RECEIVED);
    }

    // If machine is busy, store command in queue
    else if (fsm.isBusy())
    {
        if (enqueueG1(
                targetX,
                targetY,
                targetFeedRate))
        {
            Serial.println("G1 queued");
        }
        else
        {
            Serial.println("ERROR: G1 queue full");
        }
    }

    // If machine is in FAULT, reject command
    else if (fsm.isFault())
    {
        Serial.println("ERROR: G1 rejected - system in FAULT");
    }

    return;
}

    Serial.println("ERROR: invalid G-code");
}


// =====================================================
// Parse G1 command
// =====================================================

bool GCodeParser::parseG1(const char* command)
{
    const char* cursor = command;

    // Ignore leading spaces
    while (isspace(*cursor))
    {
        cursor++;
    }

    // Must begin with G
    if (*cursor != 'G')
    {
        return false;
    }

    cursor++;

    char* endPointer;

    /*
      Read number after G.

      Examples:

      G1
      G01

      Both give gNumber = 1.
    */

    long gNumber = strtol(
        cursor,
        &endPointer,
        10
    );

    // No number after G, or command is not G1
    if (endPointer == cursor || gNumber != 1)
    {
        return false;
    }

    cursor = endPointer;

    // -----------------------------
    // Default values
    // -----------------------------

    /*
      Missing X:
      X offset = 0

      Missing Y:
      Y offset = 0

      Missing F:
      inherit previous feed rate
    */

    float parsedX = 0.0f;
    float parsedY = 0.0f;
    float parsedF = lastFeedRate;

    bool hasX = false;
    bool hasY = false;
    bool hasF = false;

    // -----------------------------
    // Read parameters
    // -----------------------------

    while (*cursor != '\0')
    {
        // Ignore spaces
        while (isspace(*cursor))
        {
            cursor++;
        }

        if (*cursor == '\0')
        {
            break;
        }

        char parameter = *cursor;

        cursor++;

        float value;

        if (!parseNumber(cursor, value))
        {
            return false;
        }

        switch (parameter)
        {
            case 'X':

                // Reject duplicate X
                if (hasX)
                {
                    return false;
                }

                parsedX = value;
                hasX = true;

                break;


            case 'Y':

                // Reject duplicate Y
                if (hasY)
                {
                    return false;
                }

                parsedY = value;
                hasY = true;

                break;


            case 'F':

                // Reject duplicate F
                // Feed rate must also be positive
                if (hasF || value <= 0.0f)
                {
                    return false;
                }

                parsedF = value;
                hasF = true;

                break;


            default:

                // Unknown parameter
                return false;
        }
    }

    // Reject G1 with no parameters
    if (!hasX && !hasY && !hasF)
    {
        return false;
    }

    // -----------------------------
    // Save parsed results
    // -----------------------------

    targetX = parsedX;
    targetY = parsedY;
    targetFeedRate = parsedF;

    // Feed rate inheritance
    if (hasF)
    {
        lastFeedRate = parsedF;
    }

    // -----------------------------
    // Debug output
    // -----------------------------

    Serial.print("X = ");
    Serial.println(targetX);

    Serial.print("Y = ");
    Serial.println(targetY);

    Serial.print("F = ");
    Serial.println(targetFeedRate);

    return true;
}


// =====================================================
// Parse signed floating-point number
// =====================================================

bool GCodeParser::parseNumber(
    const char*& cursor,
    float& result
)
{
    char* endPointer;

    /*
      strtof supports:

      10
      -10
      +10
      10.5
      -10.5
    */

    result = strtod(
        cursor,
        &endPointer
    );

    // No valid number found
    if (endPointer == cursor)
    {
        return false;
    }

    // Move cursor to first character after number
    cursor = endPointer;

    return true;
}


// =====================================================
// Getter functions
// =====================================================

float GCodeParser::getTargetX() const
{
    return targetX;
}


float GCodeParser::getTargetY() const
{
    return targetY;
}


float GCodeParser::getTargetFeedRate() const
{
    return targetFeedRate;
}

// =====================================================
// Add G1 command to queue
// =====================================================

bool GCodeParser::enqueueG1(
    float x,
    float y,
    float f
)
{
    // Queue is full
    if (queueCount >= QUEUE_SIZE)
    {
        return false;
    }

    // Store command at tail
    commandQueue[queueTail].x = x;
    commandQueue[queueTail].y = y;
    commandQueue[queueTail].f = f;

    // Move tail forward
    queueTail++;

    // Circular wrap-around
    if (queueTail >= QUEUE_SIZE)
    {
        queueTail = 0;
    }

    queueCount++;

    return true;
}


// =====================================================
// Remove oldest G1 command from queue
// =====================================================

bool GCodeParser::dequeueG1(
    G1Command& command
)
{
    // Queue is empty
    if (queueCount == 0)
    {
        return false;
    }

    // Copy oldest command
    command = commandQueue[queueHead];

    // Move head forward
    queueHead++;

    // Circular wrap-around
    if (queueHead >= QUEUE_SIZE)
    {
        queueHead = 0;
    }

    queueCount--;

    return true;
}

// =====================================================
// Execute queued G1 commands
// =====================================================

void GCodeParser::updateCommandQueue()
{
    /*
      A queued G1 command can only start when
      the FSM returns to IDLE.
    */

    if (!fsm.isIdle())
    {
        return;
    }

    // Nothing waiting
    if (queueCount == 0)
    {
        return;
    }

    G1Command nextCommand;

    if (!dequeueG1(nextCommand))
    {
        return;
    }

    // Load queued command back into active target variables
    targetX = nextCommand.x;
    targetY = nextCommand.y;
    targetFeedRate = nextCommand.f;

    Serial.println("Executing queued G1");

    Serial.print("X = ");
    Serial.println(targetX);

    Serial.print("Y = ");
    Serial.println(targetY);

    Serial.print("F = ");
    Serial.println(targetFeedRate);

    // IDLE -> MOVING
    fsm.processEvent(Event::G1_RECEIVED);
}