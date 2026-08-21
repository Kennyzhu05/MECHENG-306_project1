#include "Switch.h"
#include "FSM.h"
#include "Event.h"

// Limit Switch Pins
const int TOP_SWITCH_PIN    = 25;
const int BOTTOM_SWITCH_PIN = 24;
const int RIGHT_SWITCH_PIN  = 23;
const int LEFT_SWITCH_PIN   = 22;

// Debounce settings
const unsigned long DEBOUNCE_DELAY_MS = 20;  // adjust if switches are noisier/cleaner than this

// Debounced (confirmed) switch states - these are what the rest of the code reads
bool TOP_SWITCH_TOUCHED    = false;
bool BOTTOM_SWITCH_TOUCHED = false;
bool RIGHT_SWITCH_TOUCHED  = false;
bool LEFT_SWITCH_TOUCHED   = false;

// Internal debounce tracking - last raw reading and when it last changed
namespace {
    bool topRawLast    = false;
    bool bottomRawLast = false;
    bool rightRawLast  = false;
    bool leftRawLast   = false;

    unsigned long topLastChangeTime    = 0;
    unsigned long bottomLastChangeTime = 0;
    unsigned long rightLastChangeTime  = 0;
    unsigned long leftLastChangeTime   = 0;

    // Debounces a single switch. Call every loop with the current raw reading.
    // Updates confirmedState in place only once the raw reading has been
    // stable for DEBOUNCE_DELAY_MS.
    void debounceSwitch(bool rawReading, bool &rawLast, unsigned long &lastChangeTime,
                         bool &confirmedState)
    {
        if (rawReading != rawLast)
        {
            // Raw reading changed - restart the debounce timer
            lastChangeTime = millis();
            rawLast = rawReading;
        }

        if ((millis() - lastChangeTime) >= DEBOUNCE_DELAY_MS)
        {
            // Reading has been stable long enough - accept it
            confirmedState = rawReading;
        }
    }
}


// LIMIT SWITCH FUNCTIONS

// Configure all limit switch pins
void setupSwitches()
{
    pinMode(TOP_SWITCH_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_SWITCH_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LEFT_SWITCH_PIN, INPUT_PULLUP);

    // Seed raw/last state so the first updateLimitSwitches() call doesn't
    // see a spurious "change" against the default false initializers
    topRawLast    = (digitalRead(TOP_SWITCH_PIN) == HIGH);
    bottomRawLast = (digitalRead(BOTTOM_SWITCH_PIN) == HIGH);
    rightRawLast  = (digitalRead(RIGHT_SWITCH_PIN) == HIGH);
    leftRawLast   = (digitalRead(LEFT_SWITCH_PIN) == HIGH);

    TOP_SWITCH_TOUCHED    = topRawLast;
    BOTTOM_SWITCH_TOUCHED = bottomRawLast;
    RIGHT_SWITCH_TOUCHED  = rightRawLast;
    LEFT_SWITCH_TOUCHED   = leftRawLast;

    unsigned long now = millis();
    topLastChangeTime    = now;
    bottomLastChangeTime = now;
    rightLastChangeTime  = now;
    leftLastChangeTime   = now;
}


// Read every limit switch and update the stored (debounced) states.
// Must be called frequently (e.g. every loop iteration) for debouncing to work -
// it relies on repeated sampling over time, not a single blocking read.
void updateLimitSwitches()
{
    bool topRaw    = (digitalRead(TOP_SWITCH_PIN) == HIGH);
    bool bottomRaw = (digitalRead(BOTTOM_SWITCH_PIN) == HIGH);
    bool rightRaw  = (digitalRead(RIGHT_SWITCH_PIN) == HIGH);
    bool leftRaw   = (digitalRead(LEFT_SWITCH_PIN) == HIGH);

    debounceSwitch(topRaw,    topRawLast,    topLastChangeTime,    TOP_SWITCH_TOUCHED);
    debounceSwitch(bottomRaw, bottomRawLast, bottomLastChangeTime, BOTTOM_SWITCH_TOUCHED);
    debounceSwitch(rightRaw,  rightRawLast,  rightLastChangeTime,  RIGHT_SWITCH_TOUCHED);
    debounceSwitch(leftRaw,   leftRawLast,   leftLastChangeTime,   LEFT_SWITCH_TOUCHED);
}


// Individual switch status
bool topLimitReached()
{
    return TOP_SWITCH_TOUCHED;
}

bool bottomLimitReached()
{
    return BOTTOM_SWITCH_TOUCHED;
}

bool rightLimitReached()
{
    return RIGHT_SWITCH_TOUCHED;
}

bool leftLimitReached()
{
    return LEFT_SWITCH_TOUCHED;
}


// Movement safety
bool canMoveUp()
{
    return !TOP_SWITCH_TOUCHED;
}

bool canMoveDown()
{
    return !BOTTOM_SWITCH_TOUCHED;
}

bool canMoveRight()
{
    return !RIGHT_SWITCH_TOUCHED;
}

bool canMoveLeft()
{
    return !LEFT_SWITCH_TOUCHED;
}


// Returns true if any limit switch is pressed
bool anyLimitReached()
{
    return TOP_SWITCH_TOUCHED ||
           BOTTOM_SWITCH_TOUCHED ||
           RIGHT_SWITCH_TOUCHED ||
           LEFT_SWITCH_TOUCHED;
}

bool topRightReached()
{
    return TOP_SWITCH_TOUCHED || RIGHT_SWITCH_TOUCHED;
}

bool topLeftReached()
{
    return TOP_SWITCH_TOUCHED || LEFT_SWITCH_TOUCHED;
}

bool bottomRightReached()
{
    return BOTTOM_SWITCH_TOUCHED || RIGHT_SWITCH_TOUCHED;
}

bool bottomLeftReached()
{
    return BOTTOM_SWITCH_TOUCHED || LEFT_SWITCH_TOUCHED;
}

// Home position is bottom-left
bool homeReached()
{
    return LEFT_SWITCH_TOUCHED &&
           BOTTOM_SWITCH_TOUCHED;
}


// Print all switch states to the Serial Monitor
void printLimitSwitches()
{
    Serial.print("TOP: ");
    Serial.print(TOP_SWITCH_TOUCHED);

    Serial.print(" | BOTTOM: ");
    Serial.print(BOTTOM_SWITCH_TOUCHED);

    Serial.print(" | RIGHT: ");
    Serial.print(RIGHT_SWITCH_TOUCHED);

    Serial.print(" | LEFT: ");
    Serial.println(LEFT_SWITCH_TOUCHED);
}