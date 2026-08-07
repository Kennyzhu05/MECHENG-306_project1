#include "Switch.h"

// Limit Switch Pins
const int TOP_SWITCH_PIN    = 25;
const int BOTTOM_SWITCH_PIN = 24;
const int RIGHT_SWITCH_PIN  = 23;
const int LEFT_SWITCH_PIN   = 22;

// Switch States
bool TOP_SWITCH_TOUCHED    = false;
bool BOTTOM_SWITCH_TOUCHED = false;
bool RIGHT_SWITCH_TOUCHED  = false;
bool LEFT_SWITCH_TOUCHED   = false;


// LIMIT SWITCH FUNCTIONS

// Configure all limit switch pins
void setupSwitches()
{
    pinMode(TOP_SWITCH_PIN, INPUT_PULLUP);
    pinMode(BOTTOM_SWITCH_PIN, INPUT_PULLUP);
    pinMode(RIGHT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LEFT_SWITCH_PIN, INPUT_PULLUP);
}


// Read every limit switch and update the stored states
void updateLimitSwitches()
{
    TOP_SWITCH_TOUCHED    = (digitalRead(TOP_SWITCH_PIN) == HIGH);
    BOTTOM_SWITCH_TOUCHED = (digitalRead(BOTTOM_SWITCH_PIN) == HIGH);
    RIGHT_SWITCH_TOUCHED  = (digitalRead(RIGHT_SWITCH_PIN) == HIGH);
    LEFT_SWITCH_TOUCHED   = (digitalRead(LEFT_SWITCH_PIN) == HIGH);
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