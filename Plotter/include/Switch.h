#pragma once

#include <Arduino.h>
#include "FSM.h"

// Setup
void setupSwitches();

// Update switch states
void updateLimitSwitches();

// Individual switch status
bool topLimitReached();
bool bottomLimitReached();
bool rightLimitReached();
bool leftLimitReached();

// Movement safety
bool canMoveUp();
bool canMoveDown();
bool canMoveRight();
bool canMoveLeft();

// General status
bool anyLimitReached();
bool topRightReached();
bool homeReached();

// Debugging
void printLimitSwitches();