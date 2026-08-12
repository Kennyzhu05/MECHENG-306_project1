#ifndef HOMING_H
#define HOMING_H

// Start the homing sequence.
// This should be called when the FSM enters HOMING.
void startHoming();

// Run the homing sequence.
// Call this repeatedly from loop() while the FSM is in HOMING.
void updateHoming();

// Returns true when the platform has reached the bottom-left home position.
bool isHomingComplete();

// Reset the homing controller.
void resetHoming();

#endif