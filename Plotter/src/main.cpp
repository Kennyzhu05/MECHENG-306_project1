#include <Arduino.h>
#include "Motor.h"
#include "Switch.h"
#include "RoundTesting.h"
#include "Encoder.h"
#include "Motion.h"
#include "GcodeParser.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

FSM fsm;
GCodeParser gcodeParser(fsm);

void setup()
{
  Serial.begin(115200);

  setupMotors();
  setupSwitches();
  setupEncoders();

  // Initialize
  resetEncoders();
  stopMotors();
  
  Serial.println("PROGRAM STARTED");
  fsm.begin();
  // Tell FSM that initialisation was successful
  fsm.processEvent(Event::INIT_SUCCESS);
  startHoming();
}

bool moveDone = false;

void loop()
{
    // Keep the physical limit-switch states updated
    updateLimitSwitches();

    // Update the FSM
    fsm.update();
if (Serial.available())
    {
        command = Serial.readStringUntil('\n');
        command.trim();

        Serial.print("Received: ");
        Serial.println(command);

        // Later:
        // processGCode(command);
    }
    // Other system tasks can go here
}