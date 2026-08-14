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
#include "FSM.h"
#include "Homing.h"
#include "BoundaryTesting.h"

FSM fsm;
GCodeParser gcodeParser(fsm);
String command;

void setup()
{
  Serial.begin(115200);

  setupMotors();
  stopMotors();

  setupSwitches();
  setupEncoders();

  // Initialize
  resetEncoders();
  
  beginBoundaryTest();

  Serial.println("PROGRAM STARTED");
  fsm.begin();
  // Tell FSM that initialisation was successful
  fsm.processEvent(Event::INIT_SUCCESS);
  startHoming();
}

bool moveDone = false;

void loop()
{
  // if (!moveDone) {
  //   moveXY(1000, 1000);
  //   long motorA = getLeftEncoderCount();
  //   long motorB = getRightEncoderCount();

  //   long finalX = (motorA + motorB) / 2;
  //   long finalY = (motorA - motorB) / 2;

  //   Serial.print("Final X: ");
  //   Serial.println(finalX);
  //   Serial.print("Final Y: ");
  //   Serial.println(finalY);
  //   moveDone = true;
  // }

  updateBoundaryTest();

  if (isBoundaryTestComplete() || hasBoundaryTestFault())
  {
    while (true)
    {
      // Stop the program here if the boundary test is complete or has a fault
      delay(1000);
    }
  }

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

  // Update the G-code queue after fsm.update();
  // gcodeParser.updateCommandQueue();
}