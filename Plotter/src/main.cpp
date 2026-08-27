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
#include "Helper.h"

FSM fsm;
GCodeParser gcodeParser(fsm);
String command;

static bool testStarted = false;   // whether the motion test has started

float distance_mm = 100.0f;   // distance to move in mm for the test

void setup()
{
  Serial.begin(115200);

  setupMotors();
  stopMotors();

  setupSwitches();
  setupEncoders();
  

  // Initialize
  resetEncoders();

  Serial.println("PROGRAM STARTED");
  fsm.begin();
  // Tell FSM that initialisation was successful
  fsm.processEvent(Event::INIT_SUCCESS);
  startHoming();
}

void loop()
{
  gcodeParser.updateSerialCommands();

  updateLimitSwitches();
  updateEncoders();
  updateMotion();

  fsm.update();

  gcodeParser.updateCommandQueue();
}