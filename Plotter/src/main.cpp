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
}

bool moveDone = false;

void loop()
{
  // gcodeParser.updateSerialCommands();
  if (!moveDone) {
    moveXY(100, 100);
    Serial.print("Final X: ");
    Serial.println(getLeftEncoderCount());
    Serial.print("Final Y: ");
    Serial.println(getRightEncoderCount());
    moveDone = true;
  }
  // fsm.update();
}