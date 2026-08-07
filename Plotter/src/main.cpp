#include <Arduino.h>
#include "Motor.h"
#include "Switch.h"
#include "RoundTesting.h"
#include "Encoder.h"

void setup()
{
  Serial.begin(9600);

  setupMotors();
  setupSwitches();
  setupEncoders();

  // Initialize
  resetEncoders();
  stopMotors();

  Serial.println("PROGRAM STARTED");
}

void loop()
{

}