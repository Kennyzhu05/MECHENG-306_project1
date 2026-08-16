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

static int step = 0;   // which step of the motion sequence we're on
static bool testStarted = false;   // whether the motion test has started
static unsigned long lastPrintTime = 0;
static const unsigned long PRINT_INTERVAL_MS = 500;   // print encoder counts every 500 ms

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
  // startHoming();

  resetBottomLeft();

  // Start First Motion
  startMotion(mmToXCounts(distance_mm), 0);
  testStarted = true;
  // step = 1;
}

void loop()
{
  updateMotion();
  // ===================================
  // Sequeunce of motions for testing
  // ===================================

  // if (!isMotionDone())
  // {
  //     return;   // still moving, nothing else to do this tick
  // }

  // if (step == 1)
  // {
  //     Serial.println(F("Step 1 done. Starting step 2."));
  //     startMotion(0, 7000);   // up 7000
  //     step = 2;
  // }
  // else if (step == 2)
  // {
  //     Serial.println(F("Step 2 done. Starting step 3."));
  //     startMotion(-7000, 0);  // left 7000
  //     step = 3;
  // }
  // else if (step == 3)
  // {
  //     Serial.println(F("Step 3 done. Starting step 4."));
  //     startMotion(0, -7000);  // down 7000
  //     step = 4;
  // }
  // else if (step == 4)
  // {
  //     Serial.println(F("Step 4 done. Starting step 5."));
  //     startMotion(7000, 7000);  // top-right 7000, 7000
  //     step = 5;
  // }
  // else if (step == 5)
  // {
  //     Serial.println(F("Step 5 done. Starting step 6."));
  //     startMotion(-7000, -7000);  // bottom-left 7000, 7000
  //     step = 6;
  // }
  // else if (step == 6)
  // {
  //     Serial.println(F("Step 6 done. All motions complete."));
  //     step = 7;   // stop here
  // }

  // ===================================
  // Sequence of motion end
  // ===================================


  // ===================================
  // Motion test with encoder printing
  // ===================================

  updateMotion();   // call this every loop() to update the motion controller

  if (!testStarted)
    {
        return;
    }
 
    updateMotion();
    long now = millis();

 
    if (now - lastPrintTime >= PRINT_INTERVAL_MS)
    {
        lastPrintTime = now;
        // Print encoder counts and motion active status
        Serial.println(F("Encoder counts: "));
        Serial.print(F("A: "));
        Serial.print(getLeftEncoderCount());
        Serial.print(F("  B: "));
        Serial.print(getRightEncoderCount());
        Serial.print(F("  active: "));
        Serial.println(isMotionActive());

        // Print distance to target
        Serial.println(F("Target distance: "));
        Serial.print(F("X: "));
        Serial.print(mmToXCounts((getLeftEncoderCount() + getRightEncoderCount()) / 2.0));
        Serial.print(F("  Y: "));
        Serial.print(mmToYCounts((getLeftEncoderCount() - getRightEncoderCount()) / 2.0));
        Serial.print(F("  active: "));
        Serial.println(isMotionActive());
        Serial.println();
        Serial.println();
    }
 
    if (isMotionDone())
    {
        MotionResult result = getLastMotionResult();
        Serial.print(F("Done. Result: "));
        Serial.println(result == MotionResult::SETTLED ? F("SETTLED") : F("TIMED_OUT"));
        Serial.print(F("  Distance: "));
        Serial.println(distance_mm);
        Serial.print(F("  Expected A: "));
        Serial.print(mmToXCounts(distance_mm));
        Serial.print(F("  Expected B: "));
        Serial.println(mmToXCounts(distance_mm));

        Serial.print(F("  Final A: "));
        Serial.print(getLeftEncoderCount());
        Serial.print(F("  Final B: "));
        Serial.println(getRightEncoderCount());
        
        Serial.print(F("  Final X: "));
        Serial.print(xCountsToMm((getLeftEncoderCount() + getRightEncoderCount()) / 2.0));
        Serial.print(F("  Final Y: "));
        Serial.println(yCountsToMm((getLeftEncoderCount() - getRightEncoderCount()) / 2.0));

        testStarted = false;   // stop printing; reset the board to run again
    }

    // ===================================
    // Motion test with encoder printing end
    // ===================================

    // resetEncoders();

    // driveMotorA(255);
    // driveMotorB(255);

    // unsigned long startTime = millis();

    // while (millis() - startTime < 2000)
    // {
    //     updateEncoders();
    // }

    // stopMotors();

    // Serial.print("A = ");
    // Serial.println(getLeftEncoderCount());

    // Serial.print("B = ");
    // Serial.println(getRightEncoderCount());

    // moveDone = true;
  // }


  // if (isBoundaryTestComplete() || hasBoundaryTestFault())
  // {
  //   while (true)
  //   {
  //     // Stop the program here if the boundary test is complete or has a fault
  //     delay(1000);
  //   }
  // }

  // if (Serial.available())
  // {
  //   command = Serial.readStringUntil('\n');
  //   command.trim();

  //   Serial.print("Received: ");
  //   Serial.println(command);

    // Later:
    // processGCode(command);
  // }
  // Other system tasks can go here

  // Update the G-code queue after fsm.update();
  // gcodeParser.updateCommandQueue();
}