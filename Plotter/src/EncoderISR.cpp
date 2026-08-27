#include <Arduino.h>
#include "Encoder.h"

// =====================================================
// Encoder pins
// =====================================================

const int LEFT_ENCODER_A  = 18;
const int LEFT_ENCODER_B  = 19;

const int RIGHT_ENCODER_A = 20;
const int RIGHT_ENCODER_B = 21;


// =====================================================
// Encoder counts
// =====================================================

// volatile because these variables are modified
// inside interrupt service routines (ISRs)
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;


// =====================================================
// LEFT encoder interrupt
// =====================================================

void leftEncoderISR()
{
    bool A = digitalRead(LEFT_ENCODER_A);
    bool B = digitalRead(LEFT_ENCODER_B);

    // Same direction logic as your original Encoder.cpp
    if (B != A)
    {
        leftEncoderCount++;
    }
    else
    {
        leftEncoderCount--;
    }
}


// =====================================================
// RIGHT encoder interrupt
// =====================================================

void rightEncoderISR()
{
    bool A = digitalRead(RIGHT_ENCODER_A);
    bool B = digitalRead(RIGHT_ENCODER_B);

    // Same direction logic as your original Encoder.cpp
    if (B != A)
    {
        rightEncoderCount++;
    }
    else
    {
        rightEncoderCount--;
    }
}


// =====================================================
// Setup encoders
// =====================================================

void setupEncoders()
{
    pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
    pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

    pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);


    // Interrupt whenever LEFT A changes HIGH <-> LOW
    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENCODER_A),
        leftEncoderISR,
        CHANGE
    );


    // Interrupt whenever RIGHT A changes HIGH <-> LOW
    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENCODER_A),
        rightEncoderISR,
        CHANGE
    );
}


// =====================================================
// updateEncoders()
// =====================================================

// Kept so your existing Motion.cpp does not need changing.
//
// Previously this function polled the encoders.
// Now the interrupt routines count automatically.
void updateEncoders()
{
    // Nothing required here.
}


// =====================================================
// Reset encoders
// =====================================================

void resetEncoders()
{
    noInterrupts();

    leftEncoderCount  = 0;
    rightEncoderCount = 0;

    interrupts();
}


// =====================================================
// Get LEFT encoder count
// =====================================================

long getLeftEncoderCount()
{
    noInterrupts();

    long count = leftEncoderCount;

    interrupts();

    return count;
}


// =====================================================
// Get RIGHT encoder count
// =====================================================

long getRightEncoderCount()
{
    noInterrupts();

    long count = rightEncoderCount;

    interrupts();

    return count;
}


// =====================================================
// Print encoder counts
// =====================================================

void printEncoderCounts()
{
    // Get safe copies first
    long left  = getLeftEncoderCount();
    long right = getRightEncoderCount();

    Serial.print("LEFT: ");
    Serial.print(left);

    Serial.print(" | RIGHT: ");
    Serial.println(right);
}