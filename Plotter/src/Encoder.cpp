// #include <Arduino.h>
// #include "Encoder.h"

// // Encoder pins
// const int LEFT_ENCODER_A  = 17;
// const int LEFT_ENCODER_B  = 18;

// const int RIGHT_ENCODER_A = 19;
// const int RIGHT_ENCODER_B = 20;

// // Encoder counts
// long leftEncoderCount  = 0;
// long rightEncoderCount = 0;

// // Previous A-channel states
// int previousLeftA;
// int previousRightA;


// void setupEncoders()
// {
//     pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
//     pinMode(LEFT_ENCODER_B, INPUT_PULLUP);

//     pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
//     pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

//     previousLeftA = digitalRead(LEFT_ENCODER_A);
//     previousRightA = digitalRead(RIGHT_ENCODER_A);
// }


// // Read encoder states and update counts
// void updateEncoders()
// {
//     // LEFT ENCODER
//     int currentLeftA = digitalRead(LEFT_ENCODER_A);

//     if (currentLeftA != previousLeftA)
//     {
//         if (digitalRead(LEFT_ENCODER_B) != currentLeftA)
//         {
//             leftEncoderCount++;
//         }
//         else
//         {
//             leftEncoderCount--;
//         }

//         previousLeftA = currentLeftA;
//     }


//     // RIGHT ENCODER
//     int currentRightA = digitalRead(RIGHT_ENCODER_A);

//     if (currentRightA != previousRightA)
//     {
//         if (digitalRead(RIGHT_ENCODER_B) != currentRightA)
//         {
//             rightEncoderCount++;
//         }
//         else
//         {
//             rightEncoderCount--;
//         }

//         previousRightA = currentRightA;
//     }
// }


// void resetEncoders()
// {
//     leftEncoderCount = 0;
//     rightEncoderCount = 0;
// }


// long getLeftEncoderCount()
// {
//     return leftEncoderCount;
// }


// long getRightEncoderCount()
// {
//     return rightEncoderCount;
// }


// void printEncoderCounts()
// {
//     Serial.print("LEFT: ");
//     Serial.print(leftEncoderCount);

//     Serial.print(" | RIGHT: ");
//     Serial.println(rightEncoderCount);
// }