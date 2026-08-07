#ifndef MOTOR_H
#define MOTOR_H

void setupMotors();
void stopMotors();

// Direct signed-PWM control of each motor (positive = forward on that
// motor's own axis convention, negative = reverse). Magnitude 0-255.
void driveMotorA(int signedPwm);
void driveMotorB(int signedPwm);

// General 4 Motions
void moveTop(int pwm);
void moveBottom(int pwm);
void moveRight(int pwm);
void moveLeft(int pwm);

// Diagonal Motions
void moveBottomLeft(int pwm);
void moveTopLeft(int pwm);
void moveTopRight(int pwm);
void moveBottomRight(int pwm);

#endif