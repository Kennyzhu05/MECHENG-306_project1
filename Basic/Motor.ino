// Left Motor (M1)
const int M_1 = 4;
const int M_1_PWM = 5;

// Right Motor (M2)
const int M_2 = 7;
const int M_2_PWM = 6;

void setupMotors() {
  pinMode (M_1, OUTPUT);
  pinMode(M_1_PWM, OUTPUT);

  pinMode(M_2, OUTPUT);
  pinMode(M_2_PWM, OUTPUT);
}

void stopMotors() {
  analogWrite(M_1_PWM, 0);
  analogWrite(M_2_PWM, 0);
}

// General 4 Motions
void moveTop(int pwm) {
  // M1 (+) M2 (-)
  digitalWrite(M_1, HIGH);
  digitalWrite(M_2, LOW);

  analogWrite(M_1_PWM, pwm);
  analogWrite(M_2_PWM, pwm);
}

void moveBottom(int pwm) {
  // M1 (-) M2 (+)
  digitalWrite(M_1, LOW);
  digitalWrite(M_2, HIGH);

  analogWrite(M_1_PWM, pwm);
  analogWrite(M_2_PWM, pwm);
}

void moveRight(int pwm) {
  // M1 (+) M2 (+)
  digitalWrite(M_1, HIGH);
  digitalWrite(M_2, HIGH);

  analogWrite(M_1_PWM, pwm);
  analogWrite(M_2_PWM, pwm);
}

void moveLeft(int pwm) {
  // M1 (-) M2 (-)
  digitalWrite(M_1, LOW);
  digitalWrite(M_2, LOW);

  analogWrite(M_1_PWM, pwm);
  analogWrite(M_2_PWM, pwm);
}

// Diagonal Motions
void moveBottomLeft(int pwm) {
  // Stop M2
  analogWrite(M_2_PWM, 0);
  // M1 (-)
  digitalWrite(M_1, LOW);
  analogWrite(M_1_PWM, pwm);
}

void moveTopLeft(int pwm) {
  // Stop M1
  analogWrite(M_1_PWM, 0);
  // M2 (-)
  digitalWrite(M_2, LOW);
  analogWrite(M_2_PWM, pwm);
}

void moveTopRight(int pwm) {
  // Stop M2
  analogWrite(M_2_PWM, 0);
  // M1 (+)
  digitalWrite(M_1, HIGH);
  analogWrite(M_1_PWM, pwm);
}

void moveBottomRight(int pwm) {
  // Stop M1
  analogWrite(M_1_PWM, 0);
  // M2 (+)
  digitalWrite(M_2, HIGH);
  analogWrite(M_2_PWM, pwm);
}
