#include <Servo.h>

// define the pins for the motor driver
#define M1DIR 2 // FRONT LEFT DIR
#define M1PWM 3 // FRONT LEFT PWM
#define M2DIR 4 // FRONT RIGHT DIR
#define M2PWM 5  // FRONT RIGHT PWM
#define M3DIR 6 // BACK LEFT DIR
#define M3PWM 7 // BACK LEFT PWM
#define M4DIR 8 // BACK RIGHT DIR
#define M4PWM 9 // BACK RIGHT PWM

// define the pin for the receiver
#define CH1 10
#define CH2 11
#define CH3 12
#define CH4 13
#define CH5A 53
// #define CH5B 52

// define the pins for the encoders
#define ENC1A A0
#define ENC1B A1
#define ENC2A A2
#define ENC2B A3
#define ENC3A A4
#define ENC3B A5
#define ENC4A A6
#define ENC4B A7

// Create variables to store the receiver channel values
int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;
int ch5AValue = 0;
// int ch5BValue = 0;

// Create variables to store the encoder counts and previous counts for each motor
long enc1Count = 0;
long enc1PrevCount = 0;
long enc2Count = 0;
long enc2PrevCount = 0;
long enc3Count = 0;
long enc3PrevCount = 0;
long enc4Count = 0;
long enc4PrevCount = 0;

// Create variables to store the PID values for all motors
const double kp = 1.5;
const double ki = 0.0;
const double kd = 0.01;

// Create variables to store the error, integral, and derivative values for each motor's PID control loop
double error1 = 0;
double integral1 = 0;
double derivative1 = 0;
double error2 = 0;
double integral2 = 0;
double derivative2 = 0;
double error3 = 0;
double integral3 = 0;
double derivative3 = 0;
double error4 = 0;
double integral4 = 0;
double derivative4 = 0;

// Motor Offset
double M1offset = 16.8;
double M2offset = 16.9;
double M3offset = -1;
double M4offset = 3.01;

//Serial Plotter
// Create variables to store the sum of the error values for each motor
double errorSum1 = 0;
double errorSum2 = 0;
double errorSum3 = 0;
double errorSum4 = 0;

// Create a variable to count the number of loop iterations
int loopCount = 0;

// Deadband size
double deadBand = 60;  // (1 controller tick = 100, default 20-80)

// Slow mode
double slowMo;

// Logging
int slowlog = 0;
int slowlog2 = 0;

void setup() {
  // initialize the motor driver pins as outputs
  pinMode(M1DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(M3DIR, OUTPUT);
  pinMode(M3PWM, OUTPUT);
  pinMode(M4DIR, OUTPUT);
  pinMode(M4PWM, OUTPUT);

  // initialize the receiver pins as inputs
  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);
  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);
  pinMode(CH5A, INPUT);
  // pinMode(CH5B, INPUT);

  // initialize the encoder pins as inputs and attach interrupts to them to count pulses from the encoders
  pinMode(ENC1A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1A), countEnc1A, CHANGE);
  pinMode(ENC1B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1B), countEnc1B, CHANGE);
  pinMode(ENC2A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2A), countEnc2A, CHANGE);
  pinMode(ENC2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2B), countEnc2B, CHANGE);
  pinMode(ENC3A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC3A), countEnc3A, CHANGE);
  pinMode(ENC3B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC3B), countEnc3B, CHANGE);
  pinMode(ENC4A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC4A), countEnc4A, CHANGE);
  pinMode(ENC4B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC4B), countEnc4B, CHANGE);

  //Serial Plotter
  Serial.begin(9600);
}

void loop() {
  // Read the values from the receiver channels using pulseIn()
  ch1Value = pulseIn(CH1, HIGH);
  ch2Value = pulseIn(CH2, HIGH);
  ch3Value = pulseIn(CH3, HIGH);
  ch4Value = pulseIn(CH4, HIGH);
  ch5AValue = pulseIn(CH5A, HIGH);
  // ch5BValue = pulseIn(CH5B, HIGH);

  if (slowlog >= 100) {
    // Print the receiver channel values for debugging
    Serial.print("CH1: ");
    Serial.print(ch1Value);
    Serial.print(" CH2: ");
    Serial.print(ch2Value);
    Serial.print(" CH3: ");
    Serial.print(ch3Value);
    Serial.print(" CH4: ");
    Serial.println(ch4Value);
    Serial.print(" CH5A: ");
    Serial.println(ch5AValue);
    Serial.println(slowMo);
    slowlog = 0;
  } else {
    slowlog += 10;
  }

  // Map the receiver channel values to a range of -500 to +500
  int x = map(ch1Value, 1000, 2000, -253, +253);
  int y = map(ch3Value, 1000, 2000, -253, +253);
  int r = map(ch4Value, 1000, 2000, -253, +253);

  // Calculate the desired motor speeds for a mecanum drive (X is forward, Y is right, R is clockwise)
  int m1Speed = x + y + r + M1offset; // M1 is front left
  int m2Speed = x - y + r + M2offset; // M2 is front right
  int m3Speed = x - y - r + M3offset; // M3 is back left
  int m4Speed = x + y - r + M4offset; // M4 is back right

  // Calculate the actual motor speeds based on the encoder counts
  long enc1Delta = enc1Count - enc1PrevCount;
  long enc2Delta = enc2Count - enc2PrevCount;
  long enc3Delta = enc3Count - enc3PrevCount;
  long enc4Delta = enc4Count - enc4PrevCount;
  int m1ActualSpeed = map(enc1Delta, -1000, 1000, -253, +253);
  int m2ActualSpeed = map(enc2Delta, -1000, 1000, -253, +253);
  int m3ActualSpeed = map(enc3Delta, -1000, 1000, -253, +253);
  int m4ActualSpeed = map(enc4Delta, -1000, 1000, -253, +253);

  // Calculate the error between the desired and actual motor speeds
  error1 = m1Speed - m1ActualSpeed;
  error2 = m2Speed - m2ActualSpeed;
  error3 = m3Speed - m3ActualSpeed;
  error4 = m4Speed - m4ActualSpeed;

  // Update the integral and derivative values for each motor's PID control loop
  integral1 += error1;
  derivative1 = error1 - (m1Speed - map(enc1PrevCount - enc1Count, -1000, 1000, -253, +253));
  integral2 += error2;
  derivative2 = error2 - (m2Speed - map(enc2PrevCount - enc2Count, -1000, 1000, -253, +253));
  integral3 += error3;
  derivative3 = error3 - (m3Speed - map(enc3PrevCount - enc3Count, -1000, 1000, -253, +253));
  integral4 += error4;
  derivative4 = error4 - (m4Speed - map(enc4PrevCount - enc4Count, -1000, 1000, -253, +253));

  // Calculate the PID output for each motor
  int pidOutput1 = kp * error1 + ki * integral1 + kd * derivative1;
  int pidOutput2 = kp * error2 + ki * integral2 + kd * derivative2;
  int pidOutput3 = kp * error3 + ki * integral3 + kd * derivative3;
  int pidOutput4 = kp * error4 + ki * integral4 + kd * derivative4;

  // Set the direction and speed of each motor based on the PID output
  setMotor(M1DIR, M1PWM, pidOutput1); // M1 is front left, DIR Pin = 2, PWM Pin = 3
  setMotor(M2DIR, M2PWM, pidOutput2); // M2 is front right, DIR Pin = 4, PWM Pin = 5
  setMotor(M3DIR, M3PWM, pidOutput3); // M3 is back left, DIR Pin = 6, PWM Pin = 7
  setMotor(M4DIR, M4PWM, pidOutput4); // M4 is back right, DIR Pin = 8, PWM Pin = 9

  // Serial Plotter
  // Calculate the average error values for each motor
  double avgError1 = errorSum1 / loopCount;
  double avgError2 = errorSum2 / loopCount;
  double avgError3 = errorSum3 / loopCount;
  double avgError4 = errorSum4 / loopCount;

  // Plot the expected and actual values for each motor using the Serial Plotter
  if (slowlog2 >= 100) {
    Serial.print(m1Speed);
    Serial.print(",");
    Serial.print(m1ActualSpeed);
    Serial.print(",");
    Serial.print(m2Speed);
    Serial.print(",");
    Serial.print(m2ActualSpeed);
    Serial.print(",");
    Serial.print(m3Speed);
    Serial.print(",");
    Serial.print(m3ActualSpeed);
    Serial.print(",");
    Serial.print(m4Speed);
    Serial.print(",");
    Serial.println(m4ActualSpeed);
    slowlog2 = 0;
  } else {
    slowlog2 += 5;
  }

  // Update the sum of the error values and the loop count
  errorSum1 += error1;
  errorSum2 += error2;
  errorSum3 += error3;
  errorSum4 += error4;
  loopCount++;
}

void setMotor(int dirPin, int pwmPin, int speed) {
  // Constrain the speed to the range [-255, +255]
  speed = constrain(speed, -255, +255);

  // Slow mode variable
  slowMo = map(ch5AValue, 1000, 2000, 0, 100); // In percentage

  // Determine the direction and PWM value based on the speed
  int dir = (speed > 0) ? HIGH : LOW;
  int pwm = abs(speed);

  if (speed > deadBand || speed < -deadBand) {
    // Set the direction and PWM value for the specified motor
    digitalWrite(dirPin, dir);
    analogWrite(pwmPin, pwm * (slowMo / 100));
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, 0);
  }
}

void countEnc1A() {
  if (digitalRead(ENC1B) == HIGH) {
    enc1Count++;
  } else {
    enc1Count--;
  }
}

void countEnc1B() {
  if (digitalRead(ENC1A) == LOW) {
    enc1Count++;
  } else {
    enc1Count--;
  }
}

void countEnc2A() {
  if (digitalRead(ENC2B) == HIGH) {
    enc2Count++;
  } else {
    enc2Count--;
  }
}

void countEnc2B() {
  if (digitalRead(ENC2A) == LOW) {
    enc2Count++;
  } else {
    enc2Count--;
  }
}

void countEnc3A() {
  if (digitalRead(ENC3B) == HIGH) {
    enc3Count++;
  } else {
    enc3Count--;
  }
}

void countEnc3B() {
  if (digitalRead(ENC3A) == LOW) {
    enc3Count++;
  } else {
    enc3Count--;
  }
}

void countEnc4A() {
  if (digitalRead(ENC4B) == HIGH) {
    enc4Count++;
  } else {
    enc4Count--;
  }
}

void countEnc4B() {
  if (digitalRead(ENC4A) == LOW) {
    enc4Count++;
  } else {
    enc4Count--;
  }
}