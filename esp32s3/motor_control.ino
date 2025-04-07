#include <Arduino.h>

// Hardware Serial for debug output
#define DEBUG_SERIAL Serial
// Hardware Serial for communication with Raspberry Pi
HardwareSerial RpiSerial(1);

// Pin Definitions for ESP32-S3 Feather with L298N
// Left Side Motors
#define FL_IN1 42    // Front Left Motor IN1
#define FL_IN2 2   // Front Left Motor IN2
#define FL_ENA 1    // Front Left Motor PWM

#define RL_IN1 7    // Rear Left Motor IN1
#define RL_IN2 6    // Rear Left Motor IN2
#define RL_ENA 5    // Rear Left Motor PWM

// Right Side Motors
#define FR_IN1 39   // Front Right Motor IN1
#define FR_IN2 40   // Front Right Motor IN2
#define FR_ENA 38   // Front Right Motor PWM

#define RR_IN1 16   // Rear Right Motor IN1
#define RR_IN2 17   // Rear Right Motor IN2
#define RR_ENA 18   // Rear Right Motor PWM

// Define PWM Parameters
const int pwmFreq = 1000;         // PWM frequency
const int pwmResolution = 8;      // PWM resolution (8 bits = 0 to 255)

// Define PWM channels for motors
const int pwmFL = 0;  // Front Left
const int pwmRL = 2;  // Rear Left
const int pwmFR = 1;  // Front Right
const int pwmRR = 3;  // Rear Right

// Timing variable for checking commands
unsigned long lastCommandCheckTime = 0;
const unsigned long COMMAND_CHECK_INTERVAL = 10;  // Check for commands every 10ms (100Hz)

void setup() {
  DEBUG_SERIAL.begin(115200);  // Start Serial for debug logging
  // Start Serial1 for communication with RPi (RX on pin 44, TX on pin 43)
  RpiSerial.begin(115200, SERIAL_8N1, 44, 43);
  
  DEBUG_SERIAL.println("Initializing Motor Pins...");
  // Initialize left side pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_ENA, OUTPUT);
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_ENA, OUTPUT);
  
  // Initialize right side pins
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_ENA, OUTPUT);
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_ENA, OUTPUT);

  DEBUG_SERIAL.println("Setting up PWM channels...");
  ledcSetup(pwmFL, pwmFreq, pwmResolution);
  ledcSetup(pwmFR, pwmFreq, pwmResolution);
  ledcSetup(pwmRL, pwmFreq, pwmResolution);
  ledcSetup(pwmRR, pwmFreq, pwmResolution);

  ledcAttachPin(FL_ENA, pwmFL);
  ledcAttachPin(RL_ENA, pwmRL);
  ledcAttachPin(FR_ENA, pwmFR);
  ledcAttachPin(RR_ENA, pwmRR);

  DEBUG_SERIAL.println("All systems initialized!");
}

void loop() {
  unsigned long currentTime = millis();

  // Check for incoming motor commands (asynchronous)
  if (currentTime - lastCommandCheckTime >= COMMAND_CHECK_INTERVAL) {
    lastCommandCheckTime = currentTime;
    checkForCommands();
  }
  delay(2);
}

// void loop() {
//   // Move forward: both sides drive forward at speed 100
//   moveDiffMotors(90, 90);
//   delay(1000);  // Move forward for 2 seconds

//   // Rotate anticlockwise: left side backward, right side forward
//   moveDiffMotors(-90, 150);
//   delay(1000);  // Rotate for 2 seconds

//   // Stop all motors
//   stopMotors();
//   delay(20000);  // Wait for 2 seconds before repeating the test loop
// }


// Function to check for incoming commands from RPi
void checkForCommands() {
  if (RpiSerial.available()) {
    String command = RpiSerial.readStringUntil('\r\n');
    
    // Expect command format: "PWM:DIFF:<leftSpeed>,<rightSpeed>"
    if (command.startsWith("PWM:DIFF:")) {
      String values = command.substring(9); // Remove "PWM:DIFF:" prefix
      int commaIndex = values.indexOf(',');
      if (commaIndex != -1) {
          int leftSpeed = values.substring(0, commaIndex).toInt();
          int rightSpeed = values.substring(commaIndex + 1).toInt();
          moveDiffMotors(leftSpeed, rightSpeed);
      } else {
         // If no comma is found, use the same speed for both sides
         int speed = values.toInt();
         moveDiffMotors(speed, speed);
      }
    }
  }
}

// Function to control left side motors (both front and rear left)
void moveLeftMotors(int speed) {
  if (speed > 0) { // Forward
    digitalWrite(FL_IN1, HIGH);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(RL_IN1, HIGH);
    digitalWrite(RL_IN2, LOW);
  } else if (speed < 0) { // Backward
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, HIGH);
    digitalWrite(RL_IN1, LOW);
    digitalWrite(RL_IN2, HIGH);
  } else { // Stop
    digitalWrite(FL_IN1, LOW);
    digitalWrite(FL_IN2, LOW);
    digitalWrite(RL_IN1, LOW);
    digitalWrite(RL_IN2, LOW);
  }
  int pwmValue = abs(speed);
  ledcWrite(pwmFL, pwmValue);
  ledcWrite(pwmRL, pwmValue);
}

// Function to control right side motors (both front and rear right)
void moveRightMotors(int speed) {
  if (speed > 0) { // Forward
    digitalWrite(FR_IN1, HIGH);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(RR_IN1, HIGH);
    digitalWrite(RR_IN2, LOW);
  } else if (speed < 0) { // Backward
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, HIGH);
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, HIGH);
  } else { // Stop
    digitalWrite(FR_IN1, LOW);
    digitalWrite(FR_IN2, LOW);
    digitalWrite(RR_IN1, LOW);
    digitalWrite(RR_IN2, LOW);
  }
  int pwmValue = abs(speed);
  ledcWrite(pwmFR, pwmValue);
  ledcWrite(pwmRR, pwmValue);
}

// Function to control differential drive motors by setting left and right speeds
void moveDiffMotors(int leftSpeed, int rightSpeed) {
  DEBUG_SERIAL.print("Left Speed: ");
  DEBUG_SERIAL.print(leftSpeed);
  DEBUG_SERIAL.print(" | Right Speed: ");
  DEBUG_SERIAL.println(rightSpeed);
  moveLeftMotors(leftSpeed);
  moveRightMotors(rightSpeed);
}

// Function to stop all motors
void stopMotors() {
  DEBUG_SERIAL.println("Stopping all motors...");
  // Stop left motors
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  ledcWrite(pwmFL, 0);
  ledcWrite(pwmRL, 0);
  // Stop right motors
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  ledcWrite(pwmFR, 0);
  ledcWrite(pwmRR, 0);
}
