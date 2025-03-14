#include <Arduino.h>

// Use UART1 for communication. We'll use only the RX pin (set TX to -1).
HardwareSerial MySerial(1);

// Pin Definitions for ESP32-S3 Feather with L298N
#define FL_IN1 2   // Front Left IN1
#define FL_IN2 42  // Front Left IN2
#define FL_ENA 1  // Front Left PWM

#define FR_IN1 40   // Front Right IN1
#define FR_IN2 39   // Front Right IN2
#define FR_ENA 38   // Front Right PWM

#define RL_IN1 6  // Rear Left IN1
#define RL_IN2 7  // Rear Left IN2
#define RL_ENA 5  // Rear Left PWM

#define RR_IN1 16  // Rear Right IN1
#define RR_IN2 17  // Rear Right IN2
#define RR_ENA 18   // Rear Right PWM

// Define PWM Parameters
const int pwmFreq = 1000;         // PWM frequency
const int pwmResolution = 8;      // PWM resolution (8 bits = 0 to 255)

// Define PWM channels for motors
const int pwmFL = 0;  // Front Left
const int pwmFR = 1;  // Front Right
const int pwmRL = 2;  // Rear Left
const int pwmRR = 3;  // Rear Right

// Define Direction Byte for Mecanum Modes
const byte MEC_STRAIGHT_FORWARD = B10101010;
const byte MEC_STRAIGHT_BACKWARD = B01010101;
const byte MEC_SIDEWAYS_RIGHT = B01101001;
const byte MEC_SIDEWAYS_LEFT = B10010110;
const byte MEC_ROTATE_CLOCKWISE = B01100110;
const byte MEC_ROTATE_COUNTERCLOCKWISE = B10011001;

// Define PWM Motor Speed Variables
int rf_PWM = 100;  // Right Front Motor
int lf_PWM = 100;  // Left Front Motor
int rr_PWM = 100;  // Right Rear Motor
int lr_PWM = 100;  // Left Rear Motor

void setup() {
  Serial.begin(115200); // Start Serial Logging
  MySerial.begin(115200, SERIAL_8N1, 44, -1);

  Serial.println("Initializing Motor Pins...");
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_ENA, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_ENA, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_ENA, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_ENA, OUTPUT);

  Serial.println("Setting up PWM channels...");
  ledcSetup(pwmFL, pwmFreq, pwmResolution);
  ledcSetup(pwmFR, pwmFreq, pwmResolution);
  ledcSetup(pwmRL, pwmFreq, pwmResolution);
  ledcSetup(pwmRR, pwmFreq, pwmResolution);

  ledcAttachPin(FL_ENA, pwmFL);
  ledcAttachPin(FR_ENA, pwmFR);
  ledcAttachPin(RL_ENA, pwmRL);
  ledcAttachPin(RR_ENA, pwmRR);

  Serial.println("Setup Complete!");
}

void loop() {
  if (MySerial.available()) {
    String command = MySerial.readStringUntil('\r\n');
    
    if (command.startsWith("PWM:F:")) {  // Forward/Backward command
      int pwmValue = command.substring(6).toInt();
      Serial.println(pwmValue);
      if (pwmValue > 0) {
        moveMotors(abs(pwmValue), abs(pwmValue), abs(pwmValue), abs(pwmValue), MEC_STRAIGHT_FORWARD);
      } else if (pwmValue < 0) {
        moveMotors(abs(pwmValue), abs(pwmValue), abs(pwmValue), abs(pwmValue), MEC_STRAIGHT_BACKWARD);
      } else {
        stopMotors();
      }
    }
    else if (command.startsWith("PWM:R:")) {  // Rotation command
      int pwmValue = command.substring(6).toInt();
      Serial.println(pwmValue);
      if (pwmValue > 0) {
        moveMotors(abs(pwmValue), abs(pwmValue), abs(pwmValue), abs(pwmValue), MEC_ROTATE_CLOCKWISE);
      } else if (pwmValue < 0) {
        moveMotors(abs(pwmValue), abs(pwmValue), abs(pwmValue), abs(pwmValue), MEC_ROTATE_COUNTERCLOCKWISE);
      } else {
        stopMotors();
      }
    }
  }
  delay(10);
}

// Function to move motors
void moveMotors(int speedFL, int speedFR, int speedRL, int speedRR, byte direction) {
  Serial.print("Moving Motors -> ");
  Serial.print("FL: "); Serial.print(speedFL);
  Serial.print(", FR: "); Serial.print(speedFR);
  Serial.print(", RL: "); Serial.print(speedRL);
  Serial.print(", RR: "); Serial.print(speedRR);
  Serial.print(" | Direction: "); Serial.println(direction, BIN);

  // Front Left Motor
  digitalWrite(FL_IN1, bitRead(direction, 7));
  digitalWrite(FL_IN2, bitRead(direction, 6));
  ledcWrite(pwmFL, abs(speedFL));

  // Front Right Motor
  digitalWrite(FR_IN1, bitRead(direction, 5));
  digitalWrite(FR_IN2, bitRead(direction, 4));
  ledcWrite(pwmFR, abs(speedFR));

  // Rear Left Motor
  digitalWrite(RL_IN1, bitRead(direction, 3));
  digitalWrite(RL_IN2, bitRead(direction, 2));
  ledcWrite(pwmRL, abs(speedRL));

  // Rear Right Motor
  digitalWrite(RR_IN1, bitRead(direction, 1));
  digitalWrite(RR_IN2, bitRead(direction, 0));
  ledcWrite(pwmRR, abs(speedRR));
}

// Function to stop all motors
void stopMotors() {
  Serial.println("Stopping all motors...");

  ledcWrite(pwmFL, 0);
  ledcWrite(pwmFR, 0);
  ledcWrite(pwmRL, 0);
  ledcWrite(pwmRR, 0);

  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
}
