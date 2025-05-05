#include <Arduino.h>
#include <math.h>
// #include "driver/ledc.h"

// ====== Serial Definitions ======
#define DEBUG_SERIAL Serial
// HardwareSerial RpiSerial(1);  // Use UART1 for odometry to RPi

// ====== Pin Definitions ======
// Motor 1
#define ENCODER1_PIN_A    36
#define ENCODER1_PIN_B    37
#define MOTOR1_IN1        7    // PWM-capable
#define MOTOR1_IN2        6    // PWM-capable

// Motor 2
#define ENCODER2_PIN_A    48
#define ENCODER2_PIN_B    47
#define MOTOR2_IN1        5
#define MOTOR2_IN2        4

// ====== PWM Configuration ======
#define CH1_FWD           0
#define CH1_REV           1
#define CH2_FWD           2
#define CH2_REV           3
#define PWM_FREQ         20000
#define PWM_RES          8

// ====== Robot & Control Constants ======
const float WHEEL_RADIUS = 0.08f;
const float WHEEL_BASE = 0.20f;
const int TICKS_PER_REV = 400;
const float ANGLE_PER_TICK = 2 * PI / TICKS_PER_REV;

const unsigned long SAMPLE_INTERVAL = 10;  // 10ms (100Hz) for motor control
const unsigned long ODOM_INTERVAL = 20;    // 20ms (50Hz) for odometry updates - reduced from continuous updates
const float h = SAMPLE_INTERVAL / 1000.0f;

const float Kp = 285.0f;
const float Ki = 15.0f;
const float MAX_INTEGRAL = 50.0f;
const int DEADBAND = 10;
const float VELOCITY_DEADBAND = 0.1f;  // rad/s - ignore velocity commands smaller than this

const float EMA_ALPHA = 0.05f;
const unsigned long STARTUP_DELAY = 500;

// ====== Globals ======
volatile long encoderCount1 = 0, encoderCount2 = 0;
volatile int lastEnc1 = 0, lastEnc2 = 0;

float buf1[5] = {0}, buf2[5] = {0};
int bufCount = 0;

float velFilt1 = 0, velFilt2 = 0;
bool emaInit1 = false, emaInit2 = false;

unsigned long lastSample = 0, startTime = 0, lastOdomTime = 0;

float integral1 = 0, integral2 = 0;
float targetW1 = 0.0f, targetW2 = 0.0f;

float x = 0.0f, y = 0.0f, theta = 0.0f;
float vx = 0.0f, vy = 0.0f, omega = 0.0f;
long lastLeftCount = 0, lastRightCount = 0;
unsigned long lastOdomUpdate = 0;

// Buffer for incoming serial commands
char serialBuffer[64];
uint8_t bufferIndex = 0;

void IRAM_ATTR updateEnc1();
void IRAM_ATTR updateEnc2();
float computeVel(const float buf[5], float dt);
void setPWM(int ch_fwd, int ch_rev, int pwmVal);
void processSerialCommands();
void sendOdometry();
void updateOdometry();

void setup() {
  DEBUG_SERIAL.begin(115200);
  // RpiSerial.begin(115200, SERIAL_8N1, 44, 43); // default pins if not specified

  pinMode(ENCODER1_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER1_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_A), updateEnc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN_B), updateEnc1, CHANGE);

  pinMode(ENCODER2_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER2_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_A), updateEnc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN_B), updateEnc2, CHANGE);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);

  ledcSetup(CH1_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR1_IN1, CH1_FWD);
  ledcSetup(CH1_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR1_IN2, CH1_REV);
  ledcSetup(CH2_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR2_IN1, CH2_FWD);
  ledcSetup(CH2_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR2_IN2, CH2_REV);

  encoderCount1 = encoderCount2 = 0;
  lastEnc1 = lastEnc2 = 0;
  lastLeftCount = lastRightCount = 0;
  lastOdomUpdate = millis();
  lastSample = startTime = lastOdomTime = millis();

  DEBUG_SERIAL.println("ROS2 differential drive controller initialized");
}

void loop() {
  unsigned long now = millis();

  // Process any available serial commands right away
  processSerialCommands();

  // Check for motor control update time
  if (now - lastSample >= SAMPLE_INTERVAL) {
    // Always update odometry when we update motor control
    updateOdometry();
    
    float dt = (now - lastSample) / 1000.0f;
    
    // Motor control computations
    float ang1 = encoderCount1 * ANGLE_PER_TICK;
    float ang2 = encoderCount2 * ANGLE_PER_TICK;
    
    // Update velocity filter buffers
    for (int i = 0; i < 4; i++) {
      buf1[i] = buf1[i+1];
      buf2[i] = buf2[i+1];
    }
    buf1[4] = ang1;
    buf2[4] = ang2;
    if (bufCount < 5) bufCount++;

    // Compute wheel velocities
    float omega1 = 0, omega2 = 0;
    if (bufCount >= 5) {
      omega1 = computeVel(buf1, h);
      omega2 = computeVel(buf2, h);
      
      if (!emaInit1) { velFilt1 = omega1; emaInit1 = true; }
      else { velFilt1 = EMA_ALPHA*omega1 + (1-EMA_ALPHA)*velFilt1; }
      
      if (!emaInit2) { velFilt2 = omega2; emaInit2 = true; }
      else { velFilt2 = EMA_ALPHA*omega2 + (1-EMA_ALPHA)*velFilt2; }
    }

    // Calculate motor control signals using PI controller
    float err1 = targetW1 - velFilt1;
    integral1 = constrain(integral1 + err1*dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    int pwm1 = int(Kp*err1 + Ki*integral1);
    if (pwm1 > 0 && pwm1 < DEADBAND) pwm1 = DEADBAND;
    else if (pwm1 < 0 && pwm1 > -DEADBAND) pwm1 = -DEADBAND;
    pwm1 = constrain(pwm1, -255, 255);

    float err2 = targetW2 - velFilt2;
    integral2 = constrain(integral2 + err2*dt, -MAX_INTEGRAL, MAX_INTEGRAL);
    int pwm2 = int(Kp*err2 + Ki*integral2);
    if (pwm2 > 0 && pwm2 < DEADBAND) pwm2 = DEADBAND;
    else if (pwm2 < 0 && pwm2 > -DEADBAND) pwm2 = -DEADBAND;
    pwm2 = constrain(pwm2, -255, 255);

    // Apply motor control
    if (now - startTime >= STARTUP_DELAY) {
      setPWM(CH1_FWD, CH1_REV, pwm1);
      setPWM(CH2_FWD, CH2_REV, pwm2);
    } else {
      setPWM(CH1_FWD, CH1_REV, 0);
      setPWM(CH2_FWD, CH2_REV, 0);
    }

    lastSample = now;
  }

  // Limit odometry transmission rate
  if (now - lastOdomTime >= ODOM_INTERVAL) {
    sendOdometry();
    lastOdomTime = now;
  }

  // Safety check for encoder count overflow
  if (abs(encoderCount2) > 1000000) {
    // Instead of logging (which slows things down), just reset
    encoderCount2 = 0;
  }
  
  // Using a shorter delay for better responsiveness
  delay(1);
}

// ===== Interrupts and Helpers =====

void IRAM_ATTR updateEnc1() {
  int msb = digitalRead(ENCODER1_PIN_A);
  int lsb = digitalRead(ENCODER1_PIN_B);
  int code = (lastEnc1<<2) | ((msb<<1)|lsb);
  if (code==0b1101||code==0b0100||code==0b0010||code==0b1011) encoderCount1++;
  else if (code==0b1110||code==0b0111||code==0b0001||code==0b1000) encoderCount1--;
  lastEnc1 = (msb<<1)|lsb;
}

void IRAM_ATTR updateEnc2() {
  int msb = digitalRead(ENCODER2_PIN_A);
  int lsb = digitalRead(ENCODER2_PIN_B);
  int code = (lastEnc2<<2) | ((msb<<1)|lsb);
  if (code==0b1101||code==0b0100||code==0b0010||code==0b1011) encoderCount2++;
  else if (code==0b1110||code==0b0111||code==0b0001||code==0b1000) encoderCount2--;
  lastEnc2 = (msb<<1)|lsb;
}

float computeVel(const float buf[5], float dt) {
  return (25*buf[4] - 48*buf[3] + 36*buf[2] - 16*buf[1] + 3*buf[0]) / (12.0f * dt);
}

void setPWM(int ch_fwd, int ch_rev, int pwmVal) {
  pwmVal = constrain(pwmVal, -255, 255);
  uint8_t mag = abs(pwmVal);
  if (pwmVal >= 0) {
    ledcWrite(ch_fwd, mag);
    ledcWrite(ch_rev, 0);
  } else {
    ledcWrite(ch_fwd, 0);
    ledcWrite(ch_rev, mag);
  }
}

void processSerialCommands() {
  while (DEBUG_SERIAL.available() > 0) {
    char c = DEBUG_SERIAL.read();
    
    // Store character if not end of line and buffer not full
    if (c != '\n' && bufferIndex < sizeof(serialBuffer) - 1) {
      serialBuffer[bufferIndex++] = c;
    } 
    // Process the command when we get a newline or buffer is full
    else {
      serialBuffer[bufferIndex] = '\0';  // Null terminate
      
      // Process velocity command
      if (strstr(serialBuffer, "cmd_vel:") == serialBuffer) {
        float lin = 0, ang = 0;
        if (sscanf(serialBuffer, "cmd_vel: %f,%f", &lin, &ang) == 2) {
          float vLeft = lin - (ang * WHEEL_BASE) / 2.0f;
          float vRight = lin + (ang * WHEEL_BASE) / 2.0f;
          
          // Apply velocity deadband to linear and angular velocities
          if (fabs(lin) < VELOCITY_DEADBAND) lin = 0;
          if (fabs(ang) < VELOCITY_DEADBAND) ang = 0;
          
          // Recalculate wheel velocities after deadband
          if (lin == 0 && ang == 0) {
            targetW1 = 0;
            targetW2 = 0;
          } else {
            vLeft = lin - (ang * WHEEL_BASE) / 2.0f;
            vRight = lin + (ang * WHEEL_BASE) / 2.0f;
            targetW1 = vLeft / WHEEL_RADIUS;
            targetW2 = vRight / WHEEL_RADIUS;
          }
          
          // Send acknowledgment but only if values actually changed
          static float lastTargetW1 = 0, lastTargetW2 = 0;
          if (abs(targetW1 - lastTargetW1) > 0.01 || abs(targetW2 - lastTargetW2) > 0.01) {
            DEBUG_SERIAL.printf("New targets: left=%.2f rad/s, right=%.2f rad/s\n", targetW1, targetW2);
            lastTargetW1 = targetW1;
            lastTargetW2 = targetW2;
          }
        }
      }
      
      // Reset buffer for next command
      bufferIndex = 0;
    }
  }
}

void updateOdometry() {
  long curLeftCount = encoderCount1;
  long curRightCount = encoderCount2;
  float deltaLeft = (curLeftCount - lastLeftCount) * ANGLE_PER_TICK;
  float deltaRight = (curRightCount - lastRightCount) * ANGLE_PER_TICK;
  float distLeft = deltaLeft * WHEEL_RADIUS;
  float distRight = deltaRight * WHEEL_RADIUS;
  float distCenter = (distLeft + distRight) / 2.0f;
  float dTheta = (distRight - distLeft) / WHEEL_BASE;

  if (fabs(dTheta) < 0.0001) {
    x += distCenter * cos(theta);
    y += distCenter * sin(theta);
  } else {
    float r = distCenter / dTheta;
    x += r * (sin(theta + dTheta) - sin(theta));
    y += r * (-cos(theta + dTheta) + cos(theta));
  }

  theta += dTheta;
  while (theta > PI) theta -= 2 * PI;
  while (theta < -PI) theta += 2 * PI;

  unsigned long curTime = millis();
  float dt = (curTime - lastOdomUpdate) / 1000.0f;
  if (dt > 0) {
    vx = distCenter / dt;
    vy = 0.0f;
    omega = dTheta / dt;
  }

  lastLeftCount = curLeftCount;
  lastRightCount = curRightCount;
  lastOdomUpdate = curTime;
}

void sendOdometry() {
  // Only send if there's actual data change to reduce transmission load
  static float last_x = 0, last_y = 0, last_theta = 0;
  static float last_vx = 0, last_vy = 0, last_omega = 0;
  
  // Check if the values have changed significantly before sending
  bool significant_change = 
       fabs(x - last_x) > 0.001 
    || fabs(y - last_y) > 0.001 
    || fabs(theta - last_theta) > 0.005
    || fabs(vx - last_vx) > 0.01
    || fabs(omega - last_omega) > 0.01;
  
  if (significant_change) {
    char odomMsg[128];
    
    // Format with fewer decimal places to reduce transmission size
    sprintf(odomMsg, "odom:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
            x, y, theta, vx, vy, omega);
    DEBUG_SERIAL.print(odomMsg); // Use print instead of println to save one byte
    
    // Update last values
    last_x = x;
    last_y = y;
    last_theta = theta;
    last_vx = vx;
    last_vy = vy;
    last_omega = omega;
  }
}
