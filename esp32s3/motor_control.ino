#include <Arduino.h>
#include <driver/pcnt.h>
#include <esp_timer.h>
#include <math.h>

/* ── Pinout ──────────────────────────────────────────────────────────────── */
#define ENC1_A 36
#define ENC1_B 37
#define ENC2_A 48
#define ENC2_B 47
#define M1_IN1 7   // PWM‑capable
#define M1_IN2 6
#define M2_IN1 5
#define M2_IN2 4

/* ── PWM (20 kHz @ 8‑bit) ────────────────────────────────────────────────── */
#define CH1_FWD 0
#define CH1_REV 1
#define CH2_FWD 2
#define CH2_REV 3
const uint32_t PWM_FREQ  = 20000;
const uint8_t  PWM_RES   = 8;
const uint16_t PWM_RANGE = 255;

/* ── Robot geometry & encoder ───────────────────────────────────────────── */
const float RADIUS         = 0.04f;            // m
const float BASE           = 0.22f;            // m
const int   TICKS_PER_REV  = 800;              // ticks/rev
const float RAD_PER_TICK   = 2*PI / TICKS_PER_REV;

/* ── Timers ──────────────────────────────────────────────────────────────── */
const uint32_t DT_MS     = 5;        // control loop period (ms)
const int64_t  DT_US     = DT_MS * 1000LL;
const float    DT        = DT_MS / 1000.0f;
const uint32_t ODOM_MS   = 10;       // odom send interval

/* ── PID & motion limits ────────────────────────────────────────────────── */
const float Kp               = 280.0f;
const float Ki               = 15.0f;
const float Kd               = 2.0f;
const float I_MAX            = 50.0f;
const int   DEADBAND         = 30;
const float ACC_LIMIT        = 4.0f;    // rad/s²
const float INT_FREEZE_THRESH = 0.2f;   // rad/s

/* ── Velocity filtering ─────────────────────────────────────────────────── */
const float EMA_ALPHA    = 0.03f;  // ~10 Hz

/* ── Safety parameters ───────────────────────────────────────────────────── */
const uint32_t ENC_WATCHDOG_MS = 100;    // Max time without encoder change (ms)
const int16_t MAX_ENC_DELTA = 100;       // Max expected encoder delta per cycle
const uint8_t MAX_ZERO_ENC_COUNT = 10;   // Max consecutive zero encoder readings before alert
const float MAX_VELOCITY_ERROR = 5.0f;   // Max rad/s error before emergency stop
const uint32_t COMM_TIMEOUT_MS = 500;    // Timeout for serial commands (ms)

/* ── Globals ───────────────────────────────────────────────────────────────*/
static int64_t totalEnc1 = 0, totalEnc2 = 0;
static int64_t lastTotalEnc1 = 0, lastTotalEnc2 = 0;
float filtW1 = 0, filtW2 = 0;
float integ1 = 0, integ2 = 0;
float rampW1 = 0, rampW2 = 0;
float setW1 = 0, setW2 = 0;

int64_t lastTimeUs       = 0;
unsigned long lastOdomMs = 0;
unsigned long lastCommandMs = 0;
unsigned long lastEnc1ChangeMs = 0;
unsigned long lastEnc2ChangeMs = 0;
int16_t lastEncCount1 = 0, lastEncCount2 = 0;
uint8_t zeroEnc1Count = 0, zeroEnc2Count = 0;
bool emergencyStop = false;
bool encoderError = false;

static float buf1[5] = {0}, buf2[5] = {0};
static int   bufCount = 0;

/* ── Odometry state ───────────────────────────────────────────────────────*/
float x = 0, y = 0, theta = 0;
float vx = 0, vy = 0, omega = 0;

/* ── Serial command buffer ─────────────────────────────────────────────────*/
char    serialBuffer[64];
uint8_t bufferIndex = 0;

/* ── Helpers ───────────────────────────────────────────────────────────────*/
float fiveTap(const float b[5]) {
  return (25*b[4] - 48*b[3] + 36*b[2] - 16*b[1] + 3*b[0]) / 12.0f;
}

void stopMotors() {
  ledcWrite(CH1_FWD, 0);
  ledcWrite(CH1_REV, 0);
  ledcWrite(CH2_FWD, 0);
  ledcWrite(CH2_REV, 0);
}

void clearPIDState() {
  integ1 = 0;
  integ2 = 0;
  rampW1 = 0;
  rampW2 = 0;
  filtW1 = 0;
  filtW2 = 0;
}

void setPWM(int chFwd, int chRev, int val) {
  if (emergencyStop || encoderError) {
    stopMotors();
    return;
  }
  
  val = constrain(val, -PWM_RANGE, PWM_RANGE);
  uint16_t duty = abs(val);
  if (val >= 0) {
    ledcWrite(chFwd, duty);
    ledcWrite(chRev, 0);
  } else {
    ledcWrite(chFwd, 0);
    ledcWrite(chRev, duty);
  }
}

void resetEmergencyStop() {
  emergencyStop = false;
  encoderError = false;
  clearPIDState();
  stopMotors();
  Serial.println("SAFETY: Emergency stop reset");
}

void triggerEmergencyStop(const char* reason) {
  emergencyStop = true;
  stopMotors();
  clearPIDState();
  Serial.print("EMERGENCY STOP: ");
  Serial.println(reason);
}

void sendOdometry() {
  char msg[96];
  sprintf(msg, "odom:%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
          x, y, theta, vx, vy, omega);
  Serial.print(msg);
}

void sendStatus() {
  char msg[128];
  sprintf(msg, "status:enc1=%lld,enc2=%lld,w1=%.2f,w2=%.2f,set1=%.2f,set2=%.2f,e-stop=%d,enc-err=%d\n",
          totalEnc1, totalEnc2, filtW1, filtW2, setW1, setW2, emergencyStop ? 1 : 0, encoderError ? 1 : 0);
  Serial.print(msg);
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && bufferIndex < sizeof(serialBuffer)-1) {
      serialBuffer[bufferIndex++] = c;
    } else {
      serialBuffer[bufferIndex] = '\0';
      if (strncmp(serialBuffer, "cmd_vel:", 8) == 0) {
        float lin, ang;
        if (sscanf(serialBuffer+8, " %f,%f", &lin, &ang) == 2) {
          if (fabs(lin) < INT_FREEZE_THRESH) lin = 0;
          if (fabs(ang) < INT_FREEZE_THRESH) ang = 0;
          float vL = lin + (ang * BASE)/2.0f;
          float vR = lin - (ang * BASE)/2.0f;
          setW1 = vL / RADIUS;
          setW2 = vR / RADIUS;
          lastCommandMs = millis();
        }
      } else if (strcmp(serialBuffer, "reset") == 0) {
        resetEmergencyStop();
      } else if (strcmp(serialBuffer, "stop") == 0) {
        triggerEmergencyStop("User requested stop");
      } else if (strcmp(serialBuffer, "status") == 0) {
        sendStatus();
      }
      bufferIndex = 0;
    }
  }
}

bool checkEncoderSanity(int16_t cnt1, int16_t cnt2) {
  unsigned long now = millis();
  bool enc1_ok = true;
  bool enc2_ok = true;
  
  // Check for unreasonable encoder counts
  if (abs(cnt1) > MAX_ENC_DELTA) {
    Serial.print("WARN: Encoder 1 delta too large: ");
    Serial.println(cnt1);
    enc1_ok = false;
  }
  
  if (abs(cnt2) > MAX_ENC_DELTA) {
    Serial.print("WARN: Encoder 2 delta too large: ");
    Serial.println(cnt2);
    enc2_ok = false;
  }
  
  // Track zero counts
  if (cnt1 == 0 && setW1 != 0) {
    zeroEnc1Count++;
    if (zeroEnc1Count > MAX_ZERO_ENC_COUNT) {
      Serial.println("WARN: Encoder 1 not responding");
      enc1_ok = false;
    }
  } else {
    zeroEnc1Count = 0;
    lastEncCount1 = cnt1;
    lastEnc1ChangeMs = now;
  }
  
  if (cnt2 == 0 && setW2 != 0) {
    zeroEnc2Count++;
    if (zeroEnc2Count > MAX_ZERO_ENC_COUNT) {
      Serial.println("WARN: Encoder 2 not responding");
      enc2_ok = false;
    }
  } else {
    zeroEnc2Count = 0;
    lastEncCount2 = cnt2;
    lastEnc2ChangeMs = now;
  }
  
  // Check watchdog timeouts
  if (fabs(setW1) > 0.1 && now - lastEnc1ChangeMs > ENC_WATCHDOG_MS) {
    Serial.println("ERROR: Encoder 1 timeout");
    enc1_ok = false;
  }
  
  if (fabs(setW2) > 0.1 && now - lastEnc2ChangeMs > ENC_WATCHDOG_MS) {
    Serial.println("ERROR: Encoder 2 timeout");
    enc2_ok = false;
  }
  
  return enc1_ok && enc2_ok;
}

// Add this near the top with your other global variables
unsigned long velocityTestTimer = 0;
bool testModeActive = false;

// Add this function somewhere in your code
// ── Test Velocities (edit these to whatever you want) ────────────────────
const float TEST_LINEAR_VEL  = 0.7f;  // m/s
const float TEST_ANGULAR_VEL = 0.0f;  // rad/s

// ── Simplified test function ─────────────────────────────────────────────
void runSimpleVelocityTest() {
  // convert to wheel rates:
  float vL = TEST_LINEAR_VEL - (TEST_ANGULAR_VEL * BASE) * 0.5f;
  float vR = TEST_LINEAR_VEL + (TEST_ANGULAR_VEL * BASE) * 0.5f;
  setW1 = vL / RADIUS;
  setW2 = vR / RADIUS;

  Serial.print("Test → lin:");
  Serial.print(TEST_LINEAR_VEL);
  Serial.print(" m/s, ang:");
  Serial.print(TEST_ANGULAR_VEL);
  Serial.println(" rad/s");
}

// Add this function to output plottable data
void outputPlotData(int p1, int p2) {
  // Print in format suitable for Arduino Serial Plotter
  Serial.print("CMD1:");
  Serial.print(setW1);
  Serial.print(",CMD2:");
  Serial.print(setW2);
  Serial.print(",ACT1:");
  Serial.print(filtW1);
  Serial.print(",ACT2:");
  Serial.print(filtW2);
  Serial.print(",PWM1:");
  Serial.print(p1);
  Serial.print(",PWM2:");
  Serial.println(p2);
}






/* ── Setup ─────────────────────────────────────────────────────────────────*/
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);  // Wait for serial port to connect
  }

  //── Configure PCNT for ENC1 (unit 0) ─────────────────────────────────────
  pcnt_config_t cfg = {};
  cfg.ctrl_gpio_num  = ENC1_B;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_REVERSE;
  cfg.counter_h_lim  = INT16_MAX;
  cfg.counter_l_lim  = INT16_MIN;

  // ENC1 channel 0
  cfg.pulse_gpio_num = ENC1_A;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = PCNT_UNIT_0;
  cfg.pos_mode       = PCNT_COUNT_DEC;
  cfg.neg_mode       = PCNT_COUNT_INC;
  pcnt_unit_config(&cfg);
  // ENC1 channel 1 (inverse)
  cfg.pulse_gpio_num = ENC1_B;
  cfg.ctrl_gpio_num  = ENC1_A;
  cfg.channel        = PCNT_CHANNEL_1;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DEC;
  pcnt_unit_config(&cfg);

  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);

  //── Configure PCNT for ENC2 (unit 1) ─────────────────────────────────────
  cfg.unit = PCNT_UNIT_1;
  cfg.pulse_gpio_num = ENC2_A;
  cfg.ctrl_gpio_num  = ENC2_B;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.pos_mode       = PCNT_COUNT_DEC;
  cfg.neg_mode       = PCNT_COUNT_INC;
  pcnt_unit_config(&cfg);
  cfg.pulse_gpio_num = ENC2_B;
  cfg.ctrl_gpio_num  = ENC2_A;
  cfg.channel        = PCNT_CHANNEL_1;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DEC;
  pcnt_unit_config(&cfg);

  pcnt_counter_pause(PCNT_UNIT_1);
  pcnt_counter_clear(PCNT_UNIT_1);
  pcnt_counter_resume(PCNT_UNIT_1);

  //── PWM channels ────────────────────────────────────────────────────────
  ledcSetup(CH1_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN1, CH1_FWD);
  ledcSetup(CH1_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(M1_IN2, CH1_REV);
  ledcSetup(CH2_FWD, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_IN1, CH2_FWD);
  ledcSetup(CH2_REV, PWM_FREQ, PWM_RES);
  ledcAttachPin(M2_IN2, CH2_REV);

  stopMotors();  // Ensure motors are stopped on startup
  lastTimeUs = esp_timer_get_time();
  lastOdomMs = millis();
  lastCommandMs = millis();
  lastEnc1ChangeMs = millis();
  lastEnc2ChangeMs = millis();

  
  Serial.println("ESP32S3 diff‑drive ready");
  Serial.println("Safety systems active");

  velocityTestTimer = millis();
  
  // Optionally start in test mode automatically
  testModeActive = false;
}

/* ── Main loop ────────────────────────────────────────────────────────────*/
void loop() {

  if (testModeActive) {
    runSimpleVelocityTest();
  }
  // 1) Handle incoming velocity commands
  processSerialCommands();
  
  // 3) Enforce 5 ms period
  int64_t nowUs = esp_timer_get_time();
  if (nowUs - lastTimeUs < DT_US) return;
  lastTimeUs = nowUs;

  // 4) Read & clear encoder counts
  int16_t cnt1=0, cnt2=0;
  pcnt_get_counter_value(PCNT_UNIT_0, &cnt1);
  pcnt_get_counter_value(PCNT_UNIT_1, &cnt2);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_1);
  
  // 5) Check encoder sanity
  // bool encodersOk = checkEncoderSanity(cnt1, cnt2);
  
  // if (!encodersOk && !encoderError) {
    // encoderError = true;
    // triggerEmergencyStop("Encoder failure detected");
  // } else if (encodersOk && encoderError && !emergencyStop) {
    // Automatically recover from encoder errors if they resolve themselves
    // and there's no other emergency stop condition
    // encoderError = false; 
    // Serial.println("SAFETY: Encoder error resolved");
  // }
  
  // Only update counters if encoders are okay or we want to track position anyway
  totalEnc1 += cnt1;
  totalEnc2 += cnt2;

  // 6) Velocity estimation (5‑tap + EMA)
  float ang1 = totalEnc1 * RAD_PER_TICK;
  float ang2 = totalEnc2 * RAD_PER_TICK;
  for (int i = 0; i < 4; i++) {
    buf1[i] = buf1[i+1];
    buf2[i] = buf2[i+1];
  }
  buf1[4] = ang1;  buf2[4] = ang2;
  if (bufCount < 5) {
    bufCount++;
  } else {
    float w1 = fiveTap(buf1)/DT;
    float w2 = fiveTap(buf2)/DT;
    filtW1 = EMA_ALPHA*w1 + (1-EMA_ALPHA)*filtW1;
    filtW2 = EMA_ALPHA*w2 + (1-EMA_ALPHA)*filtW2;
  }

  // 7) Slew‑rate limiter (soft start)
  float step = ACC_LIMIT * DT;
  rampW1 += constrain(setW1 - rampW1, -step, step);
  rampW2 += constrain(setW2 - rampW2, -step, step);

  // 8) PID w/ D‑term - only run if not in emergency stop
  int p1 = 0, p2 = 0;
  if (!emergencyStop && !encoderError) {
    static float lastE1=0, lastE2=0, fde1=0, fde2=0;
    float e1 = rampW1 - filtW1;
    float e2 = rampW2 - filtW2;
    
    // Only update integrator if running at speed
    if (fabs(rampW1) > INT_FREEZE_THRESH)
      integ1 = constrain(integ1 + e1*DT, -I_MAX, I_MAX);
    if (fabs(rampW2) > INT_FREEZE_THRESH)
      integ2 = constrain(integ2 + e2*DT, -I_MAX, I_MAX);
      
    float de1 = (e1 - lastE1)/DT, de2 = (e2 - lastE2)/DT;
    lastE1 = e1;  lastE2 = e2;
    fde1 = 0.1f*de1 + 0.9f*fde1;
    fde2 = 0.1f*de2 + 0.9f*fde2;
    p1 = int(Kp*e1 + Ki*integ1 + Kd*fde1);
    p2 = int(Kp*e2 + Ki*integ2 + Kd*fde2);
    
    // Apply deadband
    if (p1>0 && p1<DEADBAND) p1=DEADBAND;
    if (p1<0 && p1>-DEADBAND) p1=-DEADBAND;
    if (p2>0 && p2<DEADBAND) p2=DEADBAND;
    if (p2<0 && p2>-DEADBAND) p2=-DEADBAND;
    
    // Apply motor commands - safety checks happen inside setPWM
    setPWM(CH1_FWD, CH1_REV, p1);
    setPWM(CH2_FWD, CH2_REV, p2);
  } else {
    // Safety: Ensure motors are off in emergency mode
    stopMotors();
  }

  // 9) Debug output
  static unsigned long lastDebugMs = 0;
  unsigned long nowMs = millis();
  // if (nowMs - lastDebugMs > 500) {  // Print debug info every 500ms
  //   Serial.print("CMD:"); Serial.print(setW1); Serial.print(","); Serial.println(setW2);
  //   Serial.print("ACT:"); Serial.print(filtW1); Serial.print(","); Serial.println(filtW2);
  //   Serial.print("PWM:"); Serial.print(p1); Serial.print(","); Serial.println(p2);
  //   Serial.print("ENC:"); Serial.print(cnt1); Serial.print(","); Serial.println(cnt2);
  //   Serial.print("Status: E-Stop="); Serial.print(emergencyStop ? "YES" : "NO");
  //   Serial.print(" Enc-Err="); Serial.println(encoderError ? "YES" : "NO");
  //   lastDebugMs = nowMs;
  // }

  // 10) Odometry update & transmit
  int64_t dEnc1 = totalEnc1 - lastTotalEnc1;
  int64_t dEnc2 = totalEnc2 - lastTotalEnc2;
  float dAngL = dEnc1 * RAD_PER_TICK;
  float dAngR = dEnc2 * RAD_PER_TICK;
  float dLeft  = dAngL * RADIUS;
  float dRight = dAngR * RADIUS;
  float dC     = (dLeft + dRight)/2;
  float dTh    = (dLeft - dRight)/BASE;

  if (fabs(dTh) < 1e-4) {
    x     += dC * cos(theta);
    y     += dC * sin(theta);
  } else {
    float r = dC/dTh;
    x     += r*(sin(theta+dTh)-sin(theta));
    y     += r*(-cos(theta+dTh)+cos(theta));
  }
  theta += dTh;
  if (theta > PI)  theta -= 2*PI;
  if (theta < -PI) theta += 2*PI;

  if (nowMs - lastOdomMs >= ODOM_MS) {
    float dt_odom = (nowMs - lastOdomMs)/1000.0f;
    vx    = dC/dt_odom;
    vy    = 0;
    omega = dTh/dt_odom;
    lastTotalEnc1 = totalEnc1;
    lastTotalEnc2 = totalEnc2;
    lastOdomMs    = nowMs;
    sendOdometry();
  }

  if (testModeActive) {
    outputPlotData(p1, p2);
  }
}