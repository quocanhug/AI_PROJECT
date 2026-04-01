#include "do_line.h"
#include <Arduino.h>

// ================= Extern từ main.ino (dùng cho MODE_DELIVERY & MODE_AI_ROUTE)
// =================
extern int currentPath[15];
extern int pathLength;
extern int currentPathIndex;
extern int currentDir;
extern volatile int delivered_count;
extern bool line_mode;
extern bool is_auto_running;
extern volatile UIMode currentMode;
extern void gripOpen();
extern void gripClose();

// ================= Tọa độ node trên bản đồ (dùng tính hướng rẽ)
// ================= Grid thuc te: 4x2 o (5 cot x 3 hang) = 15 node (0-14)
// Moi o: 25cm rong x 35cm cao, xe 22cm
const int NODE_COUNT = 15;
const int node_coords[15][2] = {{30, 30},   {100, 30},  {170, 30},  {240, 30},
                                {310, 30},  {30, 100},  {100, 100}, {170, 100},
                                {240, 100}, {310, 100}, {30, 170},  {100, 170},
                                {170, 170}, {240, 170}, {310, 170}};

int getTargetDirection(int nodeA, int nodeB) {
  // ★ FIX C3: Validate node index to prevent OOB access
  if (nodeA < 0 || nodeA >= NODE_COUNT || nodeB < 0 || nodeB >= NODE_COUNT) return -1;
  int dx = node_coords[nodeB][0] - node_coords[nodeA][0];
  int dy = node_coords[nodeB][1] - node_coords[nodeA][1];
  if (dy > 0)
    return 0; // down
  if (dx > 0)
    return 1; // right
  if (dy < 0)
    return 2; // up
  if (dx < 0)
    return 3; // left
  return -1;
}

// ================= Motor pins (ESP32 + L298N) =================
#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

// ================= Line sensors =================
#define L2_SENSOR 34
#define L1_SENSOR 32
#define M_SENSOR 33
#define R1_SENSOR 27
#define R2_SENSOR 25

// ================= Encoders =================
#define ENC_L 26
#define ENC_R 22
#define PULSES_PER_REV 20
#define PPR_EFFECTIVE (PULSES_PER_REV * 3)
#define MIN_EDGE_US 1500

// ================= HC-SR04 =================
#define TRIG_PIN 21
#define ECHO_PIN 19

const float OBSTACLE_TH_CM = 25.0f;
const unsigned long US_TIMEOUT = 3000;

static unsigned long us_last_ms = 0;
float us_dist_cm = 999.0f;

static uint8_t obs_hit = 0;
static bool obs_latched = false;

const float OBSTACLE_ON_CM = OBSTACLE_TH_CM;
const float OBSTACLE_OFF_CM = 30.0f;
const uint8_t OBS_HIT_N = 3;
const unsigned long US_PERIOD_MS = 25;

// ================= Thong so co khi =================
const float WHEEL_RADIUS_M = 0.0325f;
const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M;
const float TRACK_WIDTH_M = 0.1150f;

// ================= Tham so dieu khien =================
float v_base = 0.4f;
float v_boost = 0.15f;
float v_hard = 0.20f;
float v_search = 0.2f;
// ★ FIX M6: Removed dead variable vF (was never used)

// PID
PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

const unsigned long CTRL_DT_MS = 10;
volatile long encL_count = 0, encR_count = 0, encL_total = 0, encR_total = 0;
volatile uint32_t encL_last_us = 0, encR_last_us = 0;
float vL_ema = 0.0f, vR_ema = 0.0f;
const float EMA_B = 0.7f;

const int PWM_MIN_RUN = 75;
const int PWM_SLEW = 15;
static int pwmL_prev = 0, pwmR_prev = 0;

static unsigned long t_prev = 0;
static unsigned long bad_t = 0;

enum Side { NONE, LEFT, RIGHT };
Side last_seen = NONE;
bool seen_line_ever = false;
bool avoiding = false;
static volatile bool g_line_enabled = true;
bool recovering = false;
unsigned long rec_t0 = 0;
const unsigned long RECOV_TIME_MS = 3000; // 3s timeout (luoi nho 35x25cm)

// Recovery: LUI + QUET NHE trai/phai, KHONG quay dau
int recov_sweep_count = 0;
const int RECOV_MAX_SWEEPS = 8; // 8 sweep (trai-phai luan phien)
bool recov_did_backup = false;   // da lui lan 1 chua
bool recov_did_second_backup = false; // da lui lan 2 chua

int lastConfirmedNodeIdx = 0;

const unsigned long INTERSECTION_DEBOUNCE_MS = 500;
unsigned long last_intersection_time = 0;

bool needs_initial_turn = false;

// ================= ISR encoder =================
void IRAM_ATTR encL_isr() {
  uint32_t now = micros();
  if (now - encL_last_us >= MIN_EDGE_US) {
    encL_count++;
    encL_total++;
    encL_last_us = now;
  }
}
void IRAM_ATTR encR_isr() {
  uint32_t now = micros();
  if (now - encR_last_us >= MIN_EDGE_US) {
    encR_count++;
    encR_total++;
    encR_last_us = now;
  }
}

// ================= Utils =================
inline int clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
// ★ FIX M4: Signed clamp [-255, 255] for PWM that can be negative (recovery sweep)
inline int clampSigned255(int v) { return v < -255 ? -255 : (v > 255 ? 255 : v); }
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline bool onLine(int pin) { return digitalRead(pin) == LOW; }

// ★ FIX M4: shape_pwm now preserves sign for negative PWM (recovery sweep)
static inline int shape_pwm(int target, int prev) {
  int s = target;
  if (s > 0 && s < PWM_MIN_RUN)
    s = PWM_MIN_RUN;
  if (s < 0 && s > -PWM_MIN_RUN)
    s = -PWM_MIN_RUN;
  int d = s - prev;
  if (d > PWM_SLEW)
    s = prev + PWM_SLEW;
  if (d < -PWM_SLEW)
    s = prev - PWM_SLEW;
  return clampSigned255(s);
}

// ================= Motor control =================
void driveWheelRight(float v_target, int pwm) {
  int d = clamp255(abs(pwm));
  if (v_target >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, d);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, d);
  }
}

void driveWheelLeft(float v_target, int pwm) {
  int d = clamp255(abs(pwm));
  if (v_target >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, d);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, d);
  }
}

void motorsStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

float ticksToVel(long ticks, float dt_s) {
  return ((float)ticks / (float)PPR_EFFECTIVE * CIRC) / dt_s;
}

// ★ FIX M4: pidStep now returns signed value for bidirectional control
int pidStep(PID &pid, float v_target, float v_meas, float dt_s) {
  if (dt_s < 0.001f)
    dt_s = 0.001f;
  float err = v_target - v_meas;
  pid.i_term += pid.Ki * err * dt_s;
  pid.i_term = clampf(pid.i_term, -pid.out_max, pid.out_max);
  float d = (err - pid.prev_err) / dt_s;
  float u = pid.Kp * err + pid.i_term + pid.Kd * d;
  pid.prev_err = err;
  return clampSigned255((int)u);
}

// ================= HC-SR04 =================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0)
    return -1;
  return dur * 0.0343f / 2.0f;
}

// ★ FIX M1: Median-of-3 filter for HC-SR04 noise reduction
static float readDistanceCM_filtered() {
  float samples[3];
  for (int i = 0; i < 3; i++) {
    samples[i] = readDistanceCM();
    if (i < 2) delayMicroseconds(500);
  }
  // Sort 3 values to get median
  if (samples[0] > samples[1]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  if (samples[1] > samples[2]) { float t = samples[1]; samples[1] = samples[2]; samples[2] = t; }
  if (samples[0] > samples[1]) { float t = samples[0]; samples[0] = samples[1]; samples[1] = t; }
  float d = samples[1]; // median
  if (d < 2.0f || d > 200.0f)
    d = 999.0f;
  return d;
}

static inline void updateObstacleState() {
  if (millis() - us_last_ms < US_PERIOD_MS)
    return;
  us_last_ms = millis();
  us_dist_cm = readDistanceCM_filtered();
  if (!obs_latched) {
    if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM) {
      if (++obs_hit >= OBS_HIT_N)
        obs_latched = true;
    } else {
      obs_hit = 0;
    }
  } else {
    if (us_dist_cm >= OBSTACLE_OFF_CM) {
      obs_latched = false;
      obs_hit = 0;
    }
  }
}

// ================= Hinh hoc encoder =================
long countsForDistance(double dist_m) {
  return (long)(dist_m / CIRC * PPR_EFFECTIVE + 0.5);
}

inline void motorWriteLR_signed(int pwmL, int pwmR) {
  pwmL = pwmL < -255 ? -255 : (pwmL > 255 ? 255 : pwmL);
  pwmR = pwmR < -255 ? -255 : (pwmR > 255 ? 255 : pwmR);
  if (pwmR >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, pwmR);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -pwmR);
  }
  if (pwmL >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, pwmL);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -pwmL);
  }
}

static inline double theta_from_counts(long dL, long dR, int signL, int signR) {
  return ((double)dR / PPR_EFFECTIVE * CIRC * signR -
          (double)dL / PPR_EFFECTIVE * CIRC * signL) /
         TRACK_WIDTH_M;
}

// ================= Quay theo goc =================
bool spin_left_deg(double deg, int pwmMax) {
  const double target = 1.5 * deg * 3.141592653589793 / 180.0;
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();
  while (true) {
    if (!g_line_enabled) {
      motorsStop();
      return false;
    }
    long L, R;
    noInterrupts();
    L = encL_total;
    R = encR_total;
    interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, -1, +1);
    double err = target - theta;
    if (fabs(err) <= deg_tol)
      break;
    if (millis() - t0 > T_FAIL_MS) {
      success = false;
      Serial.println("[SPIN] LEFT TIMEOUT!");
      break;
    }
    int pwmCap = (fabs(err) < 15.0 * 3.141592653589793 / 180.0) ? (pwmMax * 0.5)
                                                                : pwmMax;
    int base = (int)clamp255((int)(fabs(err) * Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);
    int corr = (int)(Kbal * ((double)dR - (double)dL));
    int pwmL = base - corr;
    if (pwmL < pwmMin)
      pwmL = pwmMin;
    int pwmR = base + corr;
    if (pwmR < pwmMin)
      pwmR = pwmMin;
    motorWriteLR_signed(-pwmL, +pwmR);
    delay(2);
  }
  motorsStop();
  return success;
}

bool spin_right_deg(double deg, int pwmMax) {
  const double target = -1.5 * (deg * 3.141592653589793 / 180.0);
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  long L0, R0;
  noInterrupts();
  L0 = encL_total;
  R0 = encR_total;
  interrupts();
  while (true) {
    if (!g_line_enabled) {
      motorsStop();
      return false;
    }
    long L, R;
    noInterrupts();
    L = encL_total;
    R = encR_total;
    interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol)
      break;
    if (millis() - t0 > T_FAIL_MS) {
      success = false;
      Serial.println("[SPIN] RIGHT TIMEOUT!");
      break;
    }
    int pwmCap = (fabs(err) < 15.0 * 3.141592653589793 / 180.0) ? (pwmMax * 0.5)
                                                                : pwmMax;
    int base = (int)clamp255((int)(fabs(err) * Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);
    int corr = (int)(Kbal * ((double)dL - (double)dR));
    int pwmL = base + corr;
    if (pwmL < pwmMin)
      pwmL = pwmMin;
    int pwmR = base - corr;
    if (pwmR < pwmMin)
      pwmR = pwmMin;
    motorWriteLR_signed(+pwmL, -pwmR);
    delay(2);
  }
  motorsStop();
  return success;
}

// ================= Tien theo quang duong =================
// ★ FIX M3: Added 5s timeout to prevent infinite blocking if encoder fails
void move_forward_distance(double dist_m, int pwmAbs) {
  long target = countsForDistance(dist_m);
  long sL, sR;
  noInterrupts();
  sL = encL_total;
  sR = encR_total;
  interrupts();
  unsigned long t0 = millis();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true) {
    if (!g_line_enabled) {
      motorsStop();
      return;
    }
    if (millis() - t0 > 5000) {
      Serial.println("[FWD] TIMEOUT!");
      break;
    }
    long cL, cR;
    noInterrupts();
    cL = encL_total;
    cR = encR_total;
    interrupts();
    bool left_done = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done)
      break;
    if (left_done && !right_done)
      motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done)
      motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
}

// ★ FIX M3: Added 5s timeout
bool move_forward_distance_until_line(double dist_m, int pwmAbs) {
  long target = countsForDistance(dist_m);
  long sL, sR;
  noInterrupts();
  sL = encL_total;
  sR = encR_total;
  interrupts();
  unsigned long t0 = millis();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true) {
    if (!g_line_enabled) {
      motorsStop();
      return false;
    }
    if (millis() - t0 > 5000) {
      Serial.println("[FWD_LINE] TIMEOUT!");
      break;
    }
    long cL, cR;
    noInterrupts();
    cL = encL_total;
    cR = encR_total;
    interrupts();
    // Kiem tra CA 5 mat cam bien (L2, L1, M, R1, R2)
    if (digitalRead(M_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
        digitalRead(R1_SENSOR) == LOW || digitalRead(L2_SENSOR) == LOW ||
        digitalRead(R2_SENSOR) == LOW) {
      motorsStop();
      return true;
    }
    bool left_done = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done)
      break;
    if (left_done && !right_done)
      motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done)
      motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
  return false;
}

// ★ FIX M7: Reset avoiding/obs state to prevent stale flags blocking next run
void do_line_abort() {
  g_line_enabled = false;
  motorsStop();
  avoiding = false;
  obs_latched = false;
  obs_hit = 0;
  recovering = false;
}

void do_line_resume() {
  g_line_enabled = true;
  seen_line_ever = true;
  recovering = false;
  avoiding = false;
  recov_did_backup = false;
  recov_did_second_backup = false;
  recov_sweep_count = 0;
  obs_latched = false;
  obs_hit = 0;

  vL_ema = 0.0f;
  vR_ema = 0.0f;
  pwmL_prev = 0;
  pwmR_prev = 0;
  pidL.i_term = 0;
  pidL.prev_err = 0;
  pidR.i_term = 0;
  pidR.prev_err = 0;

  t_prev = millis();
  bad_t = millis();

  currentPathIndex = lastConfirmedNodeIdx;
  needs_initial_turn = true;
}

void avoidObstacle() {
  const int TURN_PWM = 150;
  const int FWD_PWM = 100;
  spin_left_deg(40.0, TURN_PWM);
  motorsStop();
  delay(500);
  move_forward_distance(0.2, FWD_PWM);
  motorsStop();
  delay(500);
  spin_right_deg(40.0, TURN_PWM);
  motorsStop();
  delay(500);
  move_forward_distance(0.15, FWD_PWM);
  motorsStop();
  delay(500);
  spin_right_deg(50.0, TURN_PWM);
  motorsStop();
  delay(500);
  move_forward_distance_until_line(0.6, FWD_PWM);
  motorsStop();
  delay(500);
  spin_left_deg(15.0, TURN_PWM);
  motorsStop();
  delay(500);
}

// ================= Setup =================
void do_line_setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(L2_SENSOR, INPUT);
  pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR, INPUT);
  pinMode(R1_SENSOR, INPUT);
  pinMode(R2_SENSOR, INPUT);
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  detachInterrupt(digitalPinToInterrupt(ENC_L));
  detachInterrupt(digitalPinToInterrupt(ENC_R));
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  g_line_enabled = true;
  seen_line_ever = false;
  vL_ema = 0.0f;
  vR_ema = 0.0f;
  pwmL_prev = 0;
  pwmR_prev = 0;
  us_last_ms = millis();
  us_dist_cm = 999.0f;
  obs_hit = 0;
  obs_latched = false;
  pidL.i_term = 0;
  pidL.prev_err = 0;
  pidR.i_term = 0;
  pidR.prev_err = 0;
  last_intersection_time = 0;
  last_seen = NONE;
  recovering = false;
  recov_sweep_count = 0;
  recov_did_backup = false;
  recov_did_second_backup = false;
  lastConfirmedNodeIdx = 0;
  needs_initial_turn = true;

  t_prev = millis();
  bad_t = millis();

  motorsStop();
}

// ================= Main Loop =================
void do_line_loop() {
  if (!g_line_enabled) {
    motorsStop();
    return;
  }

  // ===== INITIAL TURN: Xoay xe chuan huong khi nhan route moi =====
  if (needs_initial_turn) {
    needs_initial_turn = false;

    if ((currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) &&
        pathLength >= 2 && currentPathIndex < pathLength - 1) {

      int curNode = currentPath[currentPathIndex];
      int nxtNode = currentPath[currentPathIndex + 1];
      int targetDir = getTargetDirection(curNode, nxtNode);

      Serial.printf("[INIT_TURN] node%d->node%d, dir %d->%d\n", curNode,
                    nxtNode, currentDir, targetDir);

      if (targetDir != -1 && targetDir != currentDir) {
        int diff = (targetDir - currentDir + 4) % 4;
        const int TURN_PWM = 160;
        bool turnOk = true;

        motorsStop();
        delay(300);

        if (diff == 1) {
          turnOk = spin_left_deg(60.0, TURN_PWM);  // 60*1.5=90 do chinh xac
          Serial.println("  >> INIT LEFT");
        } else if (diff == 3) {
          turnOk = spin_right_deg(60.0, TURN_PWM); // 60*1.5=90 do chinh xac
          Serial.println("  >> INIT RIGHT");
        } else if (diff == 2) {
          turnOk = spin_right_deg(120.0, TURN_PWM); // 120*1.5=180 do chinh xac
          Serial.println("  >> INIT U-TURN");
        }

        motorsStop();
        delay(200);

        if (!turnOk) {
          Serial.println("  INIT TURN FAILED");
          motorsStop();
          extern void wsBroadcast(const char *);
          String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                       "\"robotNode\":" +
                       String(curNode) +
                       ","
                       "\"robotDir\":" +
                       String(currentDir) +
                       ","
                       "\"reason\":\"TURN_FAILED\"}";
          wsBroadcast(msg.c_str());
          currentMode = MODE_MANUAL;
          is_auto_running = false;
          line_mode = false;
          do_line_abort();
          return;
        }

        // Tim line sau khi quay - tien 8cm (luoi nho 25cm, tranh overshoot)
        bool lineFoundAfterTurn = move_forward_distance_until_line(0.08, 90);
        if (!lineFoundAfterTurn) {
          Serial.println("  INIT: line not found after fwd, trying sweep");
          // Quet trai/phai tim line (giong intersection turn)
          bool foundWide = false;
          for (int sweep = 0; sweep < 3 && !foundWide; sweep++) {
            // Quet trai
            motorWriteLR_signed(-90, 90);
            unsigned long ts = millis();
            while (millis() - ts < 250) {
              if (!g_line_enabled) { motorsStop(); return; }
              if (digitalRead(L2_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
                  digitalRead(M_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW ||
                  digitalRead(R2_SENSOR) == LOW) {
                motorsStop(); foundWide = true; break;
              }
              delay(5);
            }
            if (foundWide) break;
            motorsStop(); delay(50);
            // Quet phai
            motorWriteLR_signed(90, -90);
            ts = millis();
            while (millis() - ts < 250) {
              if (!g_line_enabled) { motorsStop(); return; }
              if (digitalRead(L2_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
                  digitalRead(M_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW ||
                  digitalRead(R2_SENSOR) == LOW) {
                motorsStop(); foundWide = true; break;
              }
              delay(5);
            }
            motorsStop(); delay(50);
          }
          lineFoundAfterTurn = foundWide;
          if (!foundWide) {
            Serial.println("  INIT: sweep failed too!");
          }
        }

        currentDir = targetDir;
        if (lineFoundAfterTurn)
          currentPathIndex++;
        last_intersection_time = millis();
        seen_line_ever = true;
        return;
      }

      // STRAIGHT case: targetDir == currentDir, khong can xoay
      currentPathIndex++;
      last_intersection_time = millis();
      seen_line_ever = true;
      return;
    }
  } // ===== END INITIAL TURN =====

  // ===== Doc cam bien =====
  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2)
    seen_line_ever = true;

  // ===== Mat line hoan toan -> recovery =====
  bool lost_all = !L2 && !L1 && !M && !R1 && !R2;
  if (lost_all) {
    if (!seen_line_ever && is_auto_running) {
      bad_t = millis(); // Dang bo tim line, reset timeout
    } else if (!recovering && seen_line_ever && millis() - bad_t > 150) {
      // Mat line >150ms -> vao recovery (luoi nho, phan ung nhanh)
      Serial.println("[LINE] Lost >150ms -> entering RECOVERY");
      recovering = true;
      rec_t0 = millis();
      recov_sweep_count = 0;
      recov_did_backup = false;
      recov_did_second_backup = false;
      motorsStop();
      return;
    }
  } else {
    bad_t = millis();
    if (recovering) {
      recovering = false;
      Serial.println("[LINE] Recovery SUCCESS - line found!");
    }
  }

  updateObstacleState();

  // ===== AI Route: vat can -> dung, bao Web =====
  if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) && obs_latched && !avoiding) {
    motorsStop();
    obs_latched = false;
    obs_hit = 0;  // ★ FIX L1: Reset hit counter to prevent immediate re-latch
    int robotNode = (lastConfirmedNodeIdx < pathLength)
                        ? currentPath[lastConfirmedNodeIdx]
                        : 0;
    int obstacleNode = (currentPathIndex < pathLength)
                           ? currentPath[currentPathIndex]
                           : robotNode;
    extern void wsBroadcast(const char *);
    String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                 "\"robotNode\":" +
                 String(robotNode) +
                 ","
                 "\"obstacleNode\":" +
                 String(obstacleNode) +
                 ","
                 "\"robotDir\":" +
                 String(currentDir) +
                 ","
                 "\"current_step\":" +
                 String(lastConfirmedNodeIdx) +
                 ","
                 "\"distance_cm\":" +
                 String(us_dist_cm, 1) + "}";
    wsBroadcast(msg.c_str());
    Serial.printf("[AI_ROUTE] OBSTACLE! robot@node%d blocked@node%d\n",
                  robotNode, obstacleNode);
    currentMode = MODE_MANUAL;
    is_auto_running = false;
    line_mode = false;
    do_line_abort();
    return;
  }

  // ===== Cac mode khac: tranh vat can vat ly =====
  if (obs_latched && !avoiding && currentMode != MODE_AI_ROUTE) {
    avoiding = true;
    motorsStop();
    noInterrupts();
    encL_count = 0;
    encR_count = 0;
    interrupts();
    t_prev = millis();
    avoidObstacle();
    motorsStop();
    noInterrupts();
    encL_count = 0;
    encR_count = 0;
    interrupts();
    t_prev = millis();
    avoiding = false;
    return;
  }
  if (avoiding)
    return;

  float vL_tgt = 0, vR_tgt = 0;

  // ============================================================
  // GIAO LO: Ca L2 VA R2 deu thay vach ngang
  // ============================================================
  bool at_intersection = (L2 && R2 && (L1 || M || R1));

  if (at_intersection) {

    // --- MODE_LINE_ONLY: di thang qua giao lo ---
    if (currentMode == MODE_LINE_ONLY) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();
        motorsStop();
        delay(200);
        move_forward_distance(0.06, 120);
        if (!onLine(M_SENSOR) && !onLine(L1_SENSOR) && !onLine(R1_SENSOR)) {
          motorWriteLR_signed(130, -130);
          unsigned long t0 = millis();
          while (millis() - t0 < 3500) {
            if (!g_line_enabled) { motorsStop(); return; }  // ★ FIX L3: ESTOP check
            if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR))
              break;
            delay(5);
          }
          motorsStop();
        }
        last_seen = NONE;
        recovering = false;
        return;
      }
      vL_tgt = v_base * 0.8f;
      vR_tgt = v_base * 0.8f;
    }

    // --- MODE_DELIVERY / MODE_AI_ROUTE: xu ly node ---
    else if (currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();

        if (pathLength > 0) {
          // Kiem tra da den dich chua
          if (currentPathIndex >= pathLength - 1) {
            move_forward_distance(0.02, 90);
            motorsStop();
            delay(200);
            Serial.printf("[AI_ROUTE] ARRIVED at node %d\n",
                          currentPath[pathLength - 1]);

            if (currentMode == MODE_DELIVERY) {
              gripOpen();
              delay(1500);
              gripClose();
            } else {
              Serial.println("[AI_ROUTE] DESTINATION REACHED - COMPLETED");
              extern void wsBroadcast(const char *);
              String msg = "{\"type\":\"COMPLETED\",\"robotNode\":" +
                           String(currentPath[pathLength - 1]) +
                           ",\"robotDir\":" + String(currentDir) + "}";
              wsBroadcast(msg.c_str());
            }
            delivered_count++;
            currentMode = MODE_MANUAL;
            is_auto_running = false;
            line_mode = false;
            do_line_abort();
            return;
          }

          // Tinh huong re
          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          int diff = (targetDir != -1) ? (targetDir - currentDir + 4) % 4 : 0;
          Serial.printf(
              "[NODE] path[%d]=node%d -> node%d, dir %d->%d (diff=%d)\n",
              currentPathIndex, curNode, nxtNode, currentDir, targetDir, diff);

          if (diff == 0) {
            // DI THANG
            lastConfirmedNodeIdx = currentPathIndex;
            currentPathIndex++;
            currentDir = targetDir;
            move_forward_distance(0.03, 100);
            last_seen = NONE;
          } else {
            // RE / U-TURN
            move_forward_distance(0.03, 100);
            motorsStop();
            delay(300);

            // ★ FIX M5: Reset PID i_term before turn to prevent motor jerk
            pidL.i_term = 0; pidL.prev_err = 0;
            pidR.i_term = 0; pidR.prev_err = 0;

            const int TURN_PWM = 160;
            bool turnOk = true;
            if (diff == 1) {
              turnOk = spin_left_deg(60.0, TURN_PWM);  // 60*1.5=90 do
              Serial.println("  >> LEFT");
            } else if (diff == 3) {
              turnOk = spin_right_deg(60.0, TURN_PWM); // 60*1.5=90 do
              Serial.println("  >> RIGHT");
            } else if (diff == 2) {
              turnOk = spin_right_deg(120.0, TURN_PWM); // 120*1.5=180 do
              Serial.println("  >> U-TURN");
            }

            motorsStop();
            delay(200);

            if (!turnOk) {
              Serial.println("  TURN FAILED");
              motorsStop();
              extern void wsBroadcast(const char *);
              int robotNode = currentPath[currentPathIndex];
              String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                           "\"robotNode\":" +
                           String(robotNode) +
                           ","
                           "\"robotDir\":" +
                           String(currentDir) +
                           ","
                           "\"reason\":\"TURN_FAILED\"}";
              wsBroadcast(msg.c_str());
              currentMode = MODE_MANUAL;
              is_auto_running = false;
              line_mode = false;
              do_line_abort();
              return;
            }

            currentDir = targetDir;
            lastConfirmedNodeIdx = currentPathIndex;
            bool lineFound = move_forward_distance_until_line(0.08, 90);
            currentPathIndex++;
            if (!lineFound) {
              Serial.println("  Line not found after turn - wide scan");
              bool foundWide = false;
              for (int sweep = 0; sweep < 2 && !foundWide; sweep++) {
                motorWriteLR_signed(-90, 90);
                unsigned long ts = millis();
                while (millis() - ts < 200) {
                  if (!g_line_enabled) {
                    motorsStop();
                    return;
                  }
                  if (digitalRead(L2_SENSOR) == LOW ||
                      digitalRead(L1_SENSOR) == LOW ||
                      digitalRead(M_SENSOR) == LOW ||
                      digitalRead(R1_SENSOR) == LOW ||
                      digitalRead(R2_SENSOR) == LOW) {
                    motorsStop();
                    foundWide = true;
                    break;
                  }
                  delay(5);
                }
                if (foundWide)
                  break;
                motorsStop();
                delay(50);
                motorWriteLR_signed(90, -90);
                ts = millis();
                while (millis() - ts < 200) {
                  if (!g_line_enabled) {
                    motorsStop();
                    return;
                  }
                  if (digitalRead(L2_SENSOR) == LOW ||
                      digitalRead(L1_SENSOR) == LOW ||
                      digitalRead(M_SENSOR) == LOW ||
                      digitalRead(R1_SENSOR) == LOW ||
                      digitalRead(R2_SENSOR) == LOW) {
                    motorsStop();
                    foundWide = true;
                    break;
                  }
                  delay(5);
                }
                motorsStop();
                delay(50);
              }
              if (!foundWide) {
                recovering = true;
                rec_t0 = millis();
                recov_sweep_count = 0;
                recov_did_backup = false;
              }
            }
          }
        }
        return;
      }
      last_seen = NONE;
      vL_tgt = v_base * 0.8f;
      vR_tgt = v_base * 0.8f;
    }

    // --- Cac mode khac ---
    else {
      vL_tgt = v_base * 0.8f;
      vR_tgt = v_base * 0.8f;
    }
  }

  // ============================================================
  // RECOVERY: LUI NHE + QUET NHE TRAI/PHAI de tim lai line
  // Luoi nho 35x25cm -> lui ngan, quet nhe, TUYET DOI KHONG quay dau
  // ============================================================
  else if (recovering) {
    // Tim thay line -> thoat recovery
    if (L2 || L1 || M || R1 || R2) {
      recovering = false;
      motorsStop();
      Serial.println("[RECOV] Line found during recovery!");
      bad_t = millis();
      vL_ema = 0;
      vR_ema = 0;
      pwmL_prev = 0;
      pwmR_prev = 0;
      pidL.i_term = 0;
      pidL.prev_err = 0;
      pidR.i_term = 0;
      pidR.prev_err = 0;
      noInterrupts();
      encL_count = 0;
      encR_count = 0;
      interrupts();
      t_prev = millis();
      return;
    }

    // Timeout recovery -> dung han, bao Web (KHONG dao huong)
    if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false;
      motorsStop();
      Serial.println("[RECOV] TIMEOUT - recovery failed");

      if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) &&
          is_auto_running) {
        currentPathIndex = lastConfirmedNodeIdx;
        extern void wsBroadcast(const char *);
        int robotNode = currentPath[lastConfirmedNodeIdx];
        // GIU NGUYEN huong hien tai, KHONG dao 180 do
        String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                     "\"robotNode\":" +
                     String(robotNode) +
                     ","
                     "\"robotDir\":" +
                     String(currentDir) +
                     ","
                     "\"reason\":\"LINE_LOST\"}";
        wsBroadcast(msg.c_str());
        currentMode = MODE_MANUAL;
        is_auto_running = false;
        line_mode = false;
        do_line_abort();
        return;
      }
      noInterrupts();
      encL_count = 0;
      encR_count = 0;
      interrupts();
      t_prev = millis();
      return;
    }

    // ===== RECOVERY: LUI TRUOC, ROI QUET NHE TRAI/PHAI =====
    // Buoc 1: Lui nhe 2cm (tim line phia sau)
    // Buoc 2: Quet nhe trai/phai (goc nho, toc do thap)
    // Buoc 3: Lui them 2cm (sau 4 sweep dau)
    // Buoc 4: Quet tiep, rong hon mot chut
    // KHONG BAO GIO quay dau (U-turn)
    {
      // === BUOC 1: Lui nhe 2cm (lan dau) ===
      if (!recov_did_backup) {
        recov_did_backup = true;
        Serial.println("[RECOV] Buoc 1: Lui 2cm");
        const int REV_PWM = 70;
        const long revTarget = countsForDistance(0.02);
        long sL, sR;
        noInterrupts();
        sL = encL_total;
        sR = encR_total;
        interrupts();
        motorWriteLR_signed(-REV_PWM, -REV_PWM);
        unsigned long bk = millis();
        while (millis() - bk < 500) {
          if (!g_line_enabled) { motorsStop(); return; }
          if (digitalRead(L2_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
              digitalRead(M_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW ||
              digitalRead(R2_SENSOR) == LOW) {
            motorsStop();
            recovering = false;
            Serial.println("[RECOV] Line found while backing!");
            bad_t = millis();
            t_prev = millis();
            return;
          }
          long cL, cR;
          noInterrupts();
          cL = encL_total;
          cR = encR_total;
          interrupts();
          if (labs(cL - sL) >= revTarget && labs(cR - sR) >= revTarget) break;
          delay(5);
        }
        motorsStop();
        delay(100);
        rec_t0 = millis();     // Reset timer cho sweep phase
        recov_sweep_count = 0;
        return;
      }

      // === BUOC 3: Lui them 2cm (sau 4 sweep dau) ===
      if (recov_sweep_count >= 4 && !recov_did_second_backup) {
        recov_did_second_backup = true;
        Serial.println("[RECOV] Buoc 3: Lui them 2cm");
        const int REV_PWM = 70;
        const long revTarget = countsForDistance(0.02);
        long sL, sR;
        noInterrupts();
        sL = encL_total;
        sR = encR_total;
        interrupts();
        motorWriteLR_signed(-REV_PWM, -REV_PWM);
        unsigned long bk = millis();
        while (millis() - bk < 500) {
          if (!g_line_enabled) { motorsStop(); return; }
          if (digitalRead(L2_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
              digitalRead(M_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW ||
              digitalRead(R2_SENSOR) == LOW) {
            motorsStop();
            recovering = false;
            Serial.println("[RECOV] Line found while backing (2)!");
            bad_t = millis();
            t_prev = millis();
            return;
          }
          long cL, cR;
          noInterrupts();
          cL = encL_total;
          cR = encR_total;
          interrupts();
          if (labs(cL - sL) >= revTarget && labs(cR - sR) >= revTarget) break;
          delay(5);
        }
        motorsStop();
        delay(100);
        rec_t0 = millis();     // Reset timer cho sweep tiep
        recov_sweep_count = 0; // Sweep lai tu dau
        return;
      }

      // === BUOC 2 & 4: Quet nhe trai/phai ===
      // Uu tien huong last_seen (huong cuoi cung thay line)
      // Toc do thap, goc nho -> KHONG quay dau
      unsigned long elapsed = millis() - rec_t0;

      bool sweepToLeft;
      if (last_seen == LEFT) {
        sweepToLeft = (recov_sweep_count % 2 == 0); // Trai truoc
      } else if (last_seen == RIGHT) {
        sweepToLeft = (recov_sweep_count % 2 != 0); // Phai truoc
      } else {
        sweepToLeft = (recov_sweep_count % 2 == 0);
      }

      // Quet NHE: toc do thap (0.3 * v_base), tranh quay qua manh
      const float SWEEP_V = v_base * 0.3f;
      if (sweepToLeft) {
        vL_tgt = -SWEEP_V;
        vR_tgt = SWEEP_V;
      } else {
        vL_tgt = SWEEP_V;
        vR_tgt = -SWEEP_V;
      }

      // Chuyen sweep tiep theo (duration tang dan)
      unsigned long total_sweep_time = 0;
      for (int i = 0; i <= recov_sweep_count && i < RECOV_MAX_SWEEPS; i++) {
        total_sweep_time += 180 + i * 50; // 180ms, 230ms, 280ms, ...
      }
      if (elapsed > total_sweep_time) {
        recov_sweep_count++;
        if (recov_sweep_count >= RECOV_MAX_SWEEPS) {
          Serial.println("[RECOV] All sweeps exhausted, waiting timeout");
        } else {
          Serial.printf("[RECOV] Sweep #%d (%s)\n", recov_sweep_count,
                        sweepToLeft ? "LEFT" : "RIGHT");
        }
      }
    }
  }

  // ============================================================
  // DO LINE: PID dung 3 mat giua L1, M, R1
  // ============================================================
  else {
    // Muc 1: Thang hoan toan
    if (M && !L1 && !R1) {
      last_seen = NONE;
      vL_tgt = v_base;
      vR_tgt = v_base;
    }
    // Muc 2: Lech nhe
    else if (M && L1 && !R1) {
      last_seen = LEFT;
      vL_tgt = v_base - v_boost;
      vR_tgt = v_base + v_boost;
    } else if (M && R1 && !L1) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_boost;
      vR_tgt = v_base - v_boost;
    }
    // Muc 3: Lech vua
    else if (L1 && !M) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard;
      vR_tgt = v_base + v_hard;
    } else if (R1 && !M) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard;
      vR_tgt = v_base - v_hard;
    }
    // Muc 4: Lech manh (chi L2 hoac R2)
    else if (L2 && !L1 && !M && !R1 && !R2) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard;
      vR_tgt = v_base + v_hard;
    } else if (R2 && !L1 && !M && !R1 && !L2) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard;
      vR_tgt = v_base - v_hard;
    }
    // Muc 5: Mat HOAN TOAN (ca 5 mat tat)
    else if (!L2 && !L1 && !M && !R1 && !R2) {
      if (!seen_line_ever && is_auto_running) {
        vL_tgt = v_search;
        vR_tgt = v_search;
      } else if (!seen_line_ever) {
        vL_tgt = 0;
        vR_tgt = 0;
      } else {
        // Mat line -> DUNG NGAY, cho recovery xu ly
        // KHONG bo tien de tranh lech xa hon tren luoi nho 35x25cm
        vL_tgt = 0;
        vR_tgt = 0;
      }
    } else {
      vL_tgt = v_base;
      vR_tgt = v_base;
    }
  }

  // ================= PID =================
  unsigned long now = millis();
  if (now - t_prev >= CTRL_DT_MS) {
    float dt_s = (now - t_prev) / 1000.0f;
    t_prev = now;
    noInterrupts();
    long cL = encL_count;
    encL_count = 0;
    long cR = encR_count;
    encR_count = 0;
    interrupts();
    float vL_meas_inst = ticksToVel(cL, dt_s) * (vL_tgt >= 0 ? 1.0f : -1.0f);
    float vR_meas_inst = ticksToVel(cR, dt_s) * (vR_tgt >= 0 ? 1.0f : -1.0f);
    vL_ema = EMA_B * vL_ema + (1 - EMA_B) * vL_meas_inst;
    vR_ema = EMA_B * vR_ema + (1 - EMA_B) * vR_meas_inst;
    const float V_MAX = 1.5f;
    vL_tgt = clampf(vL_tgt, -V_MAX, V_MAX);
    vR_tgt = clampf(vR_tgt, -V_MAX, V_MAX);
    int pwmL = pidStep(pidL, vL_tgt, vL_ema, dt_s);
    int pwmR = pidStep(pidR, vR_tgt, vR_ema, dt_s);
    int pwmL_cmd = shape_pwm(pwmL, pwmL_prev);
    int pwmR_cmd = shape_pwm(pwmR, pwmR_prev);
    pwmL_prev = pwmL_cmd;
    pwmR_prev = pwmR_cmd;
    // ★ FIX C4: Removed PWM override that broke recovery sweep
    //   Each wheel now gets its own PID-controlled PWM (signed)
    driveWheelLeft(vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}