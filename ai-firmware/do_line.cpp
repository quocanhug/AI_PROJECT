#include "do_line.h"
#include <Arduino.h>

// ================= Extern từ main.ino =================
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

// ================= Grid thuc te: 4x2 o (5 cot x 3 hang) = 15 node (0-14)
const int NODE_COUNT = 15;
const int node_coords[15][2] = {{30, 30},   {100, 30},  {170, 30},  {240, 30},
                                {310, 30},  {30, 100},  {100, 100}, {170, 100},
                                {240, 100}, {310, 100}, {30, 170},  {100, 170},
                                {170, 170}, {240, 170}, {310, 170}};

int getTargetDirection(int nodeA, int nodeB) {
  if (nodeA < 0 || nodeA >= NODE_COUNT || nodeB < 0 || nodeB >= NODE_COUNT)
    return -1;
  int dx = node_coords[nodeB][0] - node_coords[nodeA][0];
  int dy = node_coords[nodeB][1] - node_coords[nodeA][1];
  if (dy > 0) return 0; // down
  if (dx > 0) return 1; // right
  if (dy < 0) return 2; // up
  if (dx < 0) return 3; // left
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
extern const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M;
const float TRACK_WIDTH_M = 0.1150f;

// ================= Tham so dieu khien =================
float v_base = 0.4f;
float v_boost = 0.15f;
float v_hard = 0.20f;
float v_search = 0.2f;

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
const unsigned long RECOV_TIME_MS = 3000;

// Recovery state
int recov_sweep_count = 0;
const int RECOV_MAX_SWEEPS = 4;       // 4 sweep (trai-phai luan phien, ±15°)
bool recov_did_backup = false;

int lastConfirmedNodeIdx = 0;

const unsigned long INTERSECTION_DEBOUNCE_MS = 300;
unsigned long last_intersection_time = 0;

// ★ FIX: Latch L2/R2 trong cua so thoi gian
// Khi xe hoi lech, L2 va R2 khong trigger CUNG LUC.
// Latch: nho L2/R2 trong 100ms de khong bo lo node.
static bool l2_latched = false, r2_latched = false;
static unsigned long l2_latch_ms = 0, r2_latch_ms = 0;
const unsigned long INTERSECTION_LATCH_MS = 100; // cua so 100ms

bool needs_initial_turn = false;

// ★ FIX L3: Timeout khi search line ban đầu
unsigned long search_start_ms = 0;
const unsigned long SEARCH_TIMEOUT_MS = 10000;

// ★ NEW: Encoder distance tracking from last confirmed node
// Used by recovery to limit reverse distance
long enc_at_last_node_L = 0, enc_at_last_node_R = 0;

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
inline int clampSigned255(int v) {
  return v < -255 ? -255 : (v > 255 ? 255 : v);
}
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline bool onLine(int pin) { return digitalRead(pin) == LOW; }

static inline int shape_pwm(int target, int prev) {
  int s = target;
  if (s > 0 && s < PWM_MIN_RUN) s = PWM_MIN_RUN;
  if (s < 0 && s > -PWM_MIN_RUN) s = -PWM_MIN_RUN;
  int d = s - prev;
  if (d > PWM_SLEW) s = prev + PWM_SLEW;
  if (d < -PWM_SLEW) s = prev - PWM_SLEW;
  return clampSigned255(s);
}

// ================= Motor control =================
void driveWheelRight(float v_target, int pwm) {
  int d = clamp255(abs(pwm));
  if (v_target >= 0) {
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, d);
  } else {
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, d);
  }
}

void driveWheelLeft(float v_target, int pwm) {
  int d = clamp255(abs(pwm));
  if (v_target >= 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, d);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, d);
  }
}

void motorsStop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

float ticksToVel(long ticks, float dt_s) {
  return ((float)ticks / (float)PPR_EFFECTIVE * CIRC) / dt_s;
}

int pidStep(PID &pid, float v_target, float v_meas, float dt_s) {
  if (dt_s < 0.001f) dt_s = 0.001f;
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
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

// ★ FIX C3: Single-shot non-blocking — 1 lần pulseIn mỗi call (~3ms thay vì ~10ms)
// Dùng circular buffer 3 mẫu + lấy median để giảm blocking từ 40% CPU xuống ~12%
static float us_ring[3] = {999.0f, 999.0f, 999.0f};
static uint8_t us_ring_idx = 0;

static float readDistanceCM_filtered() {
  float d = readDistanceCM();
  if (d < 2.0f || d > 200.0f) d = 999.0f;
  us_ring[us_ring_idx] = d;
  us_ring_idx = (us_ring_idx + 1) % 3;
  // Median of 3
  float a = us_ring[0], b = us_ring[1], c = us_ring[2];
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static inline void updateObstacleState() {
  if (millis() - us_last_ms < US_PERIOD_MS) return;
  us_last_ms = millis();
  us_dist_cm = readDistanceCM_filtered();
  if (!obs_latched) {
    if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM) {
      if (++obs_hit >= OBS_HIT_N) obs_latched = true;
    } else {
      obs_hit = 0;
    }
  } else {
    if (us_dist_cm >= OBSTACLE_OFF_CM) {
      obs_latched = false; obs_hit = 0;
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
  if (pwmR >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, pwmR); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, -pwmR); }
  if (pwmL >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, pwmL); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, -pwmL); }
}

static inline double theta_from_counts(long dL, long dR, int signL, int signR) {
  return ((double)dR / PPR_EFFECTIVE * CIRC * signR -
          (double)dL / PPR_EFFECTIVE * CIRC * signL) /
         TRACK_WIDTH_M;
}

// ================= Quay theo goc (★ BO HE SO 1.5x — goc truyen vao la goc THUC) =================
bool spin_left_deg(double deg, int pwmMax) {
  // ★ FIX: Bo he so 1.5 — target la goc thuc te (90° = 90°)
  const double target = deg * 3.141592653589793 / 180.0;
  const double deg_tol = 2.0 * 3.141592653589793 / 180.0; // 2° tolerance
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  long L0, R0;
  noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  while (true) {
    if (!g_line_enabled) { motorsStop(); return false; }
    long L, R;
    noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, -1, +1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) {
      success = false;
      Serial.println("[SPIN] LEFT TIMEOUT!");
      break;
    }
    int pwmCap = (fabs(err) < 15.0 * 3.141592653589793 / 180.0) ? (pwmMax * 0.5) : pwmMax;
    int base = (int)clamp255((int)(fabs(err) * Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);
    int corr = (int)(Kbal * ((double)dR - (double)dL));
    int pwmL = base - corr; if (pwmL < pwmMin) pwmL = pwmMin;
    int pwmR = base + corr; if (pwmR < pwmMin) pwmR = pwmMin;
    motorWriteLR_signed(-pwmL, +pwmR);
    delay(2);
  }
  motorsStop();
  return success;
}

bool spin_right_deg(double deg, int pwmMax) {
  // ★ FIX: Bo he so 1.5 — target la goc thuc te
  const double target = -(deg * 3.141592653589793 / 180.0);
  const double deg_tol = 2.0 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  long L0, R0;
  noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  while (true) {
    if (!g_line_enabled) { motorsStop(); return false; }
    long L, R;
    noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) {
      success = false;
      Serial.println("[SPIN] RIGHT TIMEOUT!");
      break;
    }
    int pwmCap = (fabs(err) < 15.0 * 3.141592653589793 / 180.0) ? (pwmMax * 0.5) : pwmMax;
    int base = (int)clamp255((int)(fabs(err) * Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);
    int corr = (int)(Kbal * ((double)dL - (double)dR));
    int pwmL = base + corr; if (pwmL < pwmMin) pwmL = pwmMin;
    int pwmR = base - corr; if (pwmR < pwmMin) pwmR = pwmMin;
    motorWriteLR_signed(+pwmL, -pwmR);
    delay(2);
  }
  motorsStop();
  return success;
}

// ================= Tien theo quang duong =================
void move_forward_distance(double dist_m, int pwmAbs) {
  long target = countsForDistance(dist_m);
  long sL, sR;
  noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
  unsigned long t0 = millis();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true) {
    if (!g_line_enabled) { motorsStop(); return; }
    if (millis() - t0 > 5000) { Serial.println("[FWD] TIMEOUT!"); break; }
    long cL, cR;
    noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
    bool left_done = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done) break;
    if (left_done && !right_done) motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done) motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
}

bool move_forward_distance_until_line(double dist_m, int pwmAbs) {
  long target = countsForDistance(dist_m);
  long sL, sR;
  noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
  unsigned long t0 = millis();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true) {
    if (!g_line_enabled) { motorsStop(); return false; }
    if (millis() - t0 > 5000) { Serial.println("[FWD_LINE] TIMEOUT!"); break; }
    long cL, cR;
    noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
    if (digitalRead(M_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW ||
        digitalRead(R1_SENSOR) == LOW || digitalRead(L2_SENSOR) == LOW ||
        digitalRead(R2_SENSOR) == LOW) {
      motorsStop();
      return true;
    }
    bool left_done = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done) break;
    if (left_done && !right_done) motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done) motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
  return false;
}

// ================= ★ Robot dimensions =================
// Xe dai ~22cm, grid 25x35cm
// Sensor bar o dau xe, truc banh xe (tam quay) nam phia sau sensor
// ★ CALIBRATE: Do tu thanh sensor den giua 2 truc banh (cm)
//   Neu xe quay chua dung tam: tang len. Quay qua tam: giam xuong.
const float SENSOR_TO_AXLE_CM = 3.0f;

// ================= ★ NEW: Center on intersection =================
// 2 pha: (1) Tien qua cross-bar cho den khi L2+R2 tat
//        (2) Tien them SENSOR_TO_AXLE de truc banh vao dung tam giao lo
void center_on_intersection() {
  const int CENTER_PWM = 85;
  const double MAX_PHASE1 = 0.07; // max 7cm cho pha 1
  long target1 = countsForDistance(MAX_PHASE1);
  long sL, sR;
  noInterrupts(); sL = encL_total; sR = encR_total; interrupts();

  // === Pha 1: Tien cho den khi L2+R2 tat (qua cross-bar) ===
  motorWriteLR_signed(CENTER_PWM, CENTER_PWM);
  unsigned long t0 = millis();
  bool cross_cleared = false;

  while (millis() - t0 < 1500) {
    if (!g_line_enabled) { motorsStop(); return; }
    long cL, cR;
    noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
    if (labs(cL - sL) >= target1 || labs(cR - sR) >= target1) {
      Serial.println("[CENTER] Phase1: max dist, forcing Phase2");
      cross_cleared = true;
      break;
    }
    if (!onLine(L2_SENSOR) && !onLine(R2_SENSOR)) {
      cross_cleared = true;
      break;
    }
    delay(2);
  }
  motorsStop();

  // === Pha 2: Tien them SENSOR_TO_AXLE cm de dat truc banh vao tam ===
  if (cross_cleared && SENSOR_TO_AXLE_CM > 0.5f) {
    delay(50);
    float extra_m = SENSOR_TO_AXLE_CM / 100.0f;
    Serial.printf("[CENTER] Phase2: +%.1fcm for axle alignment\n", SENSOR_TO_AXLE_CM);
    move_forward_distance(extra_m, CENTER_PWM);
  }

  motorsStop();
  delay(100);
  Serial.println("[CENTER] Done — wheel axis at intersection");
}

// ================= ★ NEW: seek_by_spin =================
// Spin slowly looking for line, limited by encoder angle
// Returns true if line found (M, L1 or R1)
bool seek_by_spin(bool go_left, double max_deg, int pwm) {
  const double target_rad = max_deg * 3.141592653589793 / 180.0;
  long L0, R0;
  noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();

  if (go_left) motorWriteLR_signed(-pwm, pwm);
  else motorWriteLR_signed(pwm, -pwm);

  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    if (!g_line_enabled) { motorsStop(); return false; }
    if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR)) {
      motorsStop();
      return true;
    }
    long dL = labs(encL_total - L0);
    long dR = labs(encR_total - R0);
    double theta = theta_from_counts(dL, dR,
                                     go_left ? -1 : 1,
                                     go_left ? 1 : -1);
    if (fabs(theta) >= target_rad) break;
    delay(2);
  }
  motorsStop();
  return false;
}

// ================= ★ NEW: seek_line_after_turn =================
// After spinning 90°/180°, fine-tune alignment to lock onto the line
// 1) Check if already on line
// 2) Advance 4cm looking for line
// 3) Sweep same direction as turn ±15°
// 4) Sweep opposite direction 30° (through center to 15° other side)
// 5) Return to center
bool seek_line_after_turn(bool turned_left) {
  // Already on line?
  if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR)) {
    Serial.println("[SEEK] Already on line after turn");
    return true;
  }

  // Step 1: Advance 4cm looking for line
  bool found = move_forward_distance_until_line(0.04, 80);
  if (found) {
    Serial.println("[SEEK] Line found after 4cm advance");
    return true;
  }

  // Step 2: Sweep same direction as turn, max 15°
  const int SEEK_PWM = 100;
  found = seek_by_spin(turned_left, 15.0, SEEK_PWM);
  if (found) {
    Serial.println("[SEEK] Line found with same-dir 15deg sweep");
    return true;
  }

  // Step 3: Sweep opposite direction 30° (15° back to center + 15° other side)
  found = seek_by_spin(!turned_left, 30.0, SEEK_PWM);
  if (found) {
    Serial.println("[SEEK] Line found with opposite-dir 30deg sweep");
    return true;
  }

  // Step 4: Return to approximate center (15° back)
  seek_by_spin(turned_left, 15.0, SEEK_PWM);

  Serial.println("[SEEK] Line NOT found after turn!");
  return false;
}

// ================= ★ NEW: Save encoder reference at node =================
inline void saveNodeEncoderRef() {
  noInterrupts();
  enc_at_last_node_L = encL_total;
  enc_at_last_node_R = encR_total;
  interrupts();
}

// ★ NEW: Get distance traveled since last node (meters)
float getDistFromLastNode() {
  long dL, dR;
  noInterrupts();
  dL = labs(encL_total - enc_at_last_node_L);
  dR = labs(encR_total - enc_at_last_node_R);
  interrupts();
  return ((float)(dL + dR) / 2.0f / PPR_EFFECTIVE * CIRC);
}

// ★ NEW: Expected real-world distance between adjacent nodes on grid
// Grid 4x2: horizontal spacing = 25cm, vertical spacing = 35cm
float expectedDistM(int nodeA, int nodeB) {
  if (nodeA < 0 || nodeA >= NODE_COUNT || nodeB < 0 || nodeB >= NODE_COUNT)
    return 0.25f; // fallback: smallest cell dimension
  int dx = node_coords[nodeB][0] - node_coords[nodeA][0];
  int dy = node_coords[nodeB][1] - node_coords[nodeA][1];
  if (dx != 0 && dy == 0) return 0.25f; // horizontal (25cm)
  if (dy != 0 && dx == 0) return 0.35f; // vertical (35cm)
  return 0.25f; // fallback
}

// ================= Abort / Resume =================
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
  // ★ FIX: Kiểm tra sensor thực tế — nếu robot bị di chuyển thủ công, có thể không trên line
  seen_line_ever = (digitalRead(M_SENSOR) == LOW ||
                    digitalRead(L1_SENSOR) == LOW ||
                    digitalRead(R1_SENSOR) == LOW ||
                    digitalRead(L2_SENSOR) == LOW ||
                    digitalRead(R2_SENSOR) == LOW);
  recovering = false;
  avoiding = false;
  recov_did_backup = false;
  recov_sweep_count = 0;
  obs_latched = false;
  obs_hit = 0;

  vL_ema = 0.0f; vR_ema = 0.0f;
  pwmL_prev = 0; pwmR_prev = 0;
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;

  t_prev = millis();
  bad_t = millis();
  l2_latched = false; r2_latched = false;
  l2_latch_ms = 0; r2_latch_ms = 0;

  currentPathIndex = lastConfirmedNodeIdx;
  needs_initial_turn = true;
  saveNodeEncoderRef();
}

void avoidObstacle() {
  const int TURN_PWM = 150;
  const int FWD_PWM = 100;
  // ★ FIX: Angles are now REAL degrees (was 40*1.5=60, now pass 60 directly)
  spin_left_deg(60.0, TURN_PWM);
  motorsStop(); delay(500);
  move_forward_distance(0.2, FWD_PWM);
  motorsStop(); delay(500);
  spin_right_deg(60.0, TURN_PWM);
  motorsStop(); delay(500);
  move_forward_distance(0.15, FWD_PWM);
  motorsStop(); delay(500);
  spin_right_deg(75.0, TURN_PWM);
  motorsStop(); delay(500);
  move_forward_distance_until_line(0.6, FWD_PWM);
  motorsStop(); delay(500);
  spin_left_deg(22.0, TURN_PWM);
  motorsStop(); delay(500);
}

// ================= Setup =================
void do_line_setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(L2_SENSOR, INPUT); pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR, INPUT); pinMode(R1_SENSOR, INPUT); pinMode(R2_SENSOR, INPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);

  detachInterrupt(digitalPinToInterrupt(ENC_L));
  detachInterrupt(digitalPinToInterrupt(ENC_R));
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  g_line_enabled = true;
  seen_line_ever = false;
  vL_ema = 0.0f; vR_ema = 0.0f;
  pwmL_prev = 0; pwmR_prev = 0;
  us_last_ms = millis();
  us_dist_cm = 999.0f;
  obs_hit = 0; obs_latched = false;
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;
  last_intersection_time = 0;
  l2_latched = false; r2_latched = false;
  l2_latch_ms = 0; r2_latch_ms = 0;
  last_seen = NONE;
  recovering = false;
  recov_sweep_count = 0;
  recov_did_backup = false;
  lastConfirmedNodeIdx = 0;
  needs_initial_turn = true;
  search_start_ms = 0;  // ★ FIX L3: Reset search timeout
  saveNodeEncoderRef();

  t_prev = millis();
  bad_t = millis();
  motorsStop();
}

// ================= Main Loop =================
void do_line_loop() {
  if (!g_line_enabled) { motorsStop(); return; }

  // ===== INITIAL TURN: Xoay xe chuan huong khi nhan route moi =====
  // ★ FIX CRITICAL: Xe dang dung TAI path[0] (node xuat phat).
  // Chi CAN xoay huong, KHONG tang currentPathIndex.
  // currentPathIndex = 0 nghia la xe dang o path[0], chua di den path[1].
  // Khi xe di den intersection DAU TIEN, do la path[1].
  if (needs_initial_turn) {
    needs_initial_turn = false;

    if ((currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) &&
        pathLength >= 2 && currentPathIndex < pathLength - 1) {

      int curNode = currentPath[currentPathIndex];
      int nxtNode = currentPath[currentPathIndex + 1];
      int targetDir = getTargetDirection(curNode, nxtNode);

      Serial.printf("[INIT_TURN] node%d->node%d, dir %d->%d\n",
                    curNode, nxtNode, currentDir, targetDir);

      if (targetDir != -1 && targetDir != currentDir) {
        int diff = (targetDir - currentDir + 4) % 4;
        const int TURN_PWM = 160;
        bool turnOk = true;

        motorsStop();
        delay(300);

        if (diff == 1) {
          turnOk = spin_left_deg(90.0, TURN_PWM);
          Serial.println("  >> INIT LEFT 90deg");
        } else if (diff == 3) {
          turnOk = spin_right_deg(90.0, TURN_PWM);
          Serial.println("  >> INIT RIGHT 90deg");
        } else if (diff == 2) {
          turnOk = spin_right_deg(180.0, TURN_PWM);
          Serial.println("  >> INIT U-TURN 180deg");
        }

        motorsStop();
        delay(200);

        if (!turnOk) {
          Serial.println("  INIT TURN FAILED");
          motorsStop();
          extern void wsBroadcast(const char *);
          String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                       "\"robotNode\":" + String(curNode) +
                       ",\"robotDir\":" + String(currentDir) +
                       ",\"reason\":\"TURN_FAILED\"}";
          wsBroadcast(msg.c_str());
          currentMode = MODE_MANUAL;
          is_auto_running = false;
          line_mode = false;
          do_line_abort();
          return;
        }

        seek_line_after_turn(diff == 1);
        currentDir = targetDir;
      }

      // ★ FIX: KHONG tang currentPathIndex!
      // Xe van o path[0]. Intersection DAU TIEN se la path[1].
      Serial.printf("[INIT_TURN] Done. pathIdx=%d, dir=%d → ready to go to path[1]=node%d\n",
                    currentPathIndex, currentDir, currentPath[min(currentPathIndex + 1, pathLength - 1)]);
      last_intersection_time = millis();
      seen_line_ever = true;
      saveNodeEncoderRef();
      return;
    }
  } // ===== END INITIAL TURN =====

  // ===== Doc cam bien =====
  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2) seen_line_ever = true;

  // ★ FIX: Latch L2/R2 — nho trang thai trong cua so 100ms
  // Khi xe lech goc, L2 trigger truoc R2 (hoac nguoc lai) vai ms.
  // Latch dam bao khong bo lo khi 2 sensor khong dong thoi.
  unsigned long now_ms = millis();
  if (L2) { l2_latched = true; l2_latch_ms = now_ms; }
  if (R2) { r2_latched = true; r2_latch_ms = now_ms; }
  // Het han latch
  if (now_ms - l2_latch_ms > INTERSECTION_LATCH_MS) l2_latched = false;
  if (now_ms - r2_latch_ms > INTERSECTION_LATCH_MS) r2_latched = false;

  // ★ FIX: Obstacle check TRƯỚC lost-line — tránh bỏ sót vật cản khi recovery
  updateObstacleState();

  // ===== Mat line hoan toan -> recovery =====
  bool lost_all = !L2 && !L1 && !M && !R1 && !R2;
  if (lost_all) {
    if (!seen_line_ever && is_auto_running) {
      // ★ FIX L3: Search timeout — dừng sau 10s nếu không tìm thấy line
      if (search_start_ms == 0) search_start_ms = millis();
      if (millis() - search_start_ms > SEARCH_TIMEOUT_MS) {
        Serial.println("[LINE] Search timeout (10s) — stopping");
        motorsStop();
        currentMode = MODE_MANUAL;
        is_auto_running = false;
        line_mode = false;
        extern void wsBroadcast(const char *);
        wsBroadcast("{\"type\":\"OBSTACLE_DETECTED\",\"reason\":\"SEARCH_TIMEOUT\"}");
        do_line_abort();
        search_start_ms = 0;
        return;
      }
      bad_t = millis();
    } else if (!recovering && seen_line_ever && millis() - bad_t > 150) {
      Serial.println("[LINE] Lost >150ms -> entering RECOVERY");
      recovering = true;
      rec_t0 = millis();
      recov_sweep_count = 0;
      recov_did_backup = false;
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

  // ===== AI Route: vat can -> dung, bao Web =====
  if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) &&
      obs_latched && !avoiding) {
    motorsStop();
    obs_latched = false;
    obs_hit = 0;
    int robotNode = (lastConfirmedNodeIdx < pathLength)
                        ? currentPath[lastConfirmedNodeIdx] : 0;
    // ★ FIX: obstacleNode = node tiếp theo (robot đang hướng tới), không phải node đang đứng
    int obstacleNode = (currentPathIndex + 1 < pathLength)
                           ? currentPath[currentPathIndex + 1] : robotNode;
    extern void wsBroadcast(const char *);
    String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                 "\"robotNode\":" + String(robotNode) +
                 ",\"obstacleNode\":" + String(obstacleNode) +
                 ",\"robotDir\":" + String(currentDir) +
                 ",\"current_step\":" + String(lastConfirmedNodeIdx) +
                 ",\"distance_cm\":" + String(us_dist_cm, 1) + "}";
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
  // ★ FIX: MODE_DELIVERY cũng dùng web re-routing, không physical avoidance
  if (obs_latched && !avoiding && currentMode != MODE_AI_ROUTE && currentMode != MODE_DELIVERY) {
    avoiding = true;
    motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis();
    avoidObstacle();
    motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis();
    avoiding = false;
    return;
  }
  if (avoiding) return;

  float vL_tgt = 0, vR_tgt = 0;

  // ============================================================
  // ★ FIX: GIAO LO dung latch — L2 va R2 khong can trigger CUNG LUC
  // Chi can ca 2 da thay trong vong 100ms + it nhat 1 sensor giua
  // ============================================================
  // ★ FIX C2+M1: Skip intersection khi recovering (tránh nhảy node sai);
  //   Bỏ yêu cầu (L1||M||R1) — L2+R2 latch 100ms đã đủ detect intersection
  bool at_intersection = !recovering && l2_latched && r2_latched;

  if (at_intersection) {

    // ★ FIX: Refresh latch khi at_intersection nhưng bị debounce block — tránh hết hạn 100ms
    l2_latch_ms = now_ms;
    r2_latch_ms = now_ms;

    // --- MODE_LINE_ONLY: di thang qua giao lo ---
    if (currentMode == MODE_LINE_ONLY) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();
        l2_latched = false; r2_latched = false;
        motorsStop(); delay(200);
        move_forward_distance(0.06, 120);
        if (!onLine(M_SENSOR) && !onLine(L1_SENSOR) && !onLine(R1_SENSOR)) {
          motorWriteLR_signed(130, -130);
          unsigned long t0 = millis();
          while (millis() - t0 < 3500) {
            if (!g_line_enabled) { motorsStop(); return; }
            if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR)) break;
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
        // Reset latch sau khi chap nhan intersection
        l2_latched = false; r2_latched = false;

        if (pathLength > 0) {
          // ★ FIX: Distance validation — reject false intersections
          // pathIdx=0 nghia la xe dang o START.
          // Node dau tien xe gap la path[1] (vi path[0] la start).
          // Kiem tra khoang cach: xe phai di DU tu path[currentPathIndex] den path[currentPathIndex+1]
          float traveled = getDistFromLastNode();
          int fromNode = currentPath[currentPathIndex];
          int toNode = currentPath[min(currentPathIndex + 1, pathLength - 1)];
          float expected = expectedDistM(fromNode, toNode);
          float minDist = expected * 0.55f; // ★ FIX M4: 55% thay vì 35% — tránh false intersection
          if (traveled < minDist) {
            Serial.printf("[NODE] FALSE intersection! dist=%.1fcm < min=%.1fcm (exp=%.1fcm) — SKIP\n",
                          traveled * 100, minDist * 100, expected * 100);
            move_forward_distance(0.04, 100);
            last_intersection_time = millis();
            return;
          }
          Serial.printf("[NODE] Distance OK: %.1fcm (exp=%.1fcm) pathIdx=%d\n",
                        traveled * 100, expected * 100, currentPathIndex);

          // ★ FIX: Tang pathIndex TRUOC — xe vua den node path[currentPathIndex+1]
          currentPathIndex++;
          lastConfirmedNodeIdx = currentPathIndex;

          // Kiem tra da den dich chua
          if (currentPathIndex >= pathLength - 1) {
            move_forward_distance(0.02, 90);
            motorsStop(); delay(200);
            Serial.printf("[AI_ROUTE] ARRIVED at node %d (pathIdx=%d)\n",
                          currentPath[currentPathIndex], currentPathIndex);

            if (currentMode == MODE_DELIVERY) {
              // ★ FIX M3: Non-blocking wait — kiểm tra g_line_enabled trong delay
              gripOpen();
              { unsigned long gt0 = millis();
                while (millis() - gt0 < 1500) {
                  if (!g_line_enabled) { gripClose(); return; }
                  delay(10);
                }
              }
              gripClose();
            } else {
              Serial.println("[AI_ROUTE] DESTINATION REACHED - COMPLETED");
              extern void wsBroadcast(const char *);
              String msg = "{\"type\":\"COMPLETED\",\"robotNode\":" +
                           String(currentPath[currentPathIndex]) +
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

          // Tinh huong re: tu node hien tai den node ke tiep
          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          int diff = (targetDir != -1) ? (targetDir - currentDir + 4) % 4 : 0;
          Serial.printf("[NODE] AT node%d, NEXT node%d, dir %d->%d (diff=%d)\n",
                        curNode, nxtNode, currentDir, targetDir, diff);

          if (diff == 0) {
            // DI THANG — tien qua giao lo
            currentDir = targetDir;
            move_forward_distance(0.04, 100); // 4cm qua cross-bar
            last_seen = NONE;
            saveNodeEncoderRef();
          } else {
            // RE / U-TURN
            // ★ FIX: Center chinh xac truoc khi quay
            center_on_intersection();

            // Reset PID truoc khi quay
            pidL.i_term = 0; pidL.prev_err = 0;
            pidR.i_term = 0; pidR.prev_err = 0;

            const int TURN_PWM = 160;
            bool turnOk = true;
            // ★ FIX: Goc thuc te — 90° va 180°
            if (diff == 1) {
              turnOk = spin_left_deg(90.0, TURN_PWM);
              Serial.println("  >> LEFT 90deg");
            } else if (diff == 3) {
              turnOk = spin_right_deg(90.0, TURN_PWM);
              Serial.println("  >> RIGHT 90deg");
            } else if (diff == 2) {
              turnOk = spin_right_deg(180.0, TURN_PWM);
              Serial.println("  >> U-TURN 180deg");
            }

            motorsStop();
            delay(200);

            if (!turnOk) {
              Serial.println("  TURN FAILED");
              motorsStop();
              extern void wsBroadcast(const char *);
              int robotNode = currentPath[currentPathIndex];
              String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                           "\"robotNode\":" + String(robotNode) +
                           ",\"robotDir\":" + String(currentDir) +
                           ",\"reason\":\"TURN_FAILED\"}";
              wsBroadcast(msg.c_str());
              currentMode = MODE_MANUAL;
              is_auto_running = false;
              line_mode = false;
              do_line_abort();
              return;
            }

            currentDir = targetDir;

            // ★ FIX: Dung seek_line_after_turn thay vi tim bat ky line
            bool lineFound = seek_line_after_turn(diff == 1);
            saveNodeEncoderRef();

            if (!lineFound) {
              Serial.println("  seek_line_after_turn FAILED -> recovery");
              recovering = true;
              rec_t0 = millis();
              recov_sweep_count = 0;
              recov_did_backup = false;
            }
          }
        }
        // ★ FIX: Reset PID timing — tránh dt spike sau blocking intersection processing
        noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
        t_prev = millis();
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
  // ★ RECOVERY REWRITE: Huong-aware, gioi han goc, lui an toan
  // Grid 4x2 (35x25cm) — lui ngan, quet ±15° max, KHONG bat line sai
  // ============================================================
  else if (recovering) {
    // Tim thay line (L1/M/R1 nhung KHONG phai cross-bar L2+R2) -> thoat
    if ((L1 || M || R1) && !(L2 && R2)) {
      recovering = false;
      motorsStop();
      Serial.println("[RECOV] Correct line found (not cross-bar)!");
      bad_t = millis();
      vL_ema = 0; vR_ema = 0;
      pwmL_prev = 0; pwmR_prev = 0;
      pidL.i_term = 0; pidL.prev_err = 0;
      pidR.i_term = 0; pidR.prev_err = 0;
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis();
      return;
    }

    // Neu bat gap cross-bar (L2+R2) trong recovery -> bo qua, co the la line sai
    if (L2 && R2) {
      Serial.println("[RECOV] Hit cross-bar (L2+R2) — ignoring (may be wrong line)");
      // Tiep tuc recovery, khong chap nhan line nay
    }

    // Timeout recovery -> dung han, bao Web
    if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false;
      motorsStop();
      Serial.println("[RECOV] TIMEOUT - recovery failed");

      if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) &&
          is_auto_running) {
        currentPathIndex = lastConfirmedNodeIdx;
        extern void wsBroadcast(const char *);
        int robotNode = currentPath[lastConfirmedNodeIdx];
        String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                     "\"robotNode\":" + String(robotNode) +
                     ",\"robotDir\":" + String(currentDir) +
                     ",\"reason\":\"LINE_LOST\"}";
        wsBroadcast(msg.c_str());
        currentMode = MODE_MANUAL;
        is_auto_running = false;
        line_mode = false;
        do_line_abort();
        return;
      }
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis();
      return;
    }

    // ===== BUOC 1: Lui theo huong hien tai, gioi han boi quang da di =====
    if (!recov_did_backup) {
      recov_did_backup = true;

      float dist_from_node = getDistFromLastNode();
      // Lui toi da 80% quang da di, max 5cm, min 1cm
      float max_reverse = dist_from_node * 0.8f;
      if (max_reverse > 0.05f) max_reverse = 0.05f;
      if (max_reverse < 0.01f) max_reverse = 0.01f;

      Serial.printf("[RECOV] Buoc 1: Lui %.1fcm (da di %.1fcm tu node)\n",
                    max_reverse * 100, dist_from_node * 100);

      const int REV_PWM = 70;
      long revTarget = countsForDistance(max_reverse);
      long sL, sR;
      noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
      motorWriteLR_signed(-REV_PWM, -REV_PWM);
      unsigned long bk = millis();
      while (millis() - bk < 1000) {
        if (!g_line_enabled) { motorsStop(); return; }
        // Kiem tra line (L1/M/R1 nhung khong phai cross-bar)
        bool l2r = onLine(L2_SENSOR), r2r = onLine(R2_SENSOR);
        if ((onLine(L1_SENSOR) || onLine(M_SENSOR) || onLine(R1_SENSOR)) && !(l2r && r2r)) {
          motorsStop();
          recovering = false;
          Serial.println("[RECOV] Line found while backing!");
          bad_t = millis(); t_prev = millis();
          return;
        }
        long cL, cR;
        noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
        if (labs(cL - sL) >= revTarget && labs(cR - sR) >= revTarget) break;
        delay(5);
      }
      motorsStop();
      delay(100);
      rec_t0 = millis();
      recov_sweep_count = 0;
      return;
    }

    // ===== BUOC 2: Quet ±15° trai/phai (encoder-based angle limit) =====
    {
      const double SWEEP_DEG = 15.0;
      const int SWEEP_PWM = 80;

      if (recov_sweep_count >= RECOV_MAX_SWEEPS) {
        Serial.println("[RECOV] All sweeps exhausted, waiting timeout");
        motorsStop();
        delay(50);
        return;
      }

      // Uu tien huong last_seen
      bool sweepToLeft;
      if (last_seen == LEFT) sweepToLeft = (recov_sweep_count % 2 == 0);
      else if (last_seen == RIGHT) sweepToLeft = (recov_sweep_count % 2 != 0);
      else sweepToLeft = (recov_sweep_count % 2 == 0);

      Serial.printf("[RECOV] Sweep #%d %s (max ±%.0fdeg)\n",
                    recov_sweep_count, sweepToLeft ? "LEFT" : "RIGHT", SWEEP_DEG);

      // ★ FIX: Dung encoder-based angle gioi han ±15°, khong dung time
      bool found = seek_by_spin(sweepToLeft, SWEEP_DEG, SWEEP_PWM);
      if (found) {
        // Kiem tra khong phai cross-bar
        if (!(onLine(L2_SENSOR) && onLine(R2_SENSOR))) {
          recovering = false;
          Serial.println("[RECOV] Line found during sweep!");
          bad_t = millis();
          vL_ema = 0; vR_ema = 0;
          pwmL_prev = 0; pwmR_prev = 0;
          pidL.i_term = 0; pidL.prev_err = 0;
          pidR.i_term = 0; pidR.prev_err = 0;
          noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
          t_prev = millis();
          return;
        } else {
          Serial.println("[RECOV] Cross-bar hit during sweep, ignoring");
        }
      }

      // Quay ve vi tri trung tam sau moi sweep
      seek_by_spin(!sweepToLeft, SWEEP_DEG, SWEEP_PWM);
      motorsStop();
      delay(50);

      recov_sweep_count++;
      return;
    }
  }

  // ============================================================
  // DO LINE: PID dung 3 mat giua L1, M, R1
  // ============================================================
  else {
    if (M && !L1 && !R1) {
      last_seen = NONE;
      vL_tgt = v_base; vR_tgt = v_base;
    }
    else if (M && L1 && !R1) {
      last_seen = LEFT;
      vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost;
    } else if (M && R1 && !L1) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost;
    }
    else if (L1 && !M) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard; vR_tgt = v_base + v_hard;
    } else if (R1 && !M) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard; vR_tgt = v_base - v_hard;
    }
    else if (L2 && !L1 && !M && !R1 && !R2) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard; vR_tgt = v_base + v_hard;
    } else if (R2 && !L1 && !M && !R1 && !L2) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard; vR_tgt = v_base - v_hard;
    }
    else if (!L2 && !L1 && !M && !R1 && !R2) {
      if (!seen_line_ever && is_auto_running) {
        vL_tgt = v_search; vR_tgt = v_search;
      } else if (!seen_line_ever) {
        vL_tgt = 0; vR_tgt = 0;
      } else {
        vL_tgt = 0; vR_tgt = 0;
      }
    } else {
      vL_tgt = v_base; vR_tgt = v_base;
    }
  }

  // ================= PID =================
  unsigned long now = millis();
  if (now - t_prev >= CTRL_DT_MS) {
    float dt_s = (now - t_prev) / 1000.0f;
    t_prev = now;
    noInterrupts();
    long cL = encL_count; encL_count = 0;
    long cR = encR_count; encR_count = 0;
    interrupts();
    float vL_meas_inst = ticksToVel(cL, dt_s) * (vL_tgt >= 0 ? 1.0f : -1.0f);
    float vR_meas_inst = ticksToVel(cR, dt_s) * (vR_tgt >= 0 ? 1.0f : -1.0f);
    // ★ FIX M2: Reset PID integral khi target đổi chiều — tránh windup
    {
      static float prev_vL_sign = 1.0f, prev_vR_sign = 1.0f;
      float sL = (vL_tgt >= 0) ? 1.0f : -1.0f;
      float sR = (vR_tgt >= 0) ? 1.0f : -1.0f;
      if (sL != prev_vL_sign) { pidL.i_term = 0; prev_vL_sign = sL; }
      if (sR != prev_vR_sign) { pidR.i_term = 0; prev_vR_sign = sR; }
    }
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
    driveWheelLeft(vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}