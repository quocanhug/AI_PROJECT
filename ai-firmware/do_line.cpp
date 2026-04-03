/**
 * @file do_line.cpp
 * @brief Module dò line và điều hướng tự hành trên lưới 5×3 (15 node).
 *
 * Chức năng chính:
 *  - Bám vạch đen bằng PID (5 cảm biến TCRT5000)
 *  - Nhận diện giao lộ và thực hiện rẽ theo lộ trình
 *  - Phát hiện vật cản (HC-SR04) và xử lý tương ứng
 *  - Phục hồi tự động khi mất line (lùi + quét tìm)
 *  - Đo vận tốc/quãng đường qua encoder quang học
 */

#include "do_line.h"
#include <Arduino.h>

/* ══════════════════════════════════════════════════════════════
 *  BIẾN EXTERN TỪ FILE CHÍNH (ai-firmware.ino)
 * ══════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════
 *  BẢN ĐỒ LƯỚI — 5 cột × 3 hàng = 15 node (0–14)
 *  Kích thước mỗi ô: 25cm rộng × 35cm cao
 * ══════════════════════════════════════════════════════════════ */
const int NODE_COUNT = 15;
const int node_coords[15][2] = {{30, 30},   {100, 30},  {170, 30},  {240, 30},
                                {310, 30},  {30, 100},  {100, 100}, {170, 100},
                                {240, 100}, {310, 100}, {30, 170},  {100, 170},
                                {170, 170}, {240, 170}, {310, 170}};

/** Tính hướng di chuyển từ nodeA đến nodeB. Trả về 0=xuống, 1=phải, 2=lên, 3=trái, -1=lỗi */
int getTargetDirection(int nodeA, int nodeB) {
  // Kiểm tra chỉ số node hợp lệ
  if (nodeA < 0 || nodeA >= NODE_COUNT || nodeB < 0 || nodeB >= NODE_COUNT)
    return -1;
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

/* ══════ Chân điều khiển động cơ (L298N Dual H-Bridge) ══════ */
#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

/* ══════ Chân cảm biến dò line (5× TCRT5000) ══════ */
#define L2_SENSOR 34
#define L1_SENSOR 32
#define M_SENSOR 33
#define R1_SENSOR 27
#define R2_SENSOR 25

/* ══════ Encoder quang học ══════ */
#define ENC_L 26
#define ENC_R 22
#define PULSES_PER_REV 20
#define PPR_EFFECTIVE (PULSES_PER_REV * 3)
#define MIN_EDGE_US 1500

/* ══════ Cảm biến siêu âm HC-SR04 (phát hiện vật cản) ══════ */
#define TRIG_PIN 21
#define ECHO_PIN 19

const float OBSTACLE_TH_CM = 25.0f;          // Ngưỡng phát hiện vật cản (cm)
const unsigned long US_TIMEOUT = 3000;        // Timeout đo khoảng cách (µs)

static unsigned long us_last_ms = 0;          // Thời điểm đo gần nhất
float us_dist_cm = 999.0f;                    // Khoảng cách đo được (cm)

static uint8_t obs_hit = 0;                   // Bộ đếm xác nhận vật cản liên tiếp
static bool obs_latched = false;              // Cờ đã xác nhận có vật cản

const float OBSTACLE_ON_CM = OBSTACLE_TH_CM;  // Ngưỡng bật cảnh báo
const float OBSTACLE_OFF_CM = 30.0f;          // Ngưỡng tắt cảnh báo (hysteresis)
const uint8_t OBS_HIT_N = 3;                  // Số lần đo liên tiếp để xác nhận
const unsigned long US_PERIOD_MS = 25;         // Chu kỳ đo (ms)

/* ══════ Thông số cơ khí xe robot ══════ */
const float WHEEL_RADIUS_M = 0.0325f;         // Bán kính bánh xe (m)
extern const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M;  // Chu vi bánh (m)
const float TRACK_WIDTH_M = 0.1150f;

/* ══════ Tham số điều khiển vận tốc (m/s) ══════ */
float v_base = 0.4f;                          // Vận tốc cơ sở khi bám line
float v_boost = 0.15f;                         // Bù vận tốc khi lệch nhẹ
float v_hard = 0.20f;                          // Bù vận tốc khi lệch mạnh
float v_search = 0.2f;                         // Vận tốc tìm line ban đầu

/* Bộ PID cho bánh trái và bánh phải */
PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

const unsigned long CTRL_DT_MS = 10;          // Chu kỳ điều khiển PID (ms)
volatile long encL_count = 0, encR_count = 0;  // Xung encoder mỗi chu kỳ (reset sau đọc)
volatile long encL_total = 0, encR_total = 0;  // Tổng xung encoder (tính quãng đường)
volatile uint32_t encL_last_us = 0, encR_last_us = 0;  // Thời điểm xung gần nhất (debounce)
float vL_ema = 0.0f, vR_ema = 0.0f;           // Vận tốc EMA (lọc nhiễu)
const float EMA_B = 0.7f;                      // Hệ số lọc EMA (0–1, cao = mượt hơn)

const int PWM_MIN_RUN = 75;                    // PWM tối thiểu để motor quay được
const int PWM_SLEW = 15;                       // Giới hạn thay đổi PWM mỗi chu kỳ (slew rate)
static int pwmL_prev = 0, pwmR_prev = 0;       // PWM chu kỳ trước (dùng cho slew limiter)

static unsigned long t_prev = 0;               // Thời điểm PID chu kỳ trước
static unsigned long bad_t = 0;                // Thời điểm bắt đầu mất line

/* ── Trạng thái dò line và phục hồi ── */
enum Side { NONE, LEFT, RIGHT };
Side last_seen = NONE;                         // Hướng cuối cùng nhìn thấy line
bool seen_line_ever = false;                   // Đã từng thấy line từ lúc khởi động
bool avoiding = false;                         // Đang trong quy trình tránh vật cản
static volatile bool g_line_enabled = true;    // Khóa an toàn module dò line
bool recovering = false;                       // Đang trong quy trình phục hồi mất line
unsigned long rec_t0 = 0;                      // Thời điểm bắt đầu recovery
const unsigned long RECOV_TIME_MS = 3000;      // Timeout recovery (3 giây)

/* Recovery: Lùi + quét nhẹ trái/phải, KHÔNG quay đầu */
int recov_sweep_count = 0;                     // Bộ đếm lần quét hiện tại
const int RECOV_MAX_SWEEPS = 8;                // Tối đa 8 lần quét luân phiên
bool recov_did_backup = false;                 // Đã thực hiện lùi lần 1
bool recov_did_second_backup = false;           // Đã thực hiện lùi lần 2

int lastConfirmedNodeIdx = 0;                  // Chỉ số node cuối cùng đã xác nhận đến

const unsigned long INTERSECTION_DEBOUNCE_MS = 500;  // Chống nhiễu giao lộ (ms)
unsigned long last_intersection_time = 0;      // Thời điểm xử lý giao lộ gần nhất

bool needs_initial_turn = false;               // Cờ cần xoay hướng khi nhận route mới

/* ══════════════════════════════════════════════════════════════
 *  NGẮT PHẦN CỨNG (ISR) — Đếm xung encoder
 *  Có debounce bằng MIN_EDGE_US để lọc nhiễu cơ khí.
 * ══════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════
 *  HÀM TIỆN ÍCH
 * ══════════════════════════════════════════════════════════════ */
inline int clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
/** Giới hạn PWM có dấu [-255, 255] (dùng cho recovery/quay tại chỗ) */
inline int clampSigned255(int v) {
  return v < -255 ? -255 : (v > 255 ? 255 : v);
}
inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
inline bool onLine(int pin) { return digitalRead(pin) == LOW; }

/** Làm mượt PWM: áp dụng slew rate limiter + ngưỡng tối thiểu, hỗ trợ PWM âm */
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

/* ══════════════════════════════════════════════════════════════
 *  ĐIỀU KHIỂN ĐỘNG CƠ — PWM có dấu, hỗ trợ tiến/lùi
 * ══════════════════════════════════════════════════════════════ */
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

/** Bước PID: tính PWM đầu ra có dấu từ vận tốc mục tiêu và đo được */
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

/* ══════════════════════════════════════════════════════════════
 *  HC-SR04 — Đo khoảng cách siêu âm + lọc trung vị 3 mẫu
 * ══════════════════════════════════════════════════════════════ */
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

/** Đo khoảng cách HC-SR04 có lọc trung vị (median-of-3) để giảm nhiễu */
static float readDistanceCM_filtered() {
  float samples[3];
  for (int i = 0; i < 3; i++) {
    samples[i] = readDistanceCM();
    if (i < 2)
      delayMicroseconds(500);
  }
  // Sort 3 values to get median
  if (samples[0] > samples[1]) {
    float t = samples[0];
    samples[0] = samples[1];
    samples[1] = t;
  }
  if (samples[1] > samples[2]) {
    float t = samples[1];
    samples[1] = samples[2];
    samples[2] = t;
  }
  if (samples[0] > samples[1]) {
    float t = samples[0];
    samples[0] = samples[1];
    samples[1] = t;
  }
  float d = samples[1]; // Giá trị trung vị
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

/* ══════════════════════════════════════════════════════════════
 *  HÌNH HỌC ENCODER — Tính quãng đường và góc quay từ xung
 * ══════════════════════════════════════════════════════════════ */
/** Chuyển đổi quãng đường (m) thành số xung encoder cần thiết */
long countsForDistance(double dist_m) {
  return (long)(dist_m / CIRC * PPR_EFFECTIVE + 0.5);
}

/** Cấp PWM có dấu cho cả hai bánh (dương=tiến, âm=lùi) */
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

/** Tính góc quay (radian) từ hiệu quãng đường hai bánh (differential drive) */
static inline double theta_from_counts(long dL, long dR, int signL, int signR) {
  return ((double)dR / PPR_EFFECTIVE * CIRC * signR -
          (double)dL / PPR_EFFECTIVE * CIRC * signL) /
         TRACK_WIDTH_M;
}

/* ══════════════════════════════════════════════════════════════
 *  QUAY TẠI CHỖ — Điều khiển góc bằng encoder + P-controller
 *  Có cân bằng hai bánh (Kbal) và giảm tốc khi gần đích.
 * ══════════════════════════════════════════════════════════════ */
/** Quay trái tại chỗ theo góc chỉ định. Trả về true nếu thành công. */
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

/** Quay phải tại chỗ theo góc chỉ định. Trả về true nếu thành công. */
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

/* ══════════════════════════════════════════════════════════════
 *  TIẾN THEO QUÃNG ĐƯỜNG — Dừng khi encoder đạt khoảng cách mục tiêu
 *  Có timeout 5 giây phòng trường hợp encoder lỗi.
 * ══════════════════════════════════════════════════════════════ */
/** Tiến thẳng một quãng đường cố định (mét) */
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

/** Tiến thẳng tối đa dist_m, dừng sớm nếu phát hiện line. Trả về true nếu tìm thấy line. */
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
    // Kiểm tra cả 5 cảm biến — dừng ngay nếu phát hiện line
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

/** Dừng khẩn cấp module dò line và reset tất cả cờ trạng thái */
void do_line_abort() {
  g_line_enabled = false;
  motorsStop();
  avoiding = false;
  obs_latched = false;
  obs_hit = 0;
  recovering = false;
}

/** Tiếp tục lộ trình từ node đã xác nhận, chỉ reset PID mà không reset route */
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

/** Quy trình tránh vật cản vật lý: rẽ trái → vượt qua → rẽ phải → tìm lại line */
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

/* ══════════════════════════════════════════════════════════════
 *  KHỞI TẠO MODULE — Cấu hình chân GPIO, encoder, reset trạng thái
 * ══════════════════════════════════════════════════════════════ */
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

/* ══════════════════════════════════════════════════════════════
 *  VÒNG LẶP CHÍNH — Dò line, xử lý giao lộ, phục hồi, PID
 * ══════════════════════════════════════════════════════════════ */
void do_line_loop() {
  if (!g_line_enabled) {
    motorsStop();
    return;
  }

  /* ===== XUẤT PHÁT: Xoay xe đúng hướng khi nhận lộ trình mới ===== */
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
          turnOk = spin_left_deg(60.0, TURN_PWM);  // 60×1.5=90° thực tế
          Serial.println("  >> INIT LEFT");
        } else if (diff == 3) {
          turnOk = spin_right_deg(60.0, TURN_PWM); // 60×1.5=90° thực tế
          Serial.println("  >> INIT RIGHT");
        } else if (diff == 2) {
          turnOk = spin_right_deg(120.0, TURN_PWM); // 120×1.5=180° thực tế
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

        // Tìm line sau khi quay — tiến 8cm (để không vượt quá ô 25cm)
        bool lineFoundAfterTurn = move_forward_distance_until_line(0.08, 90);
        if (!lineFoundAfterTurn) {
          Serial.println("  INIT: line not found after fwd, trying sweep");
          // Quét trái/phải tìm line
          bool foundWide = false;
          for (int sweep = 0; sweep < 3 && !foundWide; sweep++) {
            // Quét trái
            motorWriteLR_signed(-90, 90);
            unsigned long ts = millis();
            while (millis() - ts < 250) {
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
            // Quét phải
            motorWriteLR_signed(90, -90);
            ts = millis();
            while (millis() - ts < 250) {
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

      // Trường hợp đi thẳng: hướng đích trùng hướng hiện tại
      currentPathIndex++;
      last_intersection_time = millis();
      seen_line_ever = true;
      return;
    }
  }

  /* ===== ĐỌc cảm biến line ===== */
  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2)
    seen_line_ever = true;

  /* Biến static lưu vị trí encoder cuối cùng khi còn thấy line */
  static unsigned long ignore_intersection_until = 0; 
  static long last_line_encL = 0;
  static long last_line_encR = 0;
  static long overshot_target = 0;

  // ===== Mat line hoan toan -> recovery =====
  bool lost_all = !L2 && !L1 && !M && !R1 && !R2;
  if (lost_all) {
    if (!seen_line_ever && is_auto_running) {
      bad_t = millis(); // Chưa từng thấy line, chờ tìm
    } else if (!recovering && seen_line_ever) {
      // Phản ứng ngay lập tức khi mất line (không delay)
      Serial.println("[LINE] Mất line! VÀO RECOVERY");
      recovering = true;
      rec_t0 = millis();
      recov_sweep_count = 0;
      recov_did_backup = false;
      recov_did_second_backup = false;

      motorWriteLR_signed(-200, -200);
      delay(30); 
      motorsStop();
      
      // Tính khoảng cách overshoot (lệch khỏi line)
      long cL, cR;
      noInterrupts();
      cL = encL_total;
      cR = encR_total;
      interrupts();
      
      // Độ lệch rất nhỏ do phản ứng nhanh, cộng thêm 2cm để đảm bảo lùi đủ lại line
      overshot_target = max(labs(cL - last_line_encL), labs(cR - last_line_encR));
      
      // Lùi cố định 2cm để đảm bảo cảm biến quay lại đè lên vạch
      overshot_target += countsForDistance(0.02); 

      motorsStop();
      return;
    }
  } else {
    bad_t = millis();
    if (recovering) {
      recovering = false;
      Serial.println("[LINE] Recovery SUCCESS - line found!");
      ignore_intersection_until = millis() + 800;
    }
    
    // Lưu vị trí encoder liên tục khi còn thấy line (để tính overshoot)
    noInterrupts();
    last_line_encL = encL_total;
    last_line_encR = encR_total;
    interrupts();
  }

  updateObstacleState();

  /* ===== AI Route / Delivery: vật cản → dừng và báo Web Dashboard ===== */
  if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) &&
      obs_latched && !avoiding) {
    motorsStop();
    obs_latched = false;
    obs_hit = 0;  // Reset bộ đếm để tránh lại ngưỡng ngay sau khi xóa
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

  /* ===== Các chế độ khác: tránh vật cản vật lý ===== */
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

  /* ════════════════════════════════════════════════════════════
   * GIAO LỘ: Cả L2 và R2 đều phát hiện vạch ngang
   * ════════════════════════════════════════════════════════════ */
  bool at_intersection = (L2 && R2 && (L1 || M || R1));

  if (at_intersection && millis() < ignore_intersection_until) {
    at_intersection = false;
  }

  if (at_intersection) {

    /* --- MODE_LINE_ONLY: đi thẳng qua giao lộ --- */
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
            if (!g_line_enabled) {
              motorsStop();
              return;
            } // Kiểm tra E-STOP
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

    /* --- MODE_DELIVERY / MODE_AI_ROUTE: xử lý node trên lộ trình --- */
    else if (currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();

        if (pathLength > 0) {
          // Kiểm tra đã đến đích chưa
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

          // Tính hướng rẽ tại giao lộ
          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          int diff = (targetDir != -1) ? (targetDir - currentDir + 4) % 4 : 0;
          Serial.printf(
              "[NODE] path[%d]=node%d -> node%d, dir %d->%d (diff=%d)\n",
              currentPathIndex, curNode, nxtNode, currentDir, targetDir, diff);

          if (diff == 0) {
            // Đi thẳng — hướng đích trùng hướng hiện tại
            lastConfirmedNodeIdx = currentPathIndex;
            currentPathIndex++;
            currentDir = targetDir;
            move_forward_distance(0.03, 100);
            last_seen = NONE;
          } else {
            // Rẽ trái / phải / quay đầu
            move_forward_distance(0.03, 100);
            motorsStop();
            delay(300);

            // Reset PID trước khi quay để tránh giật motor
            pidL.i_term = 0;
            pidL.prev_err = 0;
            pidR.i_term = 0;
            pidR.prev_err = 0;

            const int TURN_PWM = 135;
            bool turnOk = true;
            motorsStop();
            delay(300);
            if (diff == 1) {
              turnOk = spin_left_deg(50.0, TURN_PWM);  // 50×1.5=75° thực tế
              Serial.println("  >> LEFT");
            } else if (diff == 3) {
              turnOk = spin_right_deg(50.0, TURN_PWM); // 50×1.5=75° thực tế
              Serial.println("  >> RIGHT");
            } else if (diff == 2) {
              turnOk = spin_right_deg(105.0, TURN_PWM); // 105×1.5=157° thực tế
              Serial.println("  >> U-TURN");
            }

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

    /* --- Các chế độ khác --- */
    else {
      vL_tgt = v_base * 0.8f;
      vR_tgt = v_base * 0.8f;
    }
  }

  /* ════════════════════════════════════════════════════════════
   * PHỤC HỒI MẤT LINE: Lùi nhẹ + quét nhẹ trái/phải
   * Lưới nhỏ 35×25cm → lùi ngắn, quét nhẹ, KHÔNG quay đầu.
   * ════════════════════════════════════════════════════════════ */
  else if (recovering) {
    // Tìm thấy line → thoát recovery
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

    // Timeout recovery → dừng hẳn, báo Web (giữ nguyên hướng)
    if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false;
      motorsStop();
      Serial.println("[RECOV] TIMEOUT - recovery failed");

      if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) &&
          is_auto_running) {
        currentPathIndex = lastConfirmedNodeIdx;
        extern void wsBroadcast(const char *);
        int robotNode = currentPath[lastConfirmedNodeIdx];
        // Giữ nguyên hướng hiện tại, không đảo 180°
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

    /* ===== QUY TRÌNH PHỤC HỒI ═════
     * Bước 1: Lùi nhẹ 10cm tìm line phía sau + căn chỉnh góc
     * Bước 2: Quét nhẹ trái/phải (góc nhỏ, tốc độ thấp)
     * Bước 3: Lùi thêm 2cm (sau 4 sweep đầu)
     * Bước 4: Quét tiếp, rộng hơn một chút
     * KHÔNG BAO GIỜ quay đầu (U-turn) */
    {
      /* === Bước 1: Lùi dò line + Căn chỉnh góc (Auto-Align) === */
      if (!recov_did_backup) {
        recov_did_backup = true;
        Serial.println("[RECOV] Buoc 1: Lui tim lai line...");
        
        const int REV_PWM = 110; 
        const long maxRevTarget = countsForDistance(0.10); 
        long sL, sR;
        
        noInterrupts();
        sL = encL_total;
        sR = encR_total;
        interrupts();
        
        motorWriteLR_signed(-REV_PWM, -REV_PWM);
        unsigned long bk = millis();
        
        while (millis() - bk < 2000) { 
          if (!g_line_enabled) {
            motorsStop();
            return;
          }
          
          // Đọc riêng biệt trạng thái các cụm mắt cảm biến
          bool l_hit = (digitalRead(L2_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW);
          bool r_hit = (digitalRead(R2_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW);
          bool m_hit = (digitalRead(M_SENSOR) == LOW);

          // Khi bất kỳ cảm biến nào phát hiện line
          if (l_hit || r_hit || m_hit) {
            motorsStop();
            delay(50); // Phanh ổn định
            
            // Căn chỉnh: xoay để mắt giữa (M) đè lên vạch
            if (l_hit && !m_hit && !r_hit) {
              Serial.println("  -> Lệch phải -> XOAY TRÁI");
              motorWriteLR_signed(-100, 100);  // Xoay trái tại chỗ
              unsigned long align_t = millis();
              // Xoay đến khi mắt giữa ngậm line (tối đa 800ms)
              while(millis() - align_t < 800) { 
                 if(digitalRead(M_SENSOR) == LOW) break; 
                 delay(5);
              }
              motorsStop();
            } 
            else if (r_hit && !m_hit && !l_hit) {
              Serial.println("  -> Lệch trái -> XOAY PHẢI");
              motorWriteLR_signed(100, -100);  // Xoay phải tại chỗ
              unsigned long align_t = millis();
              while(millis() - align_t < 800) {
                 if(digitalRead(M_SENSOR) == LOW) break;
                 delay(5);
              }
              motorsStop();
            }

            recovering = false;
            Serial.println("[RECOV] Căn chỉnh xong, tiếp tục lộ trình");
            bad_t = millis();
            t_prev = millis();
            
            // Miễn nhiễm giao lộ 800ms để xe thoát khỏi vùng lộn xộn
            ignore_intersection_until = millis() + 800; 
            return;
          }
          
          long cL, cR;
          noInterrupts();
          cL = encL_total;
          cR = encR_total;
          interrupts();
          
          // Đã lùi hết 10cm mà chưa thấy line → chuyển sang quét
          if (labs(cL - sL) >= maxRevTarget && labs(cR - sR) >= maxRevTarget) {
            break;
          }
            
          delay(5);
        }
        
        motorsStop();
        delay(100);
        rec_t0 = millis(); // Reset timer cho pha quét
        recov_sweep_count = 0;
        return;
      }

      /* === Bước 3: Lùi thêm 2cm (sau 4 sweep đầu) === */
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
          if (!g_line_enabled) {
            motorsStop();
            return;
          }
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
          if (labs(cL - sL) >= revTarget && labs(cR - sR) >= revTarget)
            break;
          delay(5);
        }
        motorsStop();
        delay(100);
        rec_t0 = millis();     // Reset timer cho sweep tiếp
        recov_sweep_count = 0; // Quét lại từ đầu
        return;
      }

      /* === Bước 2 & 4: Quét nhẹ trái/phải ===
       * Ưu tiên hướng cuối cùng thấy line. Tốc độ thấp, góc nhỏ. */
      unsigned long elapsed = millis() - rec_t0;

      bool sweepToLeft;
      if (last_seen == LEFT) {
        sweepToLeft = (recov_sweep_count % 2 == 0); // Trái trước
      } else if (last_seen == RIGHT) {
        sweepToLeft = (recov_sweep_count % 2 != 0); // Phải trước
      } else {
        sweepToLeft = (recov_sweep_count % 2 == 0);
      }

      // Quét nhẹ: tốc độ thấp (0.2 × v_base), tránh quay quá mạnh
      const float SWEEP_V = v_base * 0.2f;
      if (sweepToLeft) {
        vL_tgt = -SWEEP_V;
        vR_tgt = SWEEP_V;
      } else {
        vL_tgt = SWEEP_V;
        vR_tgt = -SWEEP_V;
      }

      // Chuyển sang sweep tiếp (thời lượng tăng dần)
      unsigned long total_sweep_time = 0;
      for (int i = 0; i <= recov_sweep_count && i < RECOV_MAX_SWEEPS; i++) {
        total_sweep_time += 100 + i * 50; // 180ms, 230ms, 280ms, ...
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

  /* ════════════════════════════════════════════════════════════
   * DÒ LINE: Tính vận tốc mục tiêu theo trạng thái 5 cảm biến
   * Dùng 3 mắt giữa (L1, M, R1) cho điều khiển chính,
   * L2/R2 cho trường hợp lệch mạnh.
   * ════════════════════════════════════════════════════════════ */
  else {
    // Mức 1: Thẳng hoàn toàn — chỉ M sáng
    if (M && !L1 && !R1) {
      last_seen = NONE;
      vL_tgt = v_base;
      vR_tgt = v_base;
    }
    // Mức 2: Lệch nhẹ — M + L1 hoặc M + R1
    else if (M && L1 && !R1) {
      last_seen = LEFT;
      vL_tgt = v_base - v_boost;
      vR_tgt = v_base + v_boost;
    } else if (M && R1 && !L1) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_boost;
      vR_tgt = v_base - v_boost;
    }
    // Mức 3: Lệch vừa — chỉ L1 hoặc R1 (mất M)
    else if (L1 && !M) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard;
      vR_tgt = v_base + v_hard;
    } else if (R1 && !M) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard;
      vR_tgt = v_base - v_hard;
    }
    // Mức 4: Lệch mạnh — chỉ còn L2 hoặc R2
    else if (L2 && !L1 && !M && !R1 && !R2) {
      last_seen = LEFT;
      vL_tgt = v_base - v_hard;
      vR_tgt = v_base + v_hard;
    } else if (R2 && !L1 && !M && !R1 && !L2) {
      last_seen = RIGHT;
      vL_tgt = v_base + v_hard;
      vR_tgt = v_base - v_hard;
    }
    // Mức 5: Mất hoàn toàn — cả 5 cảm biến tắt
    else if (!L2 && !L1 && !M && !R1 && !R2) {
      if (!seen_line_ever && is_auto_running) {
        vL_tgt = v_search;
        vR_tgt = v_search;
      } else if (!seen_line_ever) {
        vL_tgt = 0;
        vR_tgt = 0;
      } else {
        // Mất line → dừng ngay, chờ recovery xử lý
        vL_tgt = 0;
        vR_tgt = 0;
      }
    } else {
      vL_tgt = v_base;
      vR_tgt = v_base;
    }
  }

  /* ════════════════════════════════════════════════════════════
   * VÒNG LẶP PID — Đo vận tốc, tính PWM, cấp ra motor
   * ════════════════════════════════════════════════════════════ */
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
    // Cấp PWM có dấu cho từng bánh theo PID
    driveWheelLeft(vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}