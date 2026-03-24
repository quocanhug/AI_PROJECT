#include <Arduino.h>
#include "do_line.h"

/* ================= ESP32 30P + L298N + analogWrite =================
   Mapping theo yêu cầu và gợi ý:
   - Right motor: IN1=12, IN2=14, ENA=13
   - Left  motor: IN3=4,  IN4=2,  ENB=15
   - Line sensors: LEFT=32, MID=33, RIGHT=27
   - Encoders: ENC_L=26, ENC_R=22  (CHANGE)
   - HC-SR04: TRIG=21, ECHO=19
==================================================================== */

// ================= Motor pins (ESP32 + L298N) =================
// Left motor
#define IN1 12
#define IN2 14
#define ENA 13
// Right motor
#define IN3 4
#define IN4 2
#define ENB 15

// ================= Line sensors =================
#define L2_SENSOR   34   // outer-left
#define L1_SENSOR   32   // left  (giữ nguyên)
#define M_SENSOR    33   // middle(giữ nguyên)
#define R1_SENSOR   27   // right (giữ nguyên)
#define R2_SENSOR   25   // outer-right
// Nếu module TCRT xuất LOW trên line → đổi onLine() xuống LOW.

// ================= Encoders =================
#define ENC_L 26
#define ENC_R 22
#define PULSES_PER_REV 20            // 20 xung/vòng theo datasheet/encoder 1x . KHi đếm 2 sườn thì nhân đôi
const int PPR_EFFECTIVE = (PULSES_PER_REV*3); // đổi sang CHANGE → đếm 2 sườn

// Lọc nhiễu xung trong ISR (bỏ xung < MIN_EDGE_US)
#define MIN_EDGE_US 1500

// ================= HC-SR04 =================
#define TRIG_PIN 21
#define ECHO_PIN 19
 
const float OBSTACLE_TH_CM = 20.0f;     // cm

// Giảm timeout để tránh bắt nhiễu ở xa (30ms ~ 5m). 12ms ~ 2m.
const unsigned long US_TIMEOUT = 8000;

// ---- HC-SR04 filter/debounce/hysteresis ----
static unsigned long us_last_ms = 0;
static float us_dist_cm = 999.0f;

static uint8_t obs_hit = 0;
static bool obs_latched = false;

const float OBSTACLE_ON_CM  = OBSTACLE_TH_CM; // ngưỡng kích hoạt
const float OBSTACLE_OFF_CM = 20.0f;          // ngưỡng nhả (hysteresis) > ON
const uint8_t OBS_HIT_N = 2;                  // cần N lần liên tiếp mới kích
const unsigned long US_PERIOD_MS = 25;        // đo mỗi 60ms để giảm nhiễu

// ================= Thông số cơ khí =================
const float WHEEL_RADIUS_M = 0.0325f;
const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M; // m/vòng
const float TRACK_WIDTH_M = 0.1150f; // đo thực tế

// ================= Tham số điều khiển =================
float v_base   = 0.4f;    // m/s
float v_boost  = 0.11f;   // m/s
float v_hard   = 0.13f;   // m/s
float v_search = 0.2f;   // m/s
float vF = v_base * 0.90f;


// PID cho từng bánh (đã hạ Kp, thêm Ki nhỏ)
// PID struct đã định nghĩa trong do_line.h
PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

// Chu kỳ điều khiển
const unsigned long CTRL_DT_MS = 10;

// ================= Biến encoder/tốc độ =================
volatile long encL_count = 0;
volatile long encR_count = 0;
volatile long encL_total = 0;
volatile long encR_total = 0;
volatile uint32_t encL_last_us=0, encR_last_us=0;

// Lọc EMA tốc độ đo
static float vL_ema=0.0f, vR_ema=0.0f;
const float EMA_B = 0.7f; // 0.6–0.85

// Shaper PWM: deadband + slew-rate
const int PWM_MIN_RUN = 75;  // 65–90
const int PWM_SLEW    = 8;  // bước tối đa mỗi 20 ms
static int pwmL_prev=0, pwmR_prev=0;

// Lưu hướng lần cuối thấy line
enum Side {NONE, LEFT, RIGHT};
Side last_seen = NONE;

// Đã từng thấy line chưa
bool seen_line_ever = false;

// ================= Trạng thái né / recovery =================
bool avoiding = false;

static volatile bool g_line_enabled = true;

// Recovery theo thời gian khi mất line
bool recovering = false;
unsigned long rec_t0 = 0;
const unsigned long RECOV_TIME_MS = 6000; // 2.0 s

// ================= ISR encoder (đếm 2 sườn + lọc nhiễu) =================
void IRAM_ATTR encL_isr(){
  uint32_t now = micros();
  if (now - encL_last_us >= MIN_EDGE_US){
    encL_count++; encL_total++;
    encL_last_us = now;
  }
}
void IRAM_ATTR encR_isr(){
  uint32_t now = micros();
  if (now - encR_last_us >= MIN_EDGE_US){
    encR_count++; encR_total++;
    encR_last_us = now;
  }
}

// ================= Utils =================
inline int clamp255(int v){ return v<0?0:(v>255?255:v); }
inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
// Đổi thành (digitalRead(pin)==LOW) nếu TCRT của bạn xuất LOW khi ở trên line
inline bool onLine(int pin){ return digitalRead(pin) == LOW; }

inline bool isValidLineSample5(bool L2,bool L1,bool M,bool R1,bool R2){
  const int on = (int)L2 + L1 + M + R1 + R2;
  return !(on==0 || on==5);
}

static inline int shape_pwm(int target, int prev){
  int s = target;
  if (s>0 && s<PWM_MIN_RUN) s = PWM_MIN_RUN;
  if (s<0 && s>-PWM_MIN_RUN) s = -PWM_MIN_RUN;
  int d = s - prev;
  if (d >  PWM_SLEW) s = prev + PWM_SLEW;
  if (d < -PWM_SLEW) s = prev - PWM_SLEW;
  return clamp255(s);
}

/* ================= Motor control (DIR + PWM analogWrite) ================= */
// Với L298N: đặt chiều bằng INx, tốc độ bằng PWM trên ENA/ENB.
void driveWheelRight(float v_target, int pwm){
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

void driveWheelLeft(float v_target, int pwm){
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

void motorsStop(){
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// v đo được (m/s) từ số xung trong dt
float ticksToVel(long ticks, float dt_s){
  float rev = (float)ticks / (float)PPR_EFFECTIVE;
  return (rev * CIRC) / dt_s;
}

// PID tính PWM
int pidStep(PID &pid, float v_target, float v_meas, float dt_s){
  float err = v_target - v_meas;
  pid.i_term += pid.Ki * err * dt_s;
  pid.i_term = clampf(pid.i_term, pid.out_min, pid.out_max);
  float d = (err - pid.prev_err) / dt_s;
  float u = pid.Kp * err + pid.i_term + pid.Kd * d;
  pid.prev_err = err;
  return clamp255((int)u);
}

// ================= HC-SR04 (raw) =================
float readDistanceCM(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

// --- Median filter (3 mẫu) + reject giá trị vô lý ---
float readDistanceCM_filtered(){
  float a[3];
  for (int i = 0; i < 3; i++){
    float d = readDistanceCM();
    if (d < 2.0f || d > 200.0f) d = 999.0f;  // reject glitch
    a[i] = d;
    delay(2);                                // giảm delay
  }
  // sort 3 phần tử lấy median
  if (a[1] < a[0]) { float t=a[0]; a[0]=a[1]; a[1]=t; }
  if (a[2] < a[1]) { float t=a[1]; a[1]=a[2]; a[2]=t; }
  if (a[1] < a[0]) { float t=a[0]; a[0]=a[1]; a[1]=t; }
  return a[1];
}


// Cập nhật trạng thái obstacle với debounce + hysteresis
static inline void updateObstacleState(){
  if (millis() - us_last_ms < US_PERIOD_MS) return;
  us_last_ms = millis();

  us_dist_cm = readDistanceCM_filtered();
//   static unsigned long dbg_t=0;
// if (millis()-dbg_t>200){
//   dbg_t=millis();
//   Serial.print("dur=");
//   unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
//   Serial.print(dur);
//   Serial.print("  dist=");
//   Serial.print(us_dist_cm);
//   Serial.print("  latched=");
//   Serial.println(obs_latched);
// }


  if (!obs_latched){
    if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM){
      if (++obs_hit >= OBS_HIT_N) obs_latched = true;
    } else {
      obs_hit = 0;
    }
  } else {
    if (us_dist_cm >= OBSTACLE_OFF_CM){
      obs_latched = false;
      obs_hit = 0;
    }
  }
}

/* ================= Hình học encoder ================= */
long countsForDistance(double dist_m){
  double rot = dist_m / CIRC;
  return (long)(rot * PPR_EFFECTIVE + 0.5);
}

void motorWriteLR_signed(int pwmL, int pwmR){
  pwmL = pwmL < -255 ? -255 : (pwmL > 255 ? 255 : pwmL);
  pwmR = pwmR < -255 ? -255 : (pwmR > 255 ? 255 : pwmR);
  // Right
  if (pwmR >= 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, pwmR); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, -pwmR); }
  // Left
  if (pwmL >= 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, pwmL); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, -pwmL); }
}

// Tính góc từ xung (rad). signL/signR theo chiều lệnh của mỗi bánh.
static inline double theta_from_counts(long dL, long dR, int signL, int signR){
  double sL = (double)dL / PPR_EFFECTIVE * CIRC * signL;
  double sR = (double)dR / PPR_EFFECTIVE * CIRC * signR;
  return (sR - sL) / TRACK_WIDTH_M;
}

/* ================= Quay vòng kín theo góc ================= */
void spin_left_deg(double deg, int pwmMax){
  const double target = 1.5*deg * 3.141592653589793 / 180.0;
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0; // 1.5°
  const double Kp_theta = 140.0;   // PWM per rad (tune)
  const double Kbal = 0.6;         // cân bằng hai bánh (tune)
  const int pwmMin = 150;

  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();

  while (true){
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);

    double theta = theta_from_counts(dL, dR, -1, +1); // trái lùi, phải tiến
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;

    int pwmCap = (fabs(err) < 15.0*3.141592653589793/180.0) ? (pwmMax*0.5) : pwmMax;
    int base = (int)clamp255((int)(fabs(err)*Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);

    int corr = (int)(Kbal * ( (double)dR - (double)dL ));
    int pwmL = base - corr; if (pwmL < pwmMin) pwmL = pwmMin;
    int pwmR = base + corr; if (pwmR < pwmMin) pwmR = pwmMin;

    motorWriteLR_signed(-pwmL, +pwmR);
    delay(2);
  }
  motorsStop();
}

void spin_right_deg(double deg, int pwmMax){
  const double target = -1.5 * (deg * 3.141592653589793 / 180.0);  // <-- âm
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;                      // watchdog
  unsigned long t0 = millis();

  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();

  while (true){
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);

    // trái tiến (+1), phải lùi (-1) ⇒ theta âm, phù hợp target âm
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) break;                    // thoát an toàn

    int pwmCap = (fabs(err) < 15.0*3.141592653589793/180.0) ? (pwmMax*0.5) : pwmMax;
    int base = (int)clamp255((int)(fabs(err)*Kp_theta));
    base = base < pwmMin ? pwmMin : (base > pwmCap ? pwmCap : base);

    int corr = (int)(Kbal * ( (double)dL - (double)dR ));
    int pwmL = base + corr; if (pwmL < pwmMin) pwmL = pwmMin;
    int pwmR = base - corr; if (pwmR < pwmMin) pwmR = pwmMin;

    motorWriteLR_signed(+pwmL, -pwmR);
    delay(2);
  }
  motorsStop();
}

/* ================= Tiến theo quãng đường ================= */
void move_forward_distance(double dist_m, int pwmAbs){
  long target = countsForDistance(dist_m);
  long sL, sR; noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true){
    if (!g_line_enabled){ motorsStop(); return; }
    long cL, cR; noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
    bool left_done  = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done) break;
    if (left_done && !right_done)      motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done) motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
}

bool move_forward_distance_until_line(double dist_m, int pwmAbs){
  long target = countsForDistance(dist_m);
  long sL, sR; noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
  motorWriteLR_signed(+pwmAbs, +pwmAbs);
  while (true){
    if (!g_line_enabled){ motorsStop(); return false; }
    long cL, cR; noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
    if (digitalRead(M_SENSOR) == LOW){ motorsStop(); return true; }
    bool left_done  = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done) break;
    if (left_done && !right_done)      motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done) motorWriteLR_signed(+pwmAbs, 0);
    delay(1);
  }
  motorsStop();
  return false;
}

// implement API
void do_line_abort(){
  g_line_enabled = false;
  motorsStop();
}

void avoidObstacle(){
  const int TURN_PWM = 150; // chỉnh cho phù hợp
  const int FWD_PWM  = 100;

  spin_left_deg(40.0, TURN_PWM);
  motorsStop(); delay(500);

  move_forward_distance(0.2, FWD_PWM);
  motorsStop(); delay(500);

  spin_right_deg(40.0, TURN_PWM);
  motorsStop(); delay(500);

  move_forward_distance(0.15, FWD_PWM);
  motorsStop(); delay(500);

  spin_right_deg(50.0, TURN_PWM);
  motorsStop(); delay(500);

  bool seen = move_forward_distance_until_line(0.6, FWD_PWM);
  motorsStop(); delay(500);
  //if (seen) return;

  spin_left_deg(15.0, TURN_PWM);
  motorsStop(); delay(500);
}

/* ================= Setup → do_line_setup ================= */
void do_line_setup() {
//   Serial.begin(115200);
// delay(200);
// Serial.println("boot");

  // Motor DIR + PWM
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Sensors
  pinMode(L2_SENSOR, INPUT);
  pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR,  INPUT);
  pinMode(R1_SENSOR, INPUT);
  pinMode(R2_SENSOR, INPUT);

  // Encoders
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  g_line_enabled = true;
  seen_line_ever = false;
  vL_ema = 0.0f; vR_ema = 0.0f;
  pwmL_prev = 0; pwmR_prev = 0;

  // reset sonar state
  us_last_ms = 0;
  us_dist_cm = 999.0f;
  obs_hit = 0;
  obs_latched = false;

    // reset PID state  
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;

  motorsStop();
}

/* ================= Loop → do_line_loop ================= */
void do_line_loop() {
  if (!g_line_enabled) {
    motorsStop();
    return;
  }

  static unsigned long t_prev = millis();
  static unsigned long bad_t = 0; // chặn mẫu tất cả HIGH/LOW

  // ---- Đọc line (LOW = trên vạch) ----
  bool L2 = onLine(L2_SENSOR);
  bool L1 = onLine(L1_SENSOR);
  bool M  = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR);
  bool R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2) seen_line_ever = true;

  // ---- Chặn mẫu xấu (all 0 hoặc all 1) ----
  bool bad = (L2&&L1&&M&&R1&&R2) || (!L2&&!L1&&!M&&!R1&&!R2);
  if (bad) {
    if (millis() - bad_t > 1000) {
      recovering = false; avoiding = false;
      motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis();
      return;
    }
  } else {
    bad_t = millis();
  }

  // ---- Né vật cản (GIỮ NGUYÊN LUỒNG, chỉ thay cách đo sonar) ----
  bool line_follow_active = isValidLineSample5(L2,L1,M,R1,R2) && !recovering && !avoiding;

  // cập nhật sonar theo chu kỳ + lọc + debounce + hysteresis
  updateObstacleState();

  if (line_follow_active && obs_latched){
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

  // ---- Recovery theo thời gian khi mất line (giữ nguyên, chỉ dùng M) ----
  if (recovering) {
    if (L2 || L1 || M || R1 || R2) {   // M rời line nếu LOW=trên line
      recovering = false;
      motorsStop();
    } else if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false;
      motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis();
      return;
    } else {
      if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
      else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
      else                         { vL_tgt =  v_base; vR_tgt =  v_base; }
    }
  }

  // ---- Logic dò line 5 mắt ----
  if (!recovering){
    const int on_cnt = (int)L2 + L1 + M + R1 + R2;

    // 1) Thẳng
    if ( M && !L1 && !R1 && !L2 && !R2 ){
      last_seen = NONE; vL_tgt = v_base; vR_tgt = v_base;
    }
    // 2) Lệch trái nhẹ: L1 dính line (có thể kèm M)
    else if ( L1 &&  M && !R1 ){
      last_seen = LEFT; vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost;
    }
    else if ( L1 && !M && !L2 ){ // chỉ L1
      last_seen = LEFT; vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost;
    }
    // 3) Lệch trái mạnh: có L2 (outer-left) hoặc L1 mà M=0 và L2=1
    else if ( L2 && (!R1 && !R2) ){
      last_seen = LEFT; vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard;
    }
    // 4) Lệch phải nhẹ: R1 dính line (có thể kèm M)
    else if ( R1 &&  M && !L1 ){
      last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost;
    }
    else if ( R1 && !M && !R2 ){ // chỉ R1
      last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost;
    }
    // 5) Lệch phải mạnh: có R2 (outer-right)
    else if ( R2 && (!L1 && !L2) ){
      last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard;
    }
    // 6) Ngã ba/giao cắt: ≥3 mắt ON → chậm lại đi thẳng
    else if (on_cnt >= 3){
      last_seen = NONE; vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
    }
    // 7) Mất line hoàn toàn
    else {
      if (!seen_line_ever){ vL_tgt = 0; vR_tgt = 0; }
      else {
        recovering = true; rec_t0 = millis();
        if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
        else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
        else                         { vL_tgt =  v_base; vR_tgt =  v_base; }
      }
    }
  }

  // ---- Chu kỳ PID ----
  unsigned long now = millis();
  if (now - t_prev >= CTRL_DT_MS){
    float dt_s = (now - t_prev) / 1000.0f;
    t_prev = now;

    noInterrupts();
    long cL = encL_count; encL_count = 0;
    long cR = encR_count; encR_count = 0;
    interrupts();

    float vL_meas_inst = ticksToVel(cL, dt_s) * (vL_tgt >= 0 ? 1.0f : -1.0f);
    float vR_meas_inst = ticksToVel(cR, dt_s) * (vR_tgt >= 0 ? 1.0f : -1.0f);

    // EMA làm mượt tốc độ đo
    vL_ema = EMA_B*vL_ema + (1-EMA_B)*vL_meas_inst;
    vR_ema = EMA_B*vR_ema + (1-EMA_B)*vR_meas_inst;

    const float V_MAX = 1.5f;
    vL_tgt = clampf(vL_tgt, -V_MAX, V_MAX);
    vR_tgt = clampf(vR_tgt, -V_MAX, V_MAX);

    int pwmL = pidStep(pidL, vL_tgt, vL_ema, dt_s);
    int pwmR = pidStep(pidR, vR_tgt, vR_ema, dt_s);

    // Shaper PWM: deadband + slew
    int pwmL_cmd = shape_pwm((int)(1.0f*pwmL), pwmL_prev);
    int pwmR_cmd = shape_pwm((int)(1.0f*pwmR), pwmR_prev);
    pwmL_prev = pwmL_cmd; pwmR_prev = pwmR_cmd;

    if (vL_tgt >=0  && vR_tgt < 0) {
      pwmR_cmd = pwmL_cmd;
    }
    else if (vL_tgt < 0  && vR_tgt >= 0) {
      pwmL_cmd = pwmR_cmd;
    }
    driveWheelLeft (vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}
