#include <Arduino.h>
#include "do_line.h"

<<<<<<< Updated upstream
// ================= Extern từ main.ino (dùng cho MODE_DELIVERY & MODE_AI_ROUTE) =================
extern int currentPath[20];
=======
// ================= Extern từ main.ino =================
extern int currentPath[15];
>>>>>>> Stashed changes
extern int pathLength;
extern int currentPathIndex;
extern int currentDir;
extern volatile int delivered_count;
extern bool line_mode;
extern bool is_auto_running;
extern volatile UIMode currentMode;
extern void gripOpen();
extern void gripClose();

<<<<<<< Updated upstream
// ================= Tọa độ node trên bản đồ (dùng tính hướng rẽ) =================
const int node_coords[20][2] = {
=======
// ================= Tọa độ node trên bản đồ =================
const int node_coords[15][2] = {
>>>>>>> Stashed changes
  {30, 30}, {100, 30}, {170, 30}, {240, 30}, {310, 30},
  {30, 100}, {100, 100}, {170, 100}, {240, 100}, {310, 100},
  {30, 170}, {100, 170}, {170, 170}, {240, 170}, {310, 170},
  {30, 240}, {100, 240}, {170, 240}, {240, 240}, {310, 240}
};

int getTargetDirection(int nodeA, int nodeB) {
  int dx = node_coords[nodeB][0] - node_coords[nodeA][0];
  int dy = node_coords[nodeB][1] - node_coords[nodeA][1];
  if (dy > 0) return 0; // down
  if (dx > 0) return 1; // right
  if (dy < 0) return 2; // up
  if (dx < 0) return 3; // left
  return -1;
}

// ================= Motor pins & LEDC (ESP32) =================
#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

#define LEDC_CHANNEL_ENA 0
#define LEDC_CHANNEL_ENB 1
#define LEDC_FREQ 5000
#define LEDC_RES 8

// ================= Line sensors =================
#define L2_SENSOR   34
#define L1_SENSOR   32
#define M_SENSOR    33
#define R1_SENSOR   27
#define R2_SENSOR   25

// ================= Encoders =================
#define ENC_L 26
#define ENC_R 22
#define PULSES_PER_REV 20
#define PPR_EFFECTIVE (PULSES_PER_REV*3)
#define MIN_EDGE_US 500 // Giảm từ 1500 xuống 500 để tránh rớt xung ở tốc độ cao

// ================= HC-SR04 (Interrupt Driven) =================
#define TRIG_PIN 21
#define ECHO_PIN 19

<<<<<<< Updated upstream
const float OBSTACLE_TH_CM = 25.0f;  // ★ Tăng từ 20→25cm: thêm khoảng cách phanh
const unsigned long US_TIMEOUT = 8000;
=======
const float OBSTACLE_TH_CM = 25.0f;
volatile unsigned long echo_start_us = 0;
volatile unsigned long echo_dur_us = 0;
volatile bool echo_ready = false;
>>>>>>> Stashed changes

float us_dist_cm = 999.0f;
static unsigned long us_last_ms = 0;
static uint8_t obs_hit = 0;
static bool obs_latched = false;
const float OBSTACLE_ON_CM  = OBSTACLE_TH_CM;
<<<<<<< Updated upstream
const float OBSTACLE_OFF_CM = 30.0f;  // ★ Hysteresis: phải > 30cm mới xóa cờ vật cản
const uint8_t OBS_HIT_N = 2;
const unsigned long US_PERIOD_MS = 25;
=======
const float OBSTACLE_OFF_CM = 30.0f;
const uint8_t OBS_HIT_N = 3;
const unsigned long US_PERIOD_MS = 30; // Chu kỳ trigger sonar (tránh dội âm)

void IRAM_ATTR echo_isr() {
  if (digitalRead(ECHO_PIN) == HIGH) {
    echo_start_us = micros();
  } else {
    echo_dur_us = micros() - echo_start_us;
    echo_ready = true;
  }
}
>>>>>>> Stashed changes

// ================= Thông số cơ khí =================
const float WHEEL_RADIUS_M = 0.0325f;
extern const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M;
const float TRACK_WIDTH_M = 0.1150f;

// ================= Tham số điều khiển =================
float v_base   = 0.4f;
float v_boost  = 0.11f;
float v_hard   = 0.13f;
float v_search = 0.2f;

PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

const unsigned long CTRL_DT_US = 10000; // 10ms
volatile long encL_count = 0, encR_count = 0, encL_total = 0, encR_total = 0;
volatile uint32_t encL_last_us=0, encR_last_us=0;
<<<<<<< Updated upstream
float vL_ema=0.0f, vR_ema=0.0f;  // ★ non-static: extern bởi main.ino cho telemetry
const float EMA_B = 0.5f;   // ★ Giảm từ 0.7: phản ứng nhanh hơn khi lệch

const int PWM_MIN_RUN = 75;
const int PWM_SLEW    = 30;  // ★ Tăng từ 8: motor đáp ứng nhanh hơn khi sửa hướng
static int pwmL_prev=0, pwmR_prev=0;
=======
float vL_ema=0.0f, vR_ema=0.0f;
const float EMA_B = 0.85f;

const int PWM_MIN_RUN = 75;
const int PWM_SLEW    = 15;
static int pwmL_prev=0, pwmR_prev=0;
static unsigned long t_prev_us = 0;
static unsigned long bad_t  = 0;
>>>>>>> Stashed changes

enum Side {NONE, LEFT, RIGHT};
Side last_seen = NONE;
bool seen_line_ever = false;
bool avoiding = false;
static volatile bool g_line_enabled = true;
bool recovering = false;
unsigned long rec_t0 = 0;
<<<<<<< Updated upstream
const unsigned long RECOV_TIME_MS = 6000;
=======
const unsigned long RECOV_TIME_MS = 3500;

int lastConfirmedNodeIdx = 0;
const unsigned long INTERSECTION_DEBOUNCE_MS = 600;
unsigned long last_intersection_time = 0;
bool needs_initial_turn = false;
>>>>>>> Stashed changes

// ================= ISR encoder =================
void IRAM_ATTR encL_isr(){
  uint32_t now = micros();
  if (now - encL_last_us >= MIN_EDGE_US){ encL_count++; encL_total++; encL_last_us = now; }
}
void IRAM_ATTR encR_isr(){
  uint32_t now = micros();
  if (now - encR_last_us >= MIN_EDGE_US){ encR_count++; encR_total++; encR_last_us = now; }
}

// ================= Utils =================
inline int clamp255(int v){ return v<0?0:(v>255?255:v); }
inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
inline bool onLine(int pin){ return digitalRead(pin) == LOW; }

static inline int shape_pwm(int target, int prev){
  int s = target;
  if (s>0 && s<PWM_MIN_RUN) s = PWM_MIN_RUN;
  if (s<0 && s>-PWM_MIN_RUN) s = -PWM_MIN_RUN;
  int d = s - prev;
  if (d >  PWM_SLEW) s = prev + PWM_SLEW;
  if (d < -PWM_SLEW) s = prev - PWM_SLEW;
  return clamp255(s);
}

void driveWheelRight(float v_target, int pwm){
  int d = clamp255(abs(pwm));
  if (v_target >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); ledcWrite(LEDC_CHANNEL_ENB, d); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); ledcWrite(LEDC_CHANNEL_ENB, d); }
}

void driveWheelLeft(float v_target, int pwm){
  int d = clamp255(abs(pwm));
  if (v_target >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); ledcWrite(LEDC_CHANNEL_ENA, d); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); ledcWrite(LEDC_CHANNEL_ENA, d); }
}

void motorsStop(){
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  ledcWrite(LEDC_CHANNEL_ENA, 0); ledcWrite(LEDC_CHANNEL_ENB, 0);
}

float ticksToVel(long ticks, float dt_s){ return ((float)ticks / (float)PPR_EFFECTIVE * CIRC) / dt_s; }

int pidStep(PID &pid, float v_target, float v_meas, float dt_s){
<<<<<<< Updated upstream
=======
  if (dt_s < 0.001f) dt_s = 0.001f;
>>>>>>> Stashed changes
  float err = v_target - v_meas;
  pid.i_term += pid.Ki * err * dt_s;
  pid.i_term = clampf(pid.i_term, pid.out_min, pid.out_max);
  float d = (err - pid.prev_err) / dt_s;
  float u = pid.Kp * err + pid.i_term + pid.Kd * d;
  pid.prev_err = err;
  return clamp255((int)u);
}

<<<<<<< Updated upstream
// ================= HC-SR04 =================
float readDistanceCM(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return -1;
  return dur * 0.0343f / 2.0f;
}

static float readDistanceCM_filtered(){
  float a[3];
  for (int i = 0; i < 3; i++){
    float d = readDistanceCM();
    if (d < 2.0f || d > 200.0f) d = 999.0f;
    a[i] = d;
    delay(2);
  }
  if (a[1] < a[0]) { float t=a[0]; a[0]=a[1]; a[1]=t; }
  if (a[2] < a[1]) { float t=a[1]; a[1]=a[2]; a[2]=t; }
  if (a[1] < a[0]) { float t=a[0]; a[0]=a[1]; a[1]=t; }
  return a[1];
}

=======
// Đọc Sonar Non-Blocking
>>>>>>> Stashed changes
static inline void updateObstacleState(){
  unsigned long now = millis();
  if (now - us_last_ms >= US_PERIOD_MS) {
    us_last_ms = now;
    
    // Tính khoảng cách từ ngắt lần trước
    if (echo_ready) {
      if (echo_dur_us > 0 && echo_dur_us < 20000) { 
        us_dist_cm = echo_dur_us * 0.0343f / 2.0f;
      } else {
        us_dist_cm = 999.0f;
      }
    } else {
      us_dist_cm = 999.0f; // Timeout
    }

    // Trigger lần đo mới
    echo_ready = false;
    digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Cập nhật State Machine vật cản
    if (!obs_latched){
      if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM){
        if (++obs_hit >= OBS_HIT_N) obs_latched = true;
      } else { obs_hit = 0; }
    } else {
      if (us_dist_cm >= OBSTACLE_OFF_CM){ obs_latched = false; obs_hit = 0; }
    }
  }
}

long countsForDistance(double dist_m){ return (long)(dist_m / CIRC * PPR_EFFECTIVE + 0.5); }

inline void motorWriteLR_signed(int pwmL, int pwmR){
  pwmL = pwmL < -255 ? -255 : (pwmL > 255 ? 255 : pwmL);
  pwmR = pwmR < -255 ? -255 : (pwmR > 255 ? 255 : pwmR);
  if (pwmR >= 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  ledcWrite(LEDC_CHANNEL_ENB, pwmR); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); ledcWrite(LEDC_CHANNEL_ENB, -pwmR); }
  if (pwmL >= 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  ledcWrite(LEDC_CHANNEL_ENA, pwmL); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); ledcWrite(LEDC_CHANNEL_ENA, -pwmL); }
}

static inline double theta_from_counts(long dL, long dR, int signL, int signR){
  return ((double)dR / PPR_EFFECTIVE * CIRC * signR - (double)dL / PPR_EFFECTIVE * CIRC * signL) / TRACK_WIDTH_M;
}

bool spin_left_deg(double deg, int pwmMax){
  const double target = 1.5*deg * 3.141592653589793 / 180.0;
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 110;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  
  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  
  while (true){
    if (!g_line_enabled) { motorsStop(); return false; }
    
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, -1, +1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) { success = false; break; }
    
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
  return success;
}

bool spin_right_deg(double deg, int pwmMax){
  const double target = -1.5 * (deg * 3.141592653589793 / 180.0);
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  
  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  
  while (true){
    if (!g_line_enabled) { motorsStop(); return false; }
    
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) { success = false; break; }
    
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
  return success;
}

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
<<<<<<< Updated upstream
    if (digitalRead(M_SENSOR) == LOW){ motorsStop(); return true; }
    bool left_done  = (labs(cL - sL) >= target);
    bool right_done = (labs(cR - sR) >= target);
    if (left_done && right_done) break;
    if (left_done && !right_done)      motorWriteLR_signed(0, +pwmAbs);
    else if (!left_done && right_done) motorWriteLR_signed(+pwmAbs, 0);
=======
    if (digitalRead(M_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW){ motorsStop(); return true; }
    if (labs(cL - sL) >= target && labs(cR - sR) >= target) break;
>>>>>>> Stashed changes
    delay(1);
  }
  motorsStop();
  return false;
}

void do_line_abort(){ g_line_enabled = false; motorsStop(); }

<<<<<<< Updated upstream
=======
void do_line_resume() {
  g_line_enabled = true;
  seen_line_ever = true;   
  recovering = false;
  avoiding = false;
  vL_ema = 0.0f; vR_ema = 0.0f;
  pwmL_prev = 0; pwmR_prev = 0;
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;
  t_prev_us = micros();
  bad_t  = millis();
  currentPathIndex = lastConfirmedNodeIdx;
  needs_initial_turn = true;  
}

>>>>>>> Stashed changes
void avoidObstacle(){
  const int TURN_PWM = 150;
  const int FWD_PWM  = 100;
  spin_left_deg(40.0, TURN_PWM); motorsStop(); delay(500);
  move_forward_distance(0.2, FWD_PWM); motorsStop(); delay(500);
  spin_right_deg(40.0, TURN_PWM); motorsStop(); delay(500);
  move_forward_distance(0.15, FWD_PWM); motorsStop(); delay(500);
  spin_right_deg(50.0, TURN_PWM); motorsStop(); delay(500);
  move_forward_distance_until_line(0.6, FWD_PWM); motorsStop(); delay(500);
  spin_left_deg(15.0, TURN_PWM); motorsStop(); delay(500);
}

void do_line_setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // Khởi tạo LEDC PWM Hardware Timer thay cho analogWrite
  ledcSetup(LEDC_CHANNEL_ENA, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(ENA, LEDC_CHANNEL_ENA);
  ledcSetup(LEDC_CHANNEL_ENB, LEDC_FREQ, LEDC_RES);
  ledcAttachPin(ENB, LEDC_CHANNEL_ENB);

  pinMode(L2_SENSOR, INPUT); pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR,  INPUT); pinMode(R1_SENSOR, INPUT); pinMode(R2_SENSOR, INPUT);
  
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
<<<<<<< Updated upstream
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);
=======
  detachInterrupt(digitalPinToInterrupt(ENC_L));
  detachInterrupt(digitalPinToInterrupt(ENC_R));
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);
  
  // Khởi tạo ngắt cho Sonar
>>>>>>> Stashed changes
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echo_isr, CHANGE);

  g_line_enabled = true; seen_line_ever = false;
  vL_ema = 0.0f; vR_ema = 0.0f; pwmL_prev = 0; pwmR_prev = 0;
<<<<<<< Updated upstream
  us_last_ms = 0; us_dist_cm = 999.0f; obs_hit = 0; obs_latched = false;
  pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
=======
  us_last_ms = millis(); us_dist_cm = 999.0f; obs_hit = 0; obs_latched = false;
  pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
  last_intersection_time = 0;
  last_seen = NONE; recovering = false;
  lastConfirmedNodeIdx = 0;  
  needs_initial_turn = true; 
  t_prev_us = micros();
  bad_t  = millis();
>>>>>>> Stashed changes
  motorsStop();
}

void do_line_loop() {
  if (!g_line_enabled) { motorsStop(); return; }

<<<<<<< Updated upstream
  static unsigned long t_prev = millis();
  static unsigned long bad_t = millis();
  static unsigned long last_intersection_time = 0;
=======
  if (needs_initial_turn) {
    needs_initial_turn = false;
    if ((currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE)
        && pathLength >= 2 && currentPathIndex < pathLength - 1) {
      int curNode = currentPath[currentPathIndex];
      int nxtNode = currentPath[currentPathIndex + 1];
      int targetDir = getTargetDirection(curNode, nxtNode);
      if (targetDir != -1 && targetDir != currentDir) {
        int diff = (targetDir - currentDir + 4) % 4;
        const int TURN_PWM = 160;
        bool turnOk = true;
        motorsStop(); delay(400); 
        if (diff == 1) turnOk = spin_left_deg(62.0, TURN_PWM);
        else if (diff == 3) turnOk = spin_right_deg(62.0, TURN_PWM);
        else if (diff == 2) turnOk = spin_right_deg(115.0, TURN_PWM);
        motorsStop(); delay(300); 
        if (!turnOk) {
          is_auto_running = false; line_mode = false; do_line_abort(); return;
        }
      }
      bool lineFoundAfterTurn = true;
      if (targetDir != -1 && targetDir != currentDir) {
        lineFoundAfterTurn = move_forward_distance_until_line(0.12, 95);
      }
      if (lineFoundAfterTurn) currentPathIndex++;
      last_intersection_time = millis();  
      seen_line_ever = true;              
      return;
    }
  }
>>>>>>> Stashed changes

  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M  = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2) seen_line_ever = true;

  bool lost_all = !L2 && !L1 && !M && !R1 && !R2;
  if (lost_all) {
    if (!seen_line_ever && is_auto_running) {
      bad_t = millis(); 
    } else if (millis() - bad_t > 1000) {
        if (!recovering) { recovering = true; rec_t0 = millis(); }
    }
  } else { bad_t = millis(); }

  updateObstacleState();

  if (currentMode == MODE_AI_ROUTE && obs_latched && !avoiding) {
<<<<<<< Updated upstream
    motorsStop();
    int robotNode = (currentPathIndex < pathLength) ? currentPath[currentPathIndex] : 0;
    int obstacleNode = (currentPathIndex + 1 < pathLength) ? currentPath[currentPathIndex + 1] : robotNode;
    extern void wsBroadcast(const char*);
    String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                 "\"robotNode\":" + String(robotNode) + ","
                 "\"obstacleNode\":" + String(obstacleNode) + ","
                 "\"robotDir\":" + String(currentDir) + ","
                 "\"current_step\":" + String(currentPathIndex) + ","
                 "\"distance_cm\":" + String(us_dist_cm, 1) + "}";
    wsBroadcast(msg.c_str());
    Serial.printf("[AI_ROUTE] OBSTACLE! robot@node%d blocked@node%d\n", robotNode, obstacleNode);
    currentMode = MODE_MANUAL;
    is_auto_running = false;
=======
    motorsStop(); obs_latched = false;
    currentMode = MODE_MANUAL; is_auto_running = false;
>>>>>>> Stashed changes
    line_mode = false; do_line_abort(); return;
  }

  if (obs_latched && !avoiding && currentMode != MODE_AI_ROUTE) {
    avoiding = true; motorsStop(); avoidObstacle(); avoiding = false; return;
  }
  if (avoiding) return;

  float vL_tgt = 0, vR_tgt = 0;

<<<<<<< Updated upstream
  // ============================================================
  // ★ GIAO LỘ: L2 hoặc R2 phát hiện line ngang (perpendicular)
  // ============================================================
  bool at_intersection = (L2 || R2);

=======
  // ================= GIAO LỘ =================
  bool at_intersection = (L2 && R2 && (L1 || M || R1));
>>>>>>> Stashed changes
  if (at_intersection) {
    if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
      last_intersection_time = millis();
      motorsStop(); delay(400); 

<<<<<<< Updated upstream
    // --- MODE_LINE_ONLY: đi thẳng qua giao lộ ---
    if (currentMode == MODE_LINE_ONLY) {
      if (millis() - last_intersection_time > 1500) {
        last_intersection_time = millis();
        motorsStop(); delay(200);
=======
      if (currentMode == MODE_LINE_ONLY) {
>>>>>>> Stashed changes
        move_forward_distance(0.06, 120);
        last_seen = NONE; recovering = false;
        return;
      }
<<<<<<< Updated upstream
      vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
    }

    // --- MODE_DELIVERY / MODE_AI_ROUTE: xử lý node ---
    else if (currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) {
      if (millis() - last_intersection_time > 1500) {
        last_intersection_time = millis();
        motorsStop(); delay(200);

        // ★ FIX: Tiến ~5cm vào TÂM giao lộ TRƯỚC khi xoay
        // Lý do: L2/R2 nằm ở đầu xe, nhưng trục quay nằm giữa 2 bánh
        // Nếu xoay ngay → trục quay chưa ở tâm → xe lệch khỏi line sau rẽ
        move_forward_distance(0.02, 100);
        motorsStop(); delay(100);

=======
      else if (currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) {
>>>>>>> Stashed changes
        if (pathLength > 0) {
          if (currentPathIndex >= pathLength - 1) {
<<<<<<< Updated upstream
            motorsStop();
            if (currentMode == MODE_DELIVERY) {
              gripOpen(); delay(1500); gripClose();
            } else {
              Serial.println("[AI_ROUTE] DESTINATION REACHED");
              extern void wsBroadcast(const char*);
              String msg = "{\"type\":\"COMPLETED\",\"robotNode\":" + String(currentPath[pathLength-1]) +
                           ",\"robotDir\":" + String(currentDir) + "}";
              wsBroadcast(msg.c_str());
            }
            delivered_count++;
            currentMode = MODE_MANUAL;
            is_auto_running = false;
            line_mode = false; do_line_abort(); return;
          }

          // ★ FIX: Tính hướng rẽ TRƯỚC khi tăng index
          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          Serial.printf("[NODE] path[%d]=node%d -> node%d, dir %d->%d\n",
                        currentPathIndex, curNode, nxtNode, currentDir, targetDir);

          if (targetDir != -1) {
            int diff = (targetDir - currentDir + 4) % 4;
            const int TURN_PWM = 160;
            bool turnOk = true;
            // ★ C++ dirs 0=down,1=right,2=up,3=left (non-CW order)
            if (diff == 1) { turnOk = spin_left_deg(62.0, TURN_PWM); Serial.println("  >> LEFT"); }
            else if (diff == 3) { turnOk = spin_right_deg(62.0, TURN_PWM); Serial.println("  >> RIGHT"); }
            else if (diff == 2) { turnOk = spin_right_deg(125.0, TURN_PWM); Serial.println("  >> U-TURN"); }
            else { Serial.println("  >> STRAIGHT"); }

            // ★ FIX: Xử lý spin thất bại (timeout hoặc abort)
=======
            move_forward_distance(0.02, 90); motorsStop();
            if (currentMode == MODE_DELIVERY) { gripOpen(); delay(1500); gripClose(); }
            delivered_count++; currentMode = MODE_MANUAL; is_auto_running = false;
            line_mode = false; do_line_abort(); return;
          }

          move_forward_distance(0.10, 100); 
          motorsStop(); delay(300);

          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          int diff = (targetDir != -1) ? (targetDir - currentDir + 4) % 4 : 0;

          if (diff == 0) {
            lastConfirmedNodeIdx = currentPathIndex;
            currentPathIndex++;
            currentDir = targetDir;
            last_seen = NONE;
          } else {
            const int TURN_PWM_90 = 145; 
            const int TURN_PWM_180 = 180;
            bool turnOk = true;
            if (diff == 1) turnOk = spin_left_deg(62.0, TURN_PWM_90);
            else if (diff == 3) turnOk = spin_right_deg(62.0, TURN_PWM_90);
            else if (diff == 2) turnOk = spin_right_deg(125.0, TURN_PWM_90);

            motorsStop(); delay(300); 

>>>>>>> Stashed changes
            if (!turnOk) {
              currentMode = MODE_MANUAL; is_auto_running = false;
              line_mode = false; do_line_abort(); return;
            }
            currentDir = targetDir;
<<<<<<< Updated upstream
=======
            lastConfirmedNodeIdx = currentPathIndex;
            move_forward_distance_until_line(0.12, 90);
            currentPathIndex++;
>>>>>>> Stashed changes
          }

          // ★ Tăng index SAU khi đã rẽ
          currentPathIndex++;
          // ★ Tìm line thông minh: tiến tối đa 10cm, dừng ngay khi M chạm vạch
          move_forward_distance_until_line(0.10, 120);
        }
        return;
      }
<<<<<<< Updated upstream
      // Debounce: chạy chậm qua vùng giao lộ
      last_seen = NONE;
      vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
    }

    // --- Các mode khác ---
    else {
      vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
    }
  }

  // ============================================================
  // ★ DÒ LINE: CHỈ dùng 3 mắt giữa L1, M, R1
  // ============================================================
  else if (recovering) {
    if (L1 || M || R1) { recovering = false; motorsStop(); }
    else if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false; motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis(); return;
    } else {
      if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
      else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
      else                         { vL_tgt = v_base; vR_tgt = v_base; }
    }
  }
  else {
    // ★ PID dò line: CHỈ 3 mắt giữa
    if      ( M && !L1 && !R1) { last_seen = NONE;  vL_tgt = v_base;           vR_tgt = v_base; }
    else if ( L1 &&  M && !R1) { last_seen = LEFT;  vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost; }
    else if ( L1 && !M       ) { last_seen = LEFT;  vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R1 &&  M && !L1) { last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost; }
    else if ( R1 && !M       ) { last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    else if (!L1 && !M && !R1) {
      // Mất line 3 mắt giữa
      if (!seen_line_ever && is_auto_running) {
        // ★ FIX: Xe đặt trước line → bò chậm tới khi tìm thấy
        vL_tgt = v_search; vR_tgt = v_search;
      }
      else if (!seen_line_ever) { vL_tgt = 0; vR_tgt = 0; }
      else {
        recovering = true; rec_t0 = millis();
        if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
        else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
        else                         { vL_tgt = v_base; vR_tgt = v_base; }
      }
=======
    }
  }

  // ================= ĐIỀU KHIỂN DÒ LINE / RECOVERING =================
  if (recovering) {
    if (L2 || L1 || M || R1 || R2) { 
      recovering = false; motorsStop(); delay(100); 
    }
    else if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false; motorsStop();
      if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) && is_auto_running) {
        motorWriteLR_signed(-90, -90);
        delay(800); motorsStop();
        currentPathIndex = lastConfirmedNodeIdx;
        needs_initial_turn = true; 
        currentMode = MODE_MANUAL; is_auto_running = false;
        line_mode = false; do_line_abort(); return;
      }
    } else {
      float v_rev = -v_base * 0.65f;
      if (last_seen == LEFT) { vL_tgt = v_rev; vR_tgt = v_rev * 0.4f; }
      else if (last_seen == RIGHT) { vL_tgt = v_rev * 0.4f; vR_tgt = v_rev; }
      else { vL_tgt = v_rev; vR_tgt = v_rev; }
    }
  }
  else {
    if      ( M && !L1 && !R1)              { last_seen = NONE;  vL_tgt = v_base;           vR_tgt = v_base; }
    else if ( M &&  L1 && !R1)              { last_seen = LEFT;  vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost; }
    else if ( M &&  R1 && !L1)              { last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost; }
    else if ( L1 && !M)                     { last_seen = LEFT;  vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R1 && !M)                     { last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    else if ( L2)                           { last_seen = LEFT;  vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R2)                           { last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    else if (lost_all) {
      if (!seen_line_ever && is_auto_running) { vL_tgt = v_search; vR_tgt = v_search; }
      else { recovering = true; rec_t0 = millis(); }
>>>>>>> Stashed changes
    }
    else { vL_tgt = v_base; vR_tgt = v_base; }
  }

  // ================= THỰC THI PID (Sử dụng micros) =================
  unsigned long now_us = micros();
  if (now_us - t_prev_us >= CTRL_DT_US) {
    float dt_s = (now_us - t_prev_us) / 1000000.0f;
    t_prev_us = now_us;
    
    long cL, cR;
    noInterrupts();
    cL = encL_count; encL_count = 0;
    cR = encR_count; encR_count = 0;
    interrupts();
    
    float vL_meas_inst = ticksToVel(cL, dt_s) * (vL_tgt >= 0 ? 1.0f : -1.0f);
    float vR_meas_inst = ticksToVel(cR, dt_s) * (vR_tgt >= 0 ? 1.0f : -1.0f);
    
    vL_ema = EMA_B*vL_ema + (1-EMA_B)*vL_meas_inst;
    vR_ema = EMA_B*vR_ema + (1-EMA_B)*vR_meas_inst;
    
    int pwmL = pidStep(pidL, vL_tgt, vL_ema, dt_s);
    int pwmR = pidStep(pidR, vR_tgt, vR_ema, dt_s);
    
    int pwmL_cmd = shape_pwm(pwmL, pwmL_prev);
    int pwmR_cmd = shape_pwm(pwmR, pwmR_prev);
    
    pwmL_prev = pwmL_cmd; pwmR_prev = pwmR_cmd;
    driveWheelLeft (vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}