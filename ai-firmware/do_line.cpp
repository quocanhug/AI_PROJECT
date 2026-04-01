#include <Arduino.h>
#include "do_line.h"

// ================= Extern từ main.ino (dùng cho MODE_DELIVERY & MODE_AI_ROUTE) =================
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

// ================= Tọa độ node trên bản đồ (dùng tính hướng rẽ) =================
const int node_coords[15][2] = {
  {30, 30}, {100, 30}, {170, 30}, {240, 30}, {310, 30},
  {30, 100}, {100, 100}, {170, 100}, {240, 100}, {310, 100},
  {30, 170}, {100, 170}, {170, 170}, {240, 170}, {310, 170}
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

// ================= Motor pins (ESP32 + L298N) =================
#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

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
#define MIN_EDGE_US 1500

// ================= HC-SR04 =================
#define TRIG_PIN 21
#define ECHO_PIN 19

const float OBSTACLE_ON_CM  = 15.0f;   // ★ Giảm 25→15: lưới 25-30cm, 25cm sẽ detect node kế tiếp
const float OBSTACLE_OFF_CM = 20.0f;   // ★ Giảm 30→20: hysteresis theo OBSTACLE_ON
const uint8_t OBS_HIT_N = 2;           // Cần 2 lần đọc liên tiếp mới latch
const unsigned long US_PERIOD_MS = 25; // Polling sonar mỗi 25ms
const unsigned long US_TIMEOUT   = 3000; // pulseIn timeout (µs) — ★ giảm từ 8000: tránh block loop quá lâu khi sonar lỗi

static unsigned long us_last_ms = 0;
float us_dist_cm = 999.0f;     // non-static: extern bởi ai-firmware.ino cho telemetry
static uint8_t obs_hit = 0;
static bool obs_latched = false;

// ================= Thông số cơ khí =================
const float WHEEL_RADIUS_M = 0.0325f;
extern const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M;  // extern: cho ai-firmware.ino truy cập
const float TRACK_WIDTH_M = 0.1150f;

// ================= Tham số điều khiển (GIÁ TRỊ GỐC ĐÃ CHẠY TỐT) =================
float v_base   = 0.4f;
float v_boost  = 0.15f;  
float v_hard   = 0.20f; 
float v_search = 0.2f;
float vF = v_base * 0.90f;

// PID (GIÁ TRỊ GỐC)
PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

const unsigned long CTRL_DT_MS = 10;
volatile long encL_count = 0, encR_count = 0, encL_total = 0, encR_total = 0;
volatile uint32_t encL_last_us=0, encR_last_us=0;
float vL_ema=0.0f, vR_ema=0.0f;  // ★ non-static: extern bởi main.ino cho telemetry
const float EMA_B = 0.7f;   // ★ Tăng từ 0.5: lọc mượt hơn, ít giật

const int PWM_MIN_RUN = 75;
const int PWM_SLEW    = 15;  // ★ Giảm từ 30: motor thay đổi từ từ hơn, xe chạy mượt
static int pwmL_prev=0, pwmR_prev=0;

// ★ BUG FIX: t_prev và bad_t phải là file-scope static để
//   do_line_setup() có thể reset chúng — nếu là local-static
//   bên trong do_line_loop() thì chỉ init 1 lần, sẽ sai từ route thứ 2.
static unsigned long t_prev = 0;
static unsigned long bad_t  = 0;

enum Side {NONE, LEFT, RIGHT};
Side last_seen = NONE;
bool seen_line_ever = false;
bool avoiding = false;
static volatile bool g_line_enabled = true;
bool recovering = false;
unsigned long rec_t0 = 0;
const unsigned long RECOV_TIME_MS = 3000;

// ★ Node cúôi cùng xe đứng trên giao lộ (trước khi tăng currentPathIndex)
int lastConfirmedNodeIdx = 0;

// ★ Debounce giao lộ: 500ms
const unsigned long INTERSECTION_DEBOUNCE_MS = 500;
unsigned long last_intersection_time = 0;

// ★ Initial turn: xoay xe chuẩn hướng khi nhận route mới
bool needs_initial_turn = false;

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
// ★ LOW = trên vạch (đúng với module TCRT của bạn)
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

// ================= Motor control =================
void driveWheelRight(float v_target, int pwm){
  int d = clamp255(abs(pwm));
  if (v_target >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, d); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, d); }
}

void driveWheelLeft(float v_target, int pwm){
  int d = clamp255(abs(pwm));
  if (v_target >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, d); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, d); }
}

void motorsStop(){
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

float ticksToVel(long ticks, float dt_s){ return ((float)ticks / (float)PPR_EFFECTIVE * CIRC) / dt_s; }

int pidStep(PID &pid, float v_target, float v_meas, float dt_s){
  // ★ BUG FIX C2: Guard dt_s để tránh D-term blow up khi timing glitch (dt_s ≈ 0)
  if (dt_s < 0.001f) dt_s = 0.001f;
  float err = v_target - v_meas;
  pid.i_term += pid.Ki * err * dt_s;
  pid.i_term = clampf(pid.i_term, pid.out_min, pid.out_max);
  float d = (err - pid.prev_err) / dt_s;
  float u = pid.Kp * err + pid.i_term + pid.Kd * d;
  pid.prev_err = err;
  return clamp255((int)u);
}

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

static inline void updateObstacleState(){
  if (millis() - us_last_ms < US_PERIOD_MS) return;
  us_last_ms = millis();
  us_dist_cm = readDistanceCM_filtered();
  if (!obs_latched){
    if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM){
      if (++obs_hit >= OBS_HIT_N) obs_latched = true;
    } else { obs_hit = 0; }
  } else {
    if (us_dist_cm >= OBSTACLE_OFF_CM){ obs_latched = false; obs_hit = 0; }
  }
}

// ================= Hình học encoder =================
long countsForDistance(double dist_m){ return (long)(dist_m / CIRC * PPR_EFFECTIVE + 0.5); }

inline void motorWriteLR_signed(int pwmL, int pwmR){
  pwmL = pwmL < -255 ? -255 : (pwmL > 255 ? 255 : pwmL);
  pwmR = pwmR < -255 ? -255 : (pwmR > 255 ? 255 : pwmR);
  if (pwmR >= 0){ digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, pwmR); }
  else           { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, -pwmR); }
  if (pwmL >= 0){ digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, pwmL); }
  else           { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, -pwmL); }
}

static inline double theta_from_counts(long dL, long dR, int signL, int signR){
  return ((double)dR / PPR_EFFECTIVE * CIRC * signR - (double)dL / PPR_EFFECTIVE * CIRC * signL) / TRACK_WIDTH_M;
}

// ================= Quay theo góc (trả bool: true=OK, false=timeout/abort) =================
bool spin_left_deg(double deg, int pwmMax){
  const double target = 1.5*deg * 3.141592653589793 / 180.0;
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;
  unsigned long t0 = millis();
  bool success = true;
  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  while (true){
    // ★ FIX: Kiểm tra abort mỗi vòng lặp — E-STOP có tác dụng ngay
    if (!g_line_enabled) { motorsStop(); return false; }
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, -1, +1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) { success = false; Serial.println("[SPIN] LEFT TIMEOUT!"); break; }
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
    // ★ FIX: Kiểm tra abort mỗi vòng lặp — E-STOP có tác dụng ngay
    if (!g_line_enabled) { motorsStop(); return false; }
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) { success = false; Serial.println("[SPIN] RIGHT TIMEOUT!"); break; }
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

// ================= Tiến theo quãng đường =================
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
    // ★ FIX: Check cả 3 mắt giữa (L1, M, R1) thay vì chỉ M
    if (digitalRead(M_SENSOR) == LOW || digitalRead(L1_SENSOR) == LOW || digitalRead(R1_SENSOR) == LOW){ motorsStop(); return true; }
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

void do_line_abort(){ g_line_enabled = false; motorsStop(); }

// ★ BUG FIX #2 — RESUME resets route:
//   do_line_setup() xóa hết state (lastConfirmedNodeIdx=0, currentPathIndex reset về 0)
//   → robot tưởng mình đang ở node đầu → quay sai hướng khi resume.
//
//   do_line_resume() chỉ tái kích hoạt mà KHÔNG đụng vào route state.
//   Nó reset currentPathIndex về lastConfirmedNodeIdx để needs_initial_turn
//   tính đúng hướng từ node robot đang đứng thực tế.
void do_line_resume() {
  g_line_enabled = true;
  seen_line_ever = true;   // Đừng trigger auto-search từ đầu
  recovering = false;
  avoiding = false;

  // Reset PID + motor state KHÔNG reset route state
  vL_ema = 0.0f; vR_ema = 0.0f;
  pwmL_prev = 0; pwmR_prev = 0;
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;

  // Tránh "lost line > 1s" ngay khi vừa resume
  t_prev = millis();
  bad_t  = millis();

  // ★ QUAN TRỌNG: currentPathIndex phải trỏ về lastConfirmedNodeIdx
  //   để needs_initial_turn đọc đúng "curNode = currentPath[currentPathIndex]"
  //   (node robot đang đứng, không phải node đang tiến tới)
  currentPathIndex = lastConfirmedNodeIdx;
  needs_initial_turn = true;  // Re-align hướng trước khi chạy tiếp
}

void avoidObstacle(){
  const int TURN_PWM = 150;
  const int FWD_PWM  = 100;
  // ★ Scale cho lưới nhỏ 25-30cm:
  //   Trước: fwd 20+15cm + until_line 60cm = 95cm (gần 4 ô!)
  //   Sau:   fwd 8+6cm + until_line 20cm = 34cm (~1.2 ô) — an toàn
  spin_left_deg(30.0, TURN_PWM); motorsStop(); delay(100);
  move_forward_distance(0.08, FWD_PWM); motorsStop(); delay(100);
  spin_right_deg(30.0, TURN_PWM); motorsStop(); delay(100);
  move_forward_distance(0.06, FWD_PWM); motorsStop(); delay(100);
  spin_right_deg(35.0, TURN_PWM); motorsStop(); delay(100);
  move_forward_distance_until_line(0.20, FWD_PWM); motorsStop(); delay(100);
  spin_left_deg(10.0, TURN_PWM); motorsStop(); delay(100);
}

// ================= Setup =================
void do_line_setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(L2_SENSOR, INPUT); pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR,  INPUT); pinMode(R1_SENSOR, INPUT); pinMode(R2_SENSOR, INPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);

  // ★ BUG FIX #1 — ISR Duplicate:
  //   Mỗi lần gọi do_line_setup() (route mới / RESUME) phải detach trước,
  //   nếu không ESP32 tích lũy nhiều ISR → encoder đếm bội → mọi góc xoay sai.
  detachInterrupt(digitalPinToInterrupt(ENC_L));
  detachInterrupt(digitalPinToInterrupt(ENC_R));
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);

  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  g_line_enabled = true; seen_line_ever = false;
  vL_ema = 0.0f; vR_ema = 0.0f; pwmL_prev = 0; pwmR_prev = 0;
  // ★ BUG FIX m1: us_last_ms = millis() (không phải 0) để tránh sonar block 30ms
  //   ngay vòng đầu tiên của do_line_loop() sau khi nhận route mới.
  us_last_ms = millis(); us_dist_cm = 999.0f; obs_hit = 0; obs_latched = false;
  pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
  last_intersection_time = 0;
  last_seen = NONE; recovering = false;
  lastConfirmedNodeIdx = 0;  // ★ Reset về đầu route
  needs_initial_turn = true; // ★ Đánh dấu cần xoay hướng khi loop bắt đầu

  // ★ BUG FIX #2 — Stale t_prev / bad_t:
  //   t_prev và bad_t phải được reset tại đây (là file-scope statics).
  //   Nếu không, khoảng cách millis() tính từ lần chạy trước sẽ >> 1000ms
  //   → do_line_loop() tưởng robot mất line quá 1s và dừng ngay lập tức.
  t_prev = millis();
  bad_t  = millis();

  // ★ FIX F3: Reset encoder total counters cho mỗi route mới
  //   Nếu không reset, dist_cm trong telemetry tích lũy qua nhiều chuyến → số vô nghĩa.
  //   noInterrupts() để tránh race với ISR đang ghi encL_total/encR_total.
  noInterrupts(); encL_total = 0; encR_total = 0; interrupts();

  motorsStop();
}

// ================= Main Loop =================
void do_line_loop() {
  if (!g_line_enabled) { motorsStop(); return; }

  // ★★★ INITIAL TURN: Xoay xe chuẩn hướng khi nhận route mới ★★★
  // Xử lý TRƯỚC khi đọc sensor, áp dụng cho:
  // - Xuất phát lần đầu
  // - Về kho (hướng ngược)
  // - Tiếp tục (resume route)
  // - Dynamic reroute (sau vật cản)
  // - Multi-target (đã giao xong 1 điểm, quay đi điểm tiếp)
  if (needs_initial_turn) {
    needs_initial_turn = false;

    if ((currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE)
        && pathLength >= 2 && currentPathIndex < pathLength - 1) {

      int curNode = currentPath[currentPathIndex];
      int nxtNode = currentPath[currentPathIndex + 1];
      int targetDir = getTargetDirection(curNode, nxtNode);

      Serial.printf("[INIT_TURN] node%d→node%d, dir %d→%d\n",
                    curNode, nxtNode, currentDir, targetDir);

      if (targetDir != -1 && targetDir != currentDir) {
        int diff = (targetDir - currentDir + 4) % 4;
        const int TURN_PWM = 160;
        bool turnOk = true;

        // ★ Dừng hẳn trước khi quay để xoay chính xác
        motorsStop(); delay(300);

        if (diff == 1) {
          turnOk = spin_left_deg(62.0, TURN_PWM);
          Serial.println("  >> INIT LEFT");
        } else if (diff == 3) {
          turnOk = spin_right_deg(62.0, TURN_PWM);
          Serial.println("  >> INIT RIGHT");
        } else if (diff == 2) {
          turnOk = spin_right_deg(115.0, TURN_PWM);
          Serial.println("  >> INIT U-TURN");
        }

        // ★ Dừng hẳn sau khi quay xong
        motorsStop(); delay(200);

        if (!turnOk) {
          Serial.println("  ❗ INIT TURN FAILED — dừng route");
          motorsStop();
          extern void wsBroadcast(const char*);
          // ★ BUG FIX #8 — String fragmentation
          char itfBuf[192];
          snprintf(itfBuf, sizeof(itfBuf),
            "{\"type\":\"OBSTACLE_DETECTED\","
            "\"robotNode\":%d,\"robotDir\":%d,"
            "\"reason\":\"TURN_FAILED\"}",
            curNode, currentDir);
          wsBroadcast(itfBuf);
          currentMode = MODE_MANUAL;
          is_auto_running = false;
          line_mode = false; do_line_abort(); return;
        }

        currentDir = targetDir;

        // Sau khi xoay xong, tiến tìm line
        move_forward_distance_until_line(0.10, 90);  // ★ PWM giảm 120→90: chậm hơn, dễ bắt line
      } else {
        Serial.println("  >> INIT STRAIGHT (hướng đã chuẩn)");
      }

      // Đã xử lý node đầu → tiến sang đoạn tiếp theo
      currentPathIndex++;
      last_intersection_time = millis();  // Chống trigger lại intersection
      seen_line_ever = true;              // Đã trên track → không cần auto-search
      bad_t = millis();  // ★ FIX: Reset lost-line timeout sau intersection/turn
      return;
    }
  }

  // t_prev và bad_t là file-scope statics — được reset trong do_line_setup()
  // (không khai báo lại tại đây)

  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M  = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2) seen_line_ever = true;

  // Mất line hoàn toàn (5 mắt OFF) > 400ms → vào recovery
  // ★ FIX: KHÔNG can thiệp khi đã đang recovering (recovery có timeout riêng 3s)
  bool lost_all = !L2 && !L1 && !M && !R1 && !R2;
  if (lost_all) {
    if (!seen_line_ever && is_auto_running) {
      // Đang bò tìm line → KHÔNG dừng, để code ở dưới xử lý
      bad_t = millis();  // Reset timeout
    } else if (recovering) {
      // ★ Đang recovery → để recovery handler xử lý, KHÔNG giết nó
      // (recovery có RECOV_TIME_MS = 3s timeout riêng)
    } else if (millis() - bad_t > 400) {
      // ★ Mất line quá 400ms → vào recovery thay vì chỉ dừng chết
      Serial.println("[LINE] Lost >400ms → entering RECOVERY");
      recovering = true; rec_t0 = millis();
      motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis(); return;
    }
  } else { bad_t = millis(); }

  updateObstacleState();

  // ===== AI Route: vật cản → dừng, báo Web =====
  // ★ BUG FIX #10 — Obstacle spam cooldown:
  //   Nếu vật cản vẫn còn sau khi web gửi route mới → trigger liên tục.
  //   Cooldown 2s giữa các lần báo obstacle.
  static unsigned long lastObsReportMs = 0;
  if (currentMode == MODE_AI_ROUTE && obs_latched && !avoiding) {
    if (millis() - lastObsReportMs < 2000) {
      // Trong cooldown → chỉ dừng motor, không báo web
      motorsStop();
      obs_latched = false;
      return;
    }
    lastObsReportMs = millis();
    motorsStop();
    obs_latched = false;  // ★ Xóa cờ ngay để không rớ vào lần sau
    // ★ Dùng lastConfirmedNodeIdx: node xe đứng thực tế
    int robotNode    = (lastConfirmedNodeIdx < pathLength) ? currentPath[lastConfirmedNodeIdx] : 0;
    int obstacleNode = (currentPathIndex    < pathLength) ? currentPath[currentPathIndex]    : robotNode;
    extern void wsBroadcast(const char*);
    // ★ BUG FIX #8 — String fragmentation: dùng char buffer cố định
    char obsBuf[256];
    snprintf(obsBuf, sizeof(obsBuf),
      "{\"type\":\"OBSTACLE_DETECTED\","
      "\"robotNode\":%d,\"obstacleNode\":%d,"
      "\"robotDir\":%d,\"current_step\":%d,"
      "\"distance_cm\":%.1f}",
      robotNode, obstacleNode, currentDir, lastConfirmedNodeIdx, us_dist_cm);
    wsBroadcast(obsBuf);
    Serial.printf("[AI_ROUTE] OBSTACLE! robot@node%d blocked@node%d\n", robotNode, obstacleNode);
    currentMode = MODE_MANUAL;
    is_auto_running = false;
    line_mode = false; do_line_abort(); return;
  }

  // ===== Các mode khác: tránh vật cản vật lý =====
  if (obs_latched && !avoiding && currentMode != MODE_AI_ROUTE) {
    avoiding = true; motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis(); avoidObstacle(); motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis(); avoiding = false; return;
  }
  if (avoiding) return;

  float vL_tgt = 0, vR_tgt = 0;

  // ============================================================
  // ★ GIAO LỘ: Cả L2 VÀ R2 đều thấy vạch ngang (chống nhận nhầm khi xe lệch)
  // ============================================================
  bool at_intersection = (L2 && R2);

  if (at_intersection) {

    // --- MODE_LINE_ONLY: đi thẳng qua giao lộ ---
    if (currentMode == MODE_LINE_ONLY) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();
        motorsStop(); delay(200);
        move_forward_distance(0.06, 120);
        if (!onLine(M_SENSOR) && !onLine(L1_SENSOR) && !onLine(R1_SENSOR)) {
          motorWriteLR_signed(130, -130);
          unsigned long t0 = millis();
          while(millis() - t0 < 3500) {
            if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR)) break;
            delay(5);
          }
          motorsStop();
        }
        last_seen = NONE; recovering = false;
        return;
      }
      vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
    }

    // --- MODE_DELIVERY / MODE_AI_ROUTE: xử lý node ---
    else if (currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) {
      if (millis() - last_intersection_time > INTERSECTION_DEBOUNCE_MS) {
        last_intersection_time = millis();

        if (pathLength > 0) {
          // ★ Kiểm tra đã đến đích chưa
          if (currentPathIndex >= pathLength - 1) {
            // ★ Nhích thêm 2cm để tâm xe đến đúng tâm node đích
            move_forward_distance(0.02, 90);
            motorsStop(); delay(200);
            Serial.printf("[AI_ROUTE] ARRIVED at node %d\n", currentPath[pathLength-1]);

            if (currentMode == MODE_DELIVERY) {
              gripOpen(); delay(1500); gripClose();
            } else {
              Serial.println("[AI_ROUTE] DESTINATION REACHED — COMPLETED");
              extern void wsBroadcast(const char*);
              // ★ BUG FIX #8 — String fragmentation: dùng char buffer
              char compBuf[128];
              snprintf(compBuf, sizeof(compBuf),
                "{\"type\":\"COMPLETED\",\"robotNode\":%d,\"robotDir\":%d}",
                currentPath[pathLength-1], currentDir);
              wsBroadcast(compBuf);
            }
            delivered_count++;
            currentMode = MODE_MANUAL;
            is_auto_running = false;
            line_mode = false; do_line_abort(); return;
          }

          // ★ Tính hướng rẽ
          int curNode = currentPath[currentPathIndex];
          int nxtNode = currentPath[currentPathIndex + 1];
          int targetDir = getTargetDirection(curNode, nxtNode);
          int diff = (targetDir != -1) ? (targetDir - currentDir + 4) % 4 : 0;
          Serial.printf("[NODE] path[%d]=node%d -> node%d, dir %d->%d (diff=%d)\n",
                        currentPathIndex, curNode, nxtNode, currentDir, targetDir, diff);

          if (diff == 0) {
            // ★ ĐI THẲNG: KHÔNG dừng, chỉ tiến centering nhỏ rồi tiếp tục PID bình thường
            lastConfirmedNodeIdx = currentPathIndex;  // ★ Lưu node hiện tại TRƯỚC khi tăng
            currentPathIndex++;
            currentDir = targetDir;
            move_forward_distance(0.03, 100);
            last_seen = NONE;
            // Không return → tiếp tục vòng lặp bình thường, PID dò line
          } else {
            // ★ RẼ / U-TURN: Dừng hẳn, centering, xoay, tìm line
            move_forward_distance(0.03, 100);
            motorsStop(); delay(300);

            const int TURN_PWM = 160;
            bool turnOk = true;
            if (diff == 1) { turnOk = spin_left_deg(62.0, TURN_PWM);  Serial.println("  >> LEFT"); }
            else if (diff == 3) { turnOk = spin_right_deg(62.0, TURN_PWM); Serial.println("  >> RIGHT"); }
            else if (diff == 2) { turnOk = spin_right_deg(125.0, TURN_PWM); Serial.println("  >> U-TURN"); }

            motorsStop(); delay(200);

            if (!turnOk) {
              Serial.println("  ❗ TURN FAILED — dừng route, báo Web");
              motorsStop();
              extern void wsBroadcast(const char*);
              int robotNode = currentPath[currentPathIndex];
              // ★ BUG FIX #8 — String fragmentation
              char tfBuf[192];
              snprintf(tfBuf, sizeof(tfBuf),
                "{\"type\":\"OBSTACLE_DETECTED\","
                "\"robotNode\":%d,\"robotDir\":%d,"
                "\"reason\":\"TURN_FAILED\"}",
                robotNode, currentDir);
              wsBroadcast(tfBuf);
              currentMode = MODE_MANUAL;
              is_auto_running = false;
              line_mode = false; do_line_abort(); return;
            }

            currentDir = targetDir;
            lastConfirmedNodeIdx = currentPathIndex;
            bool lineFound = move_forward_distance_until_line(0.10, 90);
            currentPathIndex++;
            if (!lineFound) {
              // ★ Không tìm được line sau khi rẻ/U-turn
              // Chuẩn hướng: quét rộng hơn (200ms trái + 200ms phải x2)
              Serial.println("  ⚠️ Line not found after turn — wide scan");
              bool foundWide = false;
              for (int sweep = 0; sweep < 2 && !foundWide; sweep++) {
                // Quét trái 200ms
                motorWriteLR_signed(-90, 90);
                unsigned long ts = millis();
                while (millis() - ts < 200) {
                  if (!g_line_enabled) { motorsStop(); return; }
                  if (digitalRead(L2_SENSOR)==LOW || digitalRead(L1_SENSOR)==LOW ||
                      digitalRead(M_SENSOR)==LOW  || digitalRead(R1_SENSOR)==LOW ||
                      digitalRead(R2_SENSOR)==LOW) { motorsStop(); foundWide = true; break; }
                  delay(5);
                }
                if (foundWide) break;
                motorsStop(); delay(50);
                // Quét phải 200ms
                motorWriteLR_signed(90, -90);
                ts = millis();
                while (millis() - ts < 200) {
                  if (!g_line_enabled) { motorsStop(); return; }
                  if (digitalRead(L2_SENSOR)==LOW || digitalRead(L1_SENSOR)==LOW ||
                      digitalRead(M_SENSOR)==LOW  || digitalRead(R1_SENSOR)==LOW ||
                      digitalRead(R2_SENSOR)==LOW) { motorsStop(); foundWide = true; break; }
                  delay(5);
                }
                motorsStop(); delay(50);
              }
              if (!foundWide) {
                // Vẫn không thấy → vào recovering
                recovering = true; rec_t0 = millis();
              }
            }
          }
        }
        bad_t = millis();  // ★ FIX: Reset lost-line timeout sau intersection/turn
        return;
      }
      // Debounce: trong thời gian debounce → chạy thẳng bình thường
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
    // ★ Dừng recovering khi BẤT KỲ cảm biến nào thấy line (kể cả L2/R2)
    if (L2 || L1 || M || R1 || R2) { recovering = false; motorsStop(); }
    else if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false; motorsStop();
      // ★ AI_ROUTE: lùi thẳng về node đã xác nhận, báo Web từ đúng node đó
      if ((currentMode == MODE_AI_ROUTE || currentMode == MODE_DELIVERY) && is_auto_running) {
        Serial.printf("[RECOV] Timeout! Lùi về node[%d]=%d\n",
                      lastConfirmedNodeIdx, currentPath[lastConfirmedNodeIdx]);
        {
          const int REV_PWM = 80;
          // ★ Giảm 30→12cm: 30cm = 1 ô lưới, lùi quá sẽ qua giao lộ trước!
          const long revTarget = countsForDistance(0.12);
          long sL, sR; noInterrupts(); sL = encL_total; sR = encR_total; interrupts();
          motorWriteLR_signed(-REV_PWM, -REV_PWM);
          while (true) {
            if (!g_line_enabled) { motorsStop(); break; }
            if (digitalRead(L2_SENSOR)==LOW || digitalRead(L1_SENSOR)==LOW ||
                digitalRead(M_SENSOR)==LOW  || digitalRead(R1_SENSOR)==LOW ||
                digitalRead(R2_SENSOR)==LOW) { motorsStop(); break; }
            long cL, cR; noInterrupts(); cL = encL_total; cR = encR_total; interrupts();
            if (labs(cL-sL) >= revTarget && labs(cR-sR) >= revTarget) { motorsStop(); break; }
            delay(5);
          }
        }
        motorsStop(); delay(200);
        currentPathIndex = lastConfirmedNodeIdx;
        extern void wsBroadcast(const char*);
        int robotNode = currentPath[lastConfirmedNodeIdx];
        int reportDir = (currentDir + 2) % 4;
        // ★ BUG FIX #8 — String fragmentation
        char llBuf[192];
        snprintf(llBuf, sizeof(llBuf),
          "{\"type\":\"OBSTACLE_DETECTED\","
          "\"robotNode\":%d,\"robotDir\":%d,"
          "\"reason\":\"LINE_LOST\"}",
          robotNode, reportDir);
        wsBroadcast(llBuf);
        currentDir = reportDir;
        currentMode = MODE_MANUAL;
        is_auto_running = false;
        line_mode = false; do_line_abort(); return;
      }
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis(); return;
    } else {
      // ★ Lùi vào điều hướng mạnh:
      //   last_seen==LEFT : đuôi xe cần vừa hướng về line (bên trái)
      //     → bánh trái lùi mạnh, bánh phải lùi chậm → đuôi vòng sang trái
      //   last_seen==RIGHT: ngược lại
      //   last_seen==NONE : đảo chiều scan theo 500ms tránh chỉ dao động tại chỗ
      if (last_seen == LEFT) {
        vL_tgt = -vF;          // bánh trái lùi mạnh
        vR_tgt = -vF * 0.2f;   // bánh phải lùi yếu → đuôi vòng trái bắt line
      } else if (last_seen == RIGHT) {
        vL_tgt = -vF * 0.2f;   // bánh trái lùi yếu
        vR_tgt = -vF;          // bánh phải lùi mạnh → đuôi vòng phải bắt line
      } else {
        // NONE: đổi hướng vòng mỗi 600ms để quét cả hai phía
        bool scanLeft = ((millis() - rec_t0) / 600) % 2 == 0;
        vL_tgt = scanLeft ? -vF        : -vF * 0.2f;
        vR_tgt = scanLeft ? -vF * 0.2f : -vF;
      }
    }
  }
  else {
   // ★ PID dò line: ưu tiên từ giữa → ngoài, mất hoàn toàn khi TẤT CẢ 5 mắt tắt
    // Mức 1: Thẳng hoàn toàn
    if      ( M && !L1 && !R1)              { last_seen = NONE;  vL_tgt = v_base;           vR_tgt = v_base; }
    // Mức 2: Lệch nhẹ (M + L1 hoặc M + R1)
    else if ( M &&  L1 && !R1)              { last_seen = LEFT;  vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost; }
    else if ( M &&  R1 && !L1)              { last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost; }
    // Mức 3: Lệch vừa (L1 hoặc R1 không có M)
    else if ( L1 && !M)                     { last_seen = LEFT;  vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R1 && !M)                     { last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    // Mức 4: Lệch mạnh (chỉ L2 hoặc R2, không có L1/M/R1) → dùng v_hard như có sẵn
    else if ( L2 && !L1 && !M && !R1 && !R2) { last_seen = LEFT;  vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R2 && !L1 && !M && !R1 && !L2) { last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    // Mức 5: Mất HOÀN TOÀN (cả 5 mắt đều tắt)
    else if (!L2 && !L1 && !M && !R1 && !R2) {
      if (!seen_line_ever && is_auto_running) {
        // Xe đặt trước line → bò chậm tới khi tìm thấy
        vL_tgt = v_search; vR_tgt = v_search;
      }
      else if (!seen_line_ever) { vL_tgt = 0; vR_tgt = 0; }
      else {
        // ★ Mất line hoàn toàn → vào recovering, lùi có điều hướng
        recovering = true; rec_t0 = millis();
        if      (last_seen == LEFT)  { vL_tgt = -v_base + v_boost; vR_tgt = -v_base - v_boost; }
        else if (last_seen == RIGHT) { vL_tgt = -v_base - v_boost; vR_tgt = -v_base + v_boost; }
        else                         { vL_tgt = -v_base * 0.7f;    vR_tgt = -v_base * 0.7f; }
      }
    }
    else { vL_tgt = v_base; vR_tgt = v_base; }
  }

  // ================= PID =================
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
    vL_ema = EMA_B*vL_ema + (1-EMA_B)*vL_meas_inst;
    vR_ema = EMA_B*vR_ema + (1-EMA_B)*vR_meas_inst;
    const float V_MAX = 1.5f;
    vL_tgt = clampf(vL_tgt, -V_MAX, V_MAX);
    vR_tgt = clampf(vR_tgt, -V_MAX, V_MAX);
    int pwmL = pidStep(pidL, vL_tgt, vL_ema, dt_s);
    int pwmR = pidStep(pidR, vR_tgt, vR_ema, dt_s);
    int pwmL_cmd = shape_pwm((int)(1.0f*pwmL), pwmL_prev);
    int pwmR_cmd = shape_pwm((int)(1.0f*pwmR), pwmR_prev);
    pwmL_prev = pwmL_cmd; pwmR_prev = pwmR_cmd;
    if (vL_tgt >=0  && vR_tgt < 0) { pwmR_cmd = pwmL_cmd; }
    else if (vL_tgt < 0  && vR_tgt >= 0) { pwmL_cmd = pwmR_cmd; }
    driveWheelLeft (vL_tgt, pwmL_cmd);
    driveWheelRight(vR_tgt, pwmR_cmd);
  }
}