#include <Arduino.h>
#include "do_line.h"

extern int currentPath[20];
extern int pathLength;
extern int currentPathIndex;
extern int currentDir;
extern volatile int delivered_count;
extern bool line_mode;
extern volatile UIMode currentMode; 
extern void gripOpen();
extern void gripClose();

// CẤU HÌNH MỨC LOGIC CẢM BIẾN
#define LINE_DETECT_STATE HIGH 
inline bool onLine(int pin){ return digitalRead(pin) == LINE_DETECT_STATE; }

const int node_coords[20][2] = {
  {30, 30}, {100, 30}, {170, 30}, {240, 30}, {310, 30},
  {30, 100}, {100, 100}, {170, 100}, {240, 100}, {310, 100},
  {30, 170}, {100, 170}, {170, 170}, {240, 170}, {310, 170},
  {30, 240}, {100, 240}, {170, 240}, {240, 240}, {310, 240}
};

int getTargetDirection(int nodeA, int nodeB) {
  int dx = node_coords[nodeB][0] - node_coords[nodeA][0];
  int dy = node_coords[nodeB][1] - node_coords[nodeA][1];
  if (dy > 0) return 0; 
  if (dx > 0) return 1; 
  if (dy < 0) return 2; 
  if (dx < 0) return 3; 
  return -1;
}

#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

#define L2_SENSOR   34   
#define L1_SENSOR   32   
#define M_SENSOR    33   
#define R1_SENSOR   27   
#define R2_SENSOR   25   

#define ENC_L 26
#define ENC_R 22
#define PULSES_PER_REV 20           
#define PPR_EFFECTIVE (PULSES_PER_REV*3) 
#define MIN_EDGE_US 1500

// ================= HC-SR04 TỐI ƯU HÓA TỐC ĐỘ (KHÔNG DELAY) =================
#define TRIG_PIN 21
#define ECHO_PIN 19
const float OBSTACLE_TH_CM = 20.0f;     
const unsigned long US_TIMEOUT = 6000; // Ép timeout ngắn để không lag vòng lặp PID

static unsigned long us_last_ms = 0;
static float us_dist_cm = 999.0f;
static float us_ema = 999.0f; 
static uint8_t obs_hit = 0;
static bool obs_latched = false;

const float OBSTACLE_ON_CM  = OBSTACLE_TH_CM; 
const float OBSTACLE_OFF_CM = 25.0f;          
const uint8_t OBS_HIT_N = 2;               

float readDistanceCM_Fast(){
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, US_TIMEOUT);
  if (dur == 0) return 999.0f;
  return dur * 0.0343f / 2.0f;
}

static inline void updateObstacleState(){
  if (millis() - us_last_ms < 40) return; // Chỉ quét siêu âm 25Hz để nhẹ CPU
  us_last_ms = millis();
  
  float d = readDistanceCM_Fast();
  if (d < 2.0f) d = 999.0f; 
  
  us_ema = 0.6f * us_ema + 0.4f * d; 
  us_dist_cm = us_ema;

  if (!obs_latched){
    if (us_dist_cm > 0 && us_dist_cm <= OBSTACLE_ON_CM){
      if (++obs_hit >= OBS_HIT_N) obs_latched = true;
    } else { obs_hit = 0; }
  } else {
    if (us_dist_cm >= OBSTACLE_OFF_CM){ obs_latched = false; obs_hit = 0; }
  }
}
// =========================================================================

const float WHEEL_RADIUS_M = 0.0325f;
const float CIRC = 2.0f * 3.1415926f * WHEEL_RADIUS_M; 
const float TRACK_WIDTH_M = 0.1150f; 

float v_base   = 0.4f;    
float v_boost  = 0.11f;   
float v_hard   = 0.13f;   
float vF = v_base * 0.90f;

// PID struct is defined in do_line.h
PID pidL{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};
PID pidR{300.0f, 8.0f, 0.00f, 0, 0, 0, 255};

const unsigned long CTRL_DT_MS = 10;
volatile long encL_count = 0, encR_count = 0, encL_total = 0, encR_total = 0;
volatile uint32_t encL_last_us=0, encR_last_us=0;
static float vL_ema=0.0f, vR_ema=0.0f;
const float EMA_B = 0.7f; 

const int PWM_MIN_RUN = 75;  
const int PWM_SLEW    = 8;  
static int pwmL_prev=0, pwmR_prev=0;

enum Side {NONE, LEFT, RIGHT};
Side last_seen = NONE;
bool seen_line_ever = false;
bool avoiding = false;
static volatile bool g_line_enabled = true;
bool recovering = false;
unsigned long rec_t0 = 0;
const unsigned long RECOV_TIME_MS = 6000; 

void IRAM_ATTR encL_isr(){
  uint32_t now = micros();
  if (now - encL_last_us >= MIN_EDGE_US){ encL_count++; encL_total++; encL_last_us = now; }
}
void IRAM_ATTR encR_isr(){
  uint32_t now = micros();
  if (now - encR_last_us >= MIN_EDGE_US){ encR_count++; encR_total++; encR_last_us = now; }
}

inline int clamp255(int v){ return v<0?0:(v>255?255:v); }
inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }

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
  float err = v_target - v_meas;
  pid.i_term += pid.Ki * err * dt_s;
  pid.i_term = clampf(pid.i_term, pid.out_min, pid.out_max);
  float d = (err - pid.prev_err) / dt_s;
  float u = pid.Kp * err + pid.i_term + pid.Kd * d;
  pid.prev_err = err;
  return clamp255((int)u);
}

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

void spin_left_deg(double deg, int pwmMax){
  const double target = 1.5*deg * 3.141592653589793 / 180.0;
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0; 
  const double Kp_theta = 140.0;   
  const double Kbal = 0.6;         
  const int pwmMin = 150;
  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  while (true){
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, -1, +1); 
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
  const double target = -1.5 * (deg * 3.141592653589793 / 180.0);  
  const double deg_tol = 1.5 * 3.141592653589793 / 180.0;
  const double Kp_theta = 140.0;
  const double Kbal = 0.6;
  const int pwmMin = 150;
  const unsigned long T_FAIL_MS = 5000;                      
  unsigned long t0 = millis();
  long L0, R0; noInterrupts(); L0 = encL_total; R0 = encR_total; interrupts();
  while (true){
    long L, R; noInterrupts(); L = encL_total; R = encR_total; interrupts();
    long dL = labs(L - L0), dR = labs(R - R0);
    double theta = theta_from_counts(dL, dR, +1, -1);
    double err = target - theta;
    if (fabs(err) <= deg_tol) break;
    if (millis() - t0 > T_FAIL_MS) break;                    
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
    if (onLine(M_SENSOR)){ motorsStop(); return true; }
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

void avoidObstacle(){
  const int TURN_PWM = 150, FWD_PWM  = 100;
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
  pinMode(L2_SENSOR, INPUT); pinMode(L1_SENSOR, INPUT);
  pinMode(M_SENSOR,  INPUT); pinMode(R1_SENSOR, INPUT); pinMode(R2_SENSOR, INPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L), encL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), encR_isr, RISING);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  g_line_enabled = true; seen_line_ever = false;
  vL_ema = 0.0f; vR_ema = 0.0f; pwmL_prev = 0; pwmR_prev = 0;
  us_last_ms = 0; us_dist_cm = 999.0f; obs_hit = 0; obs_latched = false;
  pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
  motorsStop();
}

void do_line_loop() {
  if (!g_line_enabled) { motorsStop(); return; }

  static unsigned long t_prev = millis();
  static unsigned long bad_t = 0; 

  bool L2 = onLine(L2_SENSOR), L1 = onLine(L1_SENSOR);
  bool M  = onLine(M_SENSOR);
  bool R1 = onLine(R1_SENSOR), R2 = onLine(R2_SENSOR);

  if (L2 || L1 || M || R1 || R2) seen_line_ever = true;

  bool bad = (L2&&L1&&M&&R1&&R2) || (!L2&&!L1&&!M&&!R1&&!R2);
  if (bad) {
    if (millis() - bad_t > 1000) {
      recovering = false; avoiding = false; motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis(); return;
    }
  } else { bad_t = millis(); }

  bool line_follow_active = isValidLineSample5(L2,L1,M,R1,R2) && !recovering && !avoiding;
  updateObstacleState();

  if (line_follow_active && obs_latched){
    avoiding = true; motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis(); avoidObstacle(); motorsStop();
    noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
    t_prev = millis(); avoiding = false; return;
  }
  if (avoiding) return;

  float vL_tgt = 0, vR_tgt = 0;

  if (recovering) {
    if (L2 || L1 || M || R1 || R2) { recovering = false; motorsStop(); } 
    else if (millis() - rec_t0 >= RECOV_TIME_MS) {
      recovering = false; motorsStop();
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      t_prev = millis(); return;
    } else {
      if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
      else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
      else                         { vL_tgt =  vF; vR_tgt = -vF; } 
    }
  }

  if (!recovering){
    const int on_cnt = (int)L2 + L1 + M + R1 + R2;

    if ( M && !L1 && !R1 && !L2 && !R2 ){ last_seen = NONE; vL_tgt = v_base; vR_tgt = v_base; }
    else if ( L1 &&  M && !R1 ){ last_seen = LEFT; vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost; }
    else if ( L1 && !M && !L2 ){ last_seen = LEFT; vL_tgt = v_base - v_boost; vR_tgt = v_base + v_boost; }
    else if ( L2 && (!R1 && !R2) ){ last_seen = LEFT; vL_tgt = v_base - v_hard;  vR_tgt = v_base + v_hard; }
    else if ( R1 &&  M && !L1 ){ last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost; }
    else if ( R1 && !M && !R2 ){ last_seen = RIGHT; vL_tgt = v_base + v_boost; vR_tgt = v_base - v_boost; }
    else if ( R2 && (!L1 && !L2) ){ last_seen = RIGHT; vL_tgt = v_base + v_hard;  vR_tgt = v_base - v_hard; }
    
    // ================= XỬ LÝ NGÃ TƯ / NGÃ BA CHỮ T =================
    else if (on_cnt >= 3){
      if (currentMode == MODE_LINE_ONLY) {
        motorsStop(); delay(200); 
        
        move_forward_distance(0.06, 120); 
        
        if (!onLine(M_SENSOR) && !onLine(L1_SENSOR) && !onLine(R1_SENSOR)) {
          motorWriteLR_signed(130, -130); 
          unsigned long t0 = millis();
          while(millis() - t0 < 3500) {
            if (onLine(M_SENSOR) || onLine(L1_SENSOR) || onLine(R1_SENSOR)) {
              break; 
            }
            delay(5);
          }
          motorsStop();
        }
        
        last_seen = NONE; 
        recovering = false;
      } 
      else if (currentMode == MODE_DELIVERY) {
        static unsigned long last_node_time = 0;
        if (millis() - last_node_time < 1500) {
          last_seen = NONE; vL_tgt = v_base * 0.8f; vR_tgt = v_base * 0.8f;
        } else {
          last_node_time = millis();
          motorsStop(); delay(300);

          if (pathLength > 0) {
            currentPathIndex++; 
            if (currentPathIndex >= pathLength - 1) {
              gripOpen(); delay(1500); gripClose();      
              delivered_count++; 
              line_mode = false; do_line_abort(); 
              return;
            }

            int currentNode = currentPath[currentPathIndex];
            int nextNode = currentPath[currentPathIndex + 1];
            int targetDir = getTargetDirection(currentNode, nextNode);
            
            if (targetDir != -1) {
              int diff = (targetDir - currentDir + 4) % 4;
              const int TURN_PWM = 160; 

              if (diff == 1) spin_right_deg(85.0, TURN_PWM); 
              else if (diff == 3) spin_left_deg(85.0, TURN_PWM);  
              else if (diff == 2) spin_right_deg(175.0, TURN_PWM);
              currentDir = targetDir; 
            }
            move_forward_distance(0.08, 120); 
          }
        }
      }
    }
    else {
      if (!seen_line_ever){ vL_tgt = 0; vR_tgt = 0; }
      else {
        recovering = true; rec_t0 = millis();
        if (last_seen == LEFT)       { vL_tgt = -vF; vR_tgt =  vF; }
        else if (last_seen == RIGHT) { vL_tgt =  vF; vR_tgt = -vF; }
        else                         { vL_tgt =  vF; vR_tgt = -vF; } 
      }
    }
  }

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