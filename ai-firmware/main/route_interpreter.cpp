#include <Arduino.h>
#include <ArduinoJson.h>
#include "route_interpreter.h"
#include "do_line.h"

extern void spin_left_deg(double deg, int pwmMax);
extern void spin_right_deg(double deg, int pwmMax);
extern void move_forward_distance(double dist_m, int pwmAbs);
extern float readDistanceCM_Fast();
extern void driveWheelLeft(float v_target, int pwm);
extern void driveWheelRight(float v_target, int pwm);
extern int pidStep(PID &pid, float v_target, float v_meas, float dt_s);

extern volatile long encL_count;
extern volatile long encR_count;
extern volatile long encL_total;
extern volatile long encR_total;

extern PID pidL;
extern PID pidR;
extern float v_base;
extern volatile int delivered_count;
extern void gripOpen();
extern void gripClose();

#define RT_L2_SENSOR 34
#define RT_L1_SENSOR 32
#define RT_M_SENSOR  33
#define RT_R1_SENSOR 27
#define RT_R2_SENSOR 25
#define RT_TRIG_PIN 21
#define RT_ECHO_PIN 19

#define MAX_COMMANDS 64
static char commandQueue[MAX_COMMANDS];
static int cmdHead = 0;
static int cmdTail = 0;
static int cmdTotal = 0;
static int cmdExecuted = 0;

static RouteState currentState = RS_IDLE;
static unsigned long stateTimer = 0;
static int intersectionCount = 0;
static bool wasIntersection = false;

static unsigned long obstacleTimer = 0;
const unsigned long OBSTACLE_TIMEOUT_MS = 10000; 
const float RT_OBSTACLE_TH_CM = 15.0f;

static float rt_speedL = 0.0f;
static float rt_speedR = 0.0f;
static float rt_distance = 0.0f;
static float rt_obstacleDist = 999.0f;
static bool rt_sensors[5] = {0, 0, 0, 0, 0};

static float rt_vL_ema = 0.0f;
static float rt_vR_ema = 0.0f;
static int rt_pwmL_prev = 0;
static int rt_pwmR_prev = 0;
static unsigned long rt_ctrl_prev = 0;
static long rt_encL_start = 0;
static long rt_encR_start = 0;

static WsSendFn wsSendCallback = nullptr;
void route_set_ws_callback(WsSendFn fn) { wsSendCallback = fn; }

inline bool rt_onLine(int pin) { return digitalRead(pin) == HIGH; }  
inline int rt_clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
inline float rt_clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static const int RT_PWM_MIN_RUN = 75;
static const int RT_PWM_SLEW = 8;
static inline int rt_shape_pwm(int target, int prev) {
  int s = target; if (s > 0 && s < RT_PWM_MIN_RUN) s = RT_PWM_MIN_RUN;
  int d = s - prev; if (d > RT_PWM_SLEW) s = prev + RT_PWM_SLEW; if (d < -RT_PWM_SLEW) s = prev - RT_PWM_SLEW;
  return rt_clamp255(s);
}

extern float ticksToVel(long ticks, float dt_s);

// ================= TỐI ƯU CẢM BIẾN 5 MẮT: Nhận diện chính xác tránh đếm nhầm =================
bool isIntersection() {
  bool L2 = rt_onLine(RT_L2_SENSOR);
  bool L1 = rt_onLine(RT_L1_SENSOR);
  bool M  = rt_onLine(RT_M_SENSOR);
  bool R1 = rt_onLine(RT_R1_SENSOR);
  bool R2 = rt_onLine(RT_R2_SENSOR);
  
  // Bắt buộc mắt giữa và ít nhất 1 mắt rìa ngoài cùng sáng thì mới là ngã tư / ngã 3
  if (M && L2 && R2) return true; // Ngã tư chữ thập
  if (M && L2 && L1 && !R2) return true; // Ngã ba vuông góc bên trái
  if (M && R2 && R1 && !L2) return true; // Ngã ba vuông góc bên phải
  if (L2 && L1 && M && R1 && R2) return true; // Full toàn bộ vạch
  
  return false;
}

static float rt_last_line_err = 0.0f; // Nhớ hướng nếu bị văng

float getLineError() {
  bool L2 = rt_onLine(RT_L2_SENSOR); bool L1 = rt_onLine(RT_L1_SENSOR); bool M  = rt_onLine(RT_M_SENSOR);
  bool R1 = rt_onLine(RT_R1_SENSOR); bool R2 = rt_onLine(RT_R2_SENSOR);
  
  int onCount = (int)L2 + (int)L1 + (int)M + (int)R1 + (int)R2;
  
  if (onCount == 0) { // Mất line -> Tự động bẻ lái gắt tìm vạch theo trí nhớ
    if (rt_last_line_err > 0) return 4.0f;  
    if (rt_last_line_err < 0) return -4.0f; 
    return 0.0f;
  }
  
  float weighted = (-4.0f * L2) + (-2.0f * L1) + (0.0f * M) + (2.0f * R1) + (4.0f * R2);
  rt_last_line_err = weighted / onCount; 
  return rt_last_line_err;
}

void rt_wsSend(const String& msg) { if (wsSendCallback) { wsSendCallback(msg.c_str()); } }

void cmdQueueClear() { cmdHead = 0; cmdTail = 0; cmdTotal = 0; cmdExecuted = 0; }
bool cmdQueueEmpty() { return cmdHead == cmdTail; }
void cmdQueuePush(char cmd) { if (cmdTail < MAX_COMMANDS) { commandQueue[cmdTail++] = cmd; cmdTotal++; } }
char cmdQueuePop() { if (cmdHead < cmdTail) { cmdExecuted++; return commandQueue[cmdHead++]; } return '\0'; }

struct GridPos { int row; int col; int dir; }; 
static GridPos robotGridPos = {0, 0, 0};

void rt_initGridPos(int startNode, int startDir) {
  robotGridPos.row = startNode / 5; robotGridPos.col = startNode % 5; robotGridPos.dir = startDir;
}
GridPos rt_getNextGridPos(GridPos pos) {
  GridPos next = pos; int dRow[] = {-1, 0, 1, 0}; int dCol[] = {0, 1, 0, -1};
  next.row += dRow[pos.dir]; next.col += dCol[pos.dir]; return next;
}
void rt_updateDirAfterTurn(char cmd) {
  if (cmd == 'R') robotGridPos.dir = (robotGridPos.dir + 1) % 4;
  else if (cmd == 'L') robotGridPos.dir = (robotGridPos.dir + 3) % 4;
}
void rt_advanceGridPos() { robotGridPos = rt_getNextGridPos(robotGridPos); }

void route_setup() {
  currentState = RS_IDLE; cmdQueueClear(); intersectionCount = 0; wasIntersection = false;
  rt_vL_ema = 0; rt_vR_ema = 0; rt_pwmL_prev = 0; rt_pwmR_prev = 0; rt_ctrl_prev = millis(); rt_distance = 0;
  noInterrupts(); rt_encL_start = encL_total; rt_encR_start = encR_total; interrupts();
}

void route_load(const char* json) {
  StaticJsonDocument<1024> doc; DeserializationError err = deserializeJson(doc, json);
  if (err) { Serial.println("JSON error"); return; }
  if (!doc.containsKey("commands")) return;
  cmdQueueClear();
  JsonArray cmds = doc["commands"].as<JsonArray>();
  for (JsonVariant v : cmds) { const char* s = v.as<const char*>(); if (s && s[0]) cmdQueuePush(s[0]); }
  
  rt_initGridPos(doc["startNode"] | 0, doc["initialDir"] | 2);
  
  currentState = RS_FOLLOWING_LINE; intersectionCount = 0; 
  wasIntersection = true; // Khóa không cho đếm điểm xuất phát (Kho)
  stateTimer = millis();
  
  pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
  rt_vL_ema = 0; rt_vR_ema = 0; rt_pwmL_prev = 0; rt_pwmR_prev = 0; rt_ctrl_prev = millis();
  noInterrupts(); encL_count = 0; encR_count = 0; rt_encL_start = encL_total; rt_encR_start = encR_total; interrupts();
  
  rt_wsSend("{\"type\":\"ROUTE_ACK\",\"commands\":" + String(cmdTotal) + "}");
}

void rt_pidLineFollow() {
  unsigned long now = millis(); const unsigned long CTRL_DT_MS = 10;
  if (now - rt_ctrl_prev < CTRL_DT_MS) return;
  float dt_s = (now - rt_ctrl_prev) / 1000.0f; rt_ctrl_prev = now;
  
  float lineErr = getLineError();
  float v_boost = 0.10f; float v_hard  = 0.15f; 
  float vL_tgt, vR_tgt;
  float absErr = fabs(lineErr);
  if (absErr < 0.5f) { vL_tgt = v_base; vR_tgt = v_base; } 
  else if (absErr < 2.0f) { float c = v_boost * (lineErr / 2.0f); vL_tgt = v_base + c; vR_tgt = v_base - c; } 
  else { float c = v_hard * (lineErr / 4.0f); vL_tgt = v_base + c; vR_tgt = v_base - c; }
  
  noInterrupts(); long cL = encL_count; encL_count = 0; long cR = encR_count; encR_count = 0; interrupts();
  float vL_meas = ticksToVel(cL, dt_s); float vR_meas = ticksToVel(cR, dt_s);
  
  const float EMA_B = 0.7f;
  rt_vL_ema = EMA_B * rt_vL_ema + (1 - EMA_B) * vL_meas; rt_vR_ema = EMA_B * rt_vR_ema + (1 - EMA_B) * vR_meas;
  
  vL_tgt = rt_clampf(vL_tgt, 0, 1.5f); vR_tgt = rt_clampf(vR_tgt, 0, 1.5f);
  int pwmL = pidStep(pidL, vL_tgt, rt_vL_ema, dt_s); int pwmR = pidStep(pidR, vR_tgt, rt_vR_ema, dt_s);
  
  int pwmL_cmd = rt_shape_pwm(pwmL, rt_pwmL_prev); int pwmR_cmd = rt_shape_pwm(pwmR, rt_pwmR_prev);
  rt_pwmL_prev = pwmL_cmd; rt_pwmR_prev = pwmR_cmd;
  
  driveWheelLeft(vL_tgt, pwmL_cmd); driveWheelRight(vR_tgt, pwmR_cmd);
  rt_speedL = rt_vL_ema; rt_speedR = rt_vR_ema;
}

void rt_readSensors() {
  rt_sensors[0] = rt_onLine(RT_L2_SENSOR); rt_sensors[1] = rt_onLine(RT_L1_SENSOR);
  rt_sensors[2] = rt_onLine(RT_M_SENSOR); rt_sensors[3] = rt_onLine(RT_R1_SENSOR); rt_sensors[4] = rt_onLine(RT_R2_SENSOR);
}

static unsigned long rt_us_last = 0;
float rt_checkObstacle() {
  if (millis() - rt_us_last < 30) return rt_obstacleDist;
  rt_us_last = millis(); rt_obstacleDist = readDistanceCM_Fast(); return rt_obstacleDist;
}

void route_loop() {
  rt_readSensors();
  
  noInterrupts(); long totalL = encL_total - rt_encL_start; long totalR = encR_total - rt_encR_start; interrupts();
  const float RT_CIRC = 2.0f * 3.1415926f * 0.0325f;  const int RT_PPR = 20;  
  rt_distance = ((float)(totalL + totalR) / 2.0f / RT_PPR) * RT_CIRC * 100.0f;
  
  static unsigned long lastTel = 0;
  if (millis() - lastTel > 500 && currentState != RS_IDLE) {
    lastTel = millis(); rt_wsSend(route_telemetry_json());
  }

  switch (currentState) {
    case RS_IDLE: motorsStop(); break;
    
    case RS_FOLLOWING_LINE: {
      float dist = rt_checkObstacle();
      if (dist > 0 && dist < RT_OBSTACLE_TH_CM) {
        motorsStop(); currentState = RS_OBSTACLE; obstacleTimer = millis();
        GridPos obsPos = rt_getNextGridPos(robotGridPos);
        String msg = "{\"type\":\"OBSTACLE_DETECTED\",\"position\":{\"row\":" + String(obsPos.row) + ",\"col\":" + String(obsPos.col) + "},\"robot_position\":{\"row\":" + String(robotGridPos.row) + ",\"col\":" + String(robotGridPos.col) + "},\"current_step\":" + String(cmdExecuted) + ",\"distance_cm\":" + String(dist, 1) + "}";
        rt_wsSend(msg);
        break;
      }
      
      bool isInter = isIntersection();
      if (isInter && !wasIntersection) {
        motorsStop(); delay(50);
        currentState = RS_AT_INTERSECTION;
        wasIntersection = true;
        break;
      }
      wasIntersection = isInter; 
      rt_pidLineFollow();
      break;
    }
    
    case RS_AT_INTERSECTION: {
      if (cmdQueueEmpty()) { currentState = RS_DONE; break; }
      char cmd = cmdQueuePop();
      
      // TỐI ƯU CƠ KHÍ: Đi lố qua ngã tư để tránh đếm đúp Node 
      switch (cmd) {
        case 'F': 
          move_forward_distance(0.08, 120); // Phóng thẳng 8cm qua khỏi ngã tư
          rt_advanceGridPos(); 
          break;
        case 'L': 
          move_forward_distance(0.045, 120); // Đi vào tâm
          spin_left_deg(90.0, 140); motorsStop(); delay(100); 
          move_forward_distance(0.08, 120); // Trườn khỏi ngã tư
          rt_updateDirAfterTurn('L'); rt_advanceGridPos(); 
          break;
        case 'R': 
          move_forward_distance(0.045, 120); // Đi vào tâm
          spin_right_deg(90.0, 140); motorsStop(); delay(100); 
          move_forward_distance(0.08, 120); // Trườn khỏi ngã tư
          rt_updateDirAfterTurn('R'); rt_advanceGridPos(); 
          break;
      }
      
      intersectionCount++; 
      
      pidL.i_term = 0; pidL.prev_err = 0; pidR.i_term = 0; pidR.prev_err = 0;
      rt_vL_ema = 0; rt_vR_ema = 0; rt_pwmL_prev = 0; rt_pwmR_prev = 0;
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      rt_ctrl_prev = millis();
      
      if (cmdQueueEmpty()) currentState = RS_DONE;
      else currentState = RS_FOLLOWING_LINE;
      
      rt_wsSend(route_telemetry_json()); 
      break;
    }
    
    case RS_OBSTACLE: {
      motorsStop();
      if (millis() - obstacleTimer > OBSTACLE_TIMEOUT_MS) { rt_wsSend("{\"type\":\"OBSTACLE_TIMEOUT\"}"); currentState = RS_IDLE; }
      break;
    }
    case RS_REROUTING: { currentState = RS_FOLLOWING_LINE; break; }
    case RS_DONE: {
      motorsStop(); gripOpen(); delay(1500); gripClose(); delivered_count++;
      rt_wsSend("{\"type\":\"COMPLETED\",\"intersections\":" + String(intersectionCount) + ",\"distance_cm\":" + String(rt_distance, 1) + "}");
      currentState = RS_IDLE; is_auto_running = false; 
      break;
    }
  }
}

void route_abort() { motorsStop(); cmdQueueClear(); currentState = RS_IDLE; }
bool route_is_done() { return currentState == RS_IDLE || currentState == RS_DONE; }

const char* route_state_str() {
  switch (currentState) {
    case RS_IDLE: return "IDLE"; case RS_FOLLOWING_LINE: return "FOLLOWING_LINE"; case RS_AT_INTERSECTION: return "AT_INTERSECTION";
    case RS_OBSTACLE: return "OBSTACLE"; case RS_REROUTING: return "REROUTING"; case RS_DONE: return "DONE"; default: return "UNKNOWN";
  }
}
RouteState route_get_state() { return currentState; }
int route_current_step() { return cmdExecuted; }
int route_total_steps() { return cmdTotal; }

String route_telemetry_json() {
  int node = robotGridPos.row * 5 + robotGridPos.col;
  return "{\"type\":\"TELEMETRY\",\"state\":\"" + String(route_state_str()) + "\",\"step\":" + String(cmdExecuted) + ",\"total\":" + String(cmdTotal) + ",\"speedL\":" + String(rt_speedL, 3) + ",\"speedR\":" + String(rt_speedR, 3) + ",\"distance\":" + String(rt_distance, 1) + ",\"obstacle\":" + String(rt_obstacleDist, 1) + ",\"robotNode\":" + String(node) + ",\"robotDir\":" + String(robotGridPos.dir) + ",\"sensors\":[" + String((int)rt_sensors[0]) + "," + String((int)rt_sensors[1]) + "," + String((int)rt_sensors[2]) + "," + String((int)rt_sensors[3]) + "," + String((int)rt_sensors[4]) + "]}";
}