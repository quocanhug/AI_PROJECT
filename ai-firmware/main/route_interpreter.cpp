#include <Arduino.h>
#include <ArduinoJson.h>
#include "route_interpreter.h"
#include "do_line.h"

/* ================================================================
   ROUTE INTERPRETER — State Machine for AI-directed navigation
   
   Reuses functions from do_line.cpp:
   - PID line following (via do_line_loop partial logic)
   - spin_left_deg(), spin_right_deg()
   - move_forward_distance()
   - motorsStop()
   - Encoder ISR (already attached in do_line_setup)
   - HC-SR04 readDistanceCM_filtered()
   ================================================================ */

// ---- External functions from do_line.cpp ----
extern void spin_left_deg(double deg, int pwmMax);
extern void spin_right_deg(double deg, int pwmMax);
extern void move_forward_distance(double dist_m, int pwmAbs);
extern float readDistanceCM_Fast();
extern void driveWheelLeft(float v_target, int pwm);
extern void driveWheelRight(float v_target, int pwm);
extern int pidStep(PID &pid, float v_target, float v_meas, float dt_s);

// External encoder variables from do_line.cpp
extern volatile long encL_count;
extern volatile long encR_count;
extern volatile long encL_total;
extern volatile long encR_total;

// External PID structs (PID defined in do_line.h)
extern PID pidL;
extern PID pidR;

// External speed parameters
extern float v_base;

// Line sensor pins from do_line.cpp
#define RT_L2_SENSOR 34
#define RT_L1_SENSOR 32
#define RT_M_SENSOR  33
#define RT_R1_SENSOR 27
#define RT_R2_SENSOR 25

// HC-SR04 pins
#define RT_TRIG_PIN 21
#define RT_ECHO_PIN 19

// ==================== Route Queue ====================
#define MAX_COMMANDS 64

static char commandQueue[MAX_COMMANDS];  // 'F', 'L', 'R'
static int cmdHead = 0;
static int cmdTail = 0;
static int cmdTotal = 0;
static int cmdExecuted = 0;

// ==================== State Machine ====================
static RouteState currentState = RS_IDLE;
static unsigned long stateTimer = 0;

// Intersection detection
static int intersectionCount = 0;
static bool wasIntersection = false;

// Obstacle handling  
static unsigned long obstacleTimer = 0;
const unsigned long OBSTACLE_TIMEOUT_MS = 10000; // 10s timeout
const float RT_OBSTACLE_TH_CM = 15.0f;

// Telemetry
static float rt_speedL = 0.0f;
static float rt_speedR = 0.0f;
static float rt_distance = 0.0f;
static float rt_obstacleDist = 999.0f;
static bool rt_sensors[5] = {0, 0, 0, 0, 0};

// PID line follow parameters for route mode
static float rt_vL_ema = 0.0f;
static float rt_vR_ema = 0.0f;
static int rt_pwmL_prev = 0;
static int rt_pwmR_prev = 0;
static unsigned long rt_ctrl_prev = 0;

// Forward distance tracking
static long rt_encL_start = 0;
static long rt_encR_start = 0;

// WebSocket send callback (set by main.ino, WsSendFn typedef in route_interpreter.h)
static WsSendFn wsSendCallback = nullptr;

void route_set_ws_callback(WsSendFn fn) {
  wsSendCallback = fn;
}

// ==================== Utility ====================
inline bool rt_onLine(int pin) { return digitalRead(pin) == HIGH; }  
inline int rt_clamp255(int v) { return v < 0 ? 0 : (v > 255 ? 255 : v); }
inline float rt_clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static const int RT_PWM_MIN_RUN = 75;
static const int RT_PWM_SLEW = 8;

static inline int rt_shape_pwm(int target, int prev) {
  int s = target;
  if (s > 0 && s < RT_PWM_MIN_RUN) s = RT_PWM_MIN_RUN;
  int d = s - prev;
  if (d > RT_PWM_SLEW) s = prev + RT_PWM_SLEW;
  if (d < -RT_PWM_SLEW) s = prev - RT_PWM_SLEW;
  return rt_clamp255(s);
}

extern float ticksToVel(long ticks, float dt_s);

// Check if all 5 sensors are on line (intersection)
bool isIntersection() {
  return rt_onLine(RT_L2_SENSOR) && rt_onLine(RT_L1_SENSOR) &&
         rt_onLine(RT_M_SENSOR) && rt_onLine(RT_R1_SENSOR) && rt_onLine(RT_R2_SENSOR);
}

// Get line error from 5 sensors: [-4, +4]
float getLineError() {
  bool L2 = rt_onLine(RT_L2_SENSOR);
  bool L1 = rt_onLine(RT_L1_SENSOR);
  bool M  = rt_onLine(RT_M_SENSOR);
  bool R1 = rt_onLine(RT_R1_SENSOR);
  bool R2 = rt_onLine(RT_R2_SENSOR);
  
  int onCount = (int)L2 + L1 + M + R1 + R2;
  if (onCount == 0) return 0.0f;  // lost line, keep straight
  
  // Weighted position: L2=-4, L1=-2, M=0, R1=+2, R2=+4
  float weighted = (-4.0f * L2) + (-2.0f * L1) + (0.0f * M) + (2.0f * R1) + (4.0f * R2);
  return weighted / onCount;
}

// Send message via WebSocket
void rt_wsSend(const String& msg) {
  if (wsSendCallback) {
    wsSendCallback(msg.c_str());
  }
  Serial.println(msg);
}

// ==================== Queue Operations ====================
void cmdQueueClear() {
  cmdHead = 0;
  cmdTail = 0;
  cmdTotal = 0;
  cmdExecuted = 0;
}

bool cmdQueueEmpty() {
  return cmdHead == cmdTail;
}

void cmdQueuePush(char cmd) {
  if (cmdTail < MAX_COMMANDS) {
    commandQueue[cmdTail++] = cmd;
    cmdTotal++;
  }
}

char cmdQueuePop() {
  if (cmdHead < cmdTail) {
    cmdExecuted++;
    return commandQueue[cmdHead++];
  }
  return '\0';
}

int cmdQueueSize() {
  return cmdTail - cmdHead;
}

// ==================== Route Setup ====================
void route_setup() {
  currentState = RS_IDLE;
  cmdQueueClear();
  intersectionCount = 0;
  wasIntersection = false;
  rt_vL_ema = 0;
  rt_vR_ema = 0;
  rt_pwmL_prev = 0;
  rt_pwmR_prev = 0;
  rt_ctrl_prev = millis();
  rt_distance = 0;
  
  noInterrupts();
  rt_encL_start = encL_total;
  rt_encR_start = encR_total;
  interrupts();
}

// ==================== Route Load ====================
void route_load(const char* json) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }
  
  if (!doc.containsKey("commands")) return;
  
  cmdQueueClear();
  JsonArray cmds = doc["commands"].as<JsonArray>();
  for (JsonVariant v : cmds) {
    const char* s = v.as<const char*>();
    if (s && s[0]) {
      cmdQueuePush(s[0]);
    }
  }
  
  bool isRerouted = doc["rerouted"] | false;
  
  Serial.printf("Route loaded: %d commands", cmdTotal);
  if (isRerouted) Serial.print(" (rerouted)");
  Serial.println();
  
  // Transition to following line
  currentState = RS_FOLLOWING_LINE;
  intersectionCount = 0;
  wasIntersection = false;
  stateTimer = millis();
  
  // Reset PID
  pidL.i_term = 0; pidL.prev_err = 0;
  pidR.i_term = 0; pidR.prev_err = 0;
  rt_vL_ema = 0; rt_vR_ema = 0;
  rt_pwmL_prev = 0; rt_pwmR_prev = 0;
  rt_ctrl_prev = millis();
  
  noInterrupts();
  encL_count = 0; encR_count = 0;
  rt_encL_start = encL_total;
  rt_encR_start = encR_total;
  interrupts();
  
  // Send ACK
  rt_wsSend("{\"type\":\"ROUTE_ACK\",\"commands\":" + String(cmdTotal) + "}");
}

// ==================== PID Line Following (for route mode) ====================
void rt_pidLineFollow() {
  unsigned long now = millis();
  const unsigned long CTRL_DT_MS = 10;
  
  if (now - rt_ctrl_prev < CTRL_DT_MS) return;
  
  float dt_s = (now - rt_ctrl_prev) / 1000.0f;
  rt_ctrl_prev = now;
  
  // Read line error
  float lineErr = getLineError();
  
  // Compute target velocities based on line error
  float v_boost = 0.11f;
  float v_hard  = 0.13f;
  float vL_tgt, vR_tgt;
  
  float absErr = fabs(lineErr);
  if (absErr < 0.5f) {
    // On center
    vL_tgt = v_base;
    vR_tgt = v_base;
  } else if (absErr < 2.0f) {
    // Light deviation
    float correction = v_boost * (lineErr / 2.0f);
    vL_tgt = v_base + correction;
    vR_tgt = v_base - correction;
  } else {
    // Heavy deviation
    float correction = v_hard * (lineErr / 4.0f);
    vL_tgt = v_base + correction;
    vR_tgt = v_base - correction;
  }
  
  // Read encoder ticks
  noInterrupts();
  long cL = encL_count; encL_count = 0;
  long cR = encR_count; encR_count = 0;
  interrupts();
  
  float vL_meas = ticksToVel(cL, dt_s);
  float vR_meas = ticksToVel(cR, dt_s);
  
  // EMA filter
  const float EMA_B = 0.7f;
  rt_vL_ema = EMA_B * rt_vL_ema + (1 - EMA_B) * vL_meas;
  rt_vR_ema = EMA_B * rt_vR_ema + (1 - EMA_B) * vR_meas;
  
  vL_tgt = rt_clampf(vL_tgt, 0, 1.5f);
  vR_tgt = rt_clampf(vR_tgt, 0, 1.5f);
  
  int pwmL = pidStep(pidL, vL_tgt, rt_vL_ema, dt_s);
  int pwmR = pidStep(pidR, vR_tgt, rt_vR_ema, dt_s);
  
  int pwmL_cmd = rt_shape_pwm(pwmL, rt_pwmL_prev);
  int pwmR_cmd = rt_shape_pwm(pwmR, rt_pwmR_prev);
  rt_pwmL_prev = pwmL_cmd;
  rt_pwmR_prev = pwmR_cmd;
  
  driveWheelLeft(vL_tgt, pwmL_cmd);
  driveWheelRight(vR_tgt, pwmR_cmd);
  
  // Update telemetry
  rt_speedL = rt_vL_ema;
  rt_speedR = rt_vR_ema;
}

// ==================== Read Sensors ====================
void rt_readSensors() {
  rt_sensors[0] = rt_onLine(RT_L2_SENSOR);
  rt_sensors[1] = rt_onLine(RT_L1_SENSOR);
  rt_sensors[2] = rt_onLine(RT_M_SENSOR);
  rt_sensors[3] = rt_onLine(RT_R1_SENSOR);
  rt_sensors[4] = rt_onLine(RT_R2_SENSOR);
}

// ==================== Check HC-SR04 ====================  
static unsigned long rt_us_last = 0;

float rt_checkObstacle() {
  if (millis() - rt_us_last < 30) return rt_obstacleDist;
  rt_us_last = millis();
  rt_obstacleDist = readDistanceCM_Fast();
  return rt_obstacleDist;
}

// ==================== Compute grid position ====================
struct GridPos {
  int row;
  int col;
  int dir; // 0=down(+Y), 1=right(+X), 2=up(-Y), 3=left(-X)
};

static GridPos robotGridPos = {0, 0, 0};

void rt_initGridPos(int startRow, int startCol, int startDir) {
  robotGridPos.row = startRow;
  robotGridPos.col = startCol;
  robotGridPos.dir = startDir;
}

GridPos rt_getNextGridPos(GridPos pos) {
  GridPos next = pos;
  int dRow[] = {1, 0, -1, 0}; 
  int dCol[] = {0, 1, 0, -1};
  next.row += dRow[pos.dir];
  next.col += dCol[pos.dir];
  return next;
}

void rt_updateDirAfterTurn(char cmd) {
  if (cmd == 'R') {
    robotGridPos.dir = (robotGridPos.dir + 1) % 4;
  } else if (cmd == 'L') {
    robotGridPos.dir = (robotGridPos.dir + 3) % 4;
  }
}

void rt_advanceGridPos() {
  robotGridPos = rt_getNextGridPos(robotGridPos);
}

// ==================== STATE MACHINE MAIN LOOP ====================
void route_loop() {
  rt_readSensors();
  
  // Calculate distance
  noInterrupts();
  long totalL = encL_total - rt_encL_start;
  long totalR = encR_total - rt_encR_start;
  interrupts();
  const float RT_CIRC = 2.0f * 3.1415926f * 0.0325f;  
  const int RT_PPR = 20;  // ĐÃ SỬA THÀNH 20 XUNG CHO KHỚP ENCODER
  rt_distance = ((float)(totalL + totalR) / 2.0f / RT_PPR) * RT_CIRC * 100.0f; // cm
  
  switch (currentState) {
    
    // ========== IDLE ==========
    case RS_IDLE:
      motorsStop();
      break;
    
    // ========== FOLLOWING LINE ==========
    case RS_FOLLOWING_LINE: {
      // Check for obstacle
      float dist = rt_checkObstacle();
      if (dist > 0 && dist < RT_OBSTACLE_TH_CM) {
        motorsStop();
        currentState = RS_OBSTACLE;
        obstacleTimer = millis();
        Serial.println("OBSTACLE detected, stopping!");
        
        GridPos obsPos = rt_getNextGridPos(robotGridPos);
        
        String msg = "{\"type\":\"OBSTACLE_DETECTED\","
                     "\"position\":{\"row\":" + String(obsPos.row) + ",\"col\":" + String(obsPos.col) + "},"
                     "\"robot_position\":{\"row\":" + String(robotGridPos.row) + ",\"col\":" + String(robotGridPos.col) + "},"
                     "\"current_step\":" + String(cmdExecuted) + ","
                     "\"distance_cm\":" + String(dist, 1) + "}";
        rt_wsSend(msg);
        break;
      }
      
      // Check for intersection
      bool isInter = isIntersection();
      if (isInter && !wasIntersection) {
        motorsStop();
        delay(50);
        
        move_forward_distance(0.02, 100);
        motorsStop();
        delay(50);
        
        currentState = RS_AT_INTERSECTION;
        wasIntersection = true;
        break;
      }
      wasIntersection = isInter;
      
      rt_pidLineFollow();
      break;
    }
    
    // ========== AT INTERSECTION ==========
    case RS_AT_INTERSECTION: {
      if (cmdQueueEmpty()) {
        currentState = RS_DONE;
        break;
      }
      
      char cmd = cmdQueuePop();
      Serial.printf("Intersection #%d: command '%c' (step %d/%d)\n", 
                     intersectionCount + 1, cmd, cmdExecuted, cmdTotal);
      
      String msg = "{\"type\":\"TELEMETRY\",\"state\":\"AT_INTERSECTION\","
                   "\"step\":" + String(cmdExecuted) + ",\"total\":" + String(cmdTotal) + ","
                   "\"command\":\"" + String(cmd) + "\"}";
      rt_wsSend(msg);
      
      switch (cmd) {
        case 'F':
          rt_advanceGridPos();
          break;
        case 'L':
          spin_left_deg(90.0, 140); // ĐÃ GIẢM LỰC XUỐNG 140
          motorsStop();
          delay(100);
          rt_updateDirAfterTurn('L');
          rt_advanceGridPos();
          break;
        case 'R':
          spin_right_deg(90.0, 140); // ĐÃ GIẢM LỰC XUỐNG 140
          motorsStop();
          delay(100);
          rt_updateDirAfterTurn('R');
          rt_advanceGridPos();
          break;
        default:
          Serial.printf("Unknown command: %c\n", cmd);
          break;
      }
      
      intersectionCount++;
      wasIntersection = false;
      
      // Reset PID
      pidL.i_term = 0; pidL.prev_err = 0;
      pidR.i_term = 0; pidR.prev_err = 0;
      rt_vL_ema = 0; rt_vR_ema = 0;
      rt_pwmL_prev = 0; rt_pwmR_prev = 0;
      noInterrupts(); encL_count = 0; encR_count = 0; interrupts();
      rt_ctrl_prev = millis();
      
      if (cmdQueueEmpty()) {
        currentState = RS_DONE;
      } else {
        currentState = RS_FOLLOWING_LINE;
      }
      break;
    }
    
    // ========== OBSTACLE ==========
    case RS_OBSTACLE: {
      motorsStop();
      if (millis() - obstacleTimer > OBSTACLE_TIMEOUT_MS) {
        Serial.println("Obstacle timeout! Entering safe mode.");
        rt_wsSend("{\"type\":\"OBSTACLE_TIMEOUT\"}");
        currentState = RS_IDLE;
      }
      break;
    }
    
    // ========== REROUTING ==========
    case RS_REROUTING: {
      currentState = RS_FOLLOWING_LINE;
      break;
    }
    
    // ========== DONE ==========
    case RS_DONE: {
      motorsStop();
      Serial.println("Route completed!");
      rt_wsSend("{\"type\":\"COMPLETED\",\"intersections\":" + String(intersectionCount) + 
                ",\"distance_cm\":" + String(rt_distance, 1) + "}");
      currentState = RS_IDLE;
      break;
    }
  }
}

// ==================== API IMPLEMENTATIONS ====================
void route_abort() {
  motorsStop();
  cmdQueueClear();
  currentState = RS_IDLE;
  Serial.println("Route aborted");
}

bool route_is_done() {
  return currentState == RS_IDLE || currentState == RS_DONE;
}

const char* route_state_str() {
  switch (currentState) {
    case RS_IDLE: return "IDLE";
    case RS_FOLLOWING_LINE: return "FOLLOWING_LINE";
    case RS_AT_INTERSECTION: return "AT_INTERSECTION";
    case RS_OBSTACLE: return "OBSTACLE";
    case RS_REROUTING: return "REROUTING";
    case RS_DONE: return "DONE";
    default: return "UNKNOWN";
  }
}

RouteState route_get_state() {
  return currentState;
}

int route_current_step() {
  return cmdExecuted;
}

int route_total_steps() {
  return cmdTotal;
}

String route_telemetry_json() {
  String json = "{\"type\":\"TELEMETRY\","
                "\"state\":\"" + String(route_state_str()) + "\","
                "\"step\":" + String(cmdExecuted) + ","
                "\"total\":" + String(cmdTotal) + ","
                "\"speedL\":" + String(rt_speedL, 3) + ","
                "\"speedR\":" + String(rt_speedR, 3) + ","
                "\"distance\":" + String(rt_distance, 1) + ","
                "\"obstacle\":" + String(rt_obstacleDist, 1) + ","
                "\"sensors\":[" + 
                  String((int)rt_sensors[0]) + "," +
                  String((int)rt_sensors[1]) + "," +
                  String((int)rt_sensors[2]) + "," +
                  String((int)rt_sensors[3]) + "," +
                  String((int)rt_sensors[4]) + "]}";
  return json;
}