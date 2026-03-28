# Walkthrough — Animation + WebSocket + Edge Editor

## Tổng kết thay đổi

### ✅ Web — Animation từng bước thuật toán

**5 thuật toán** (BFS, DFS, UCS, A*, Greedy) giờ trả về step history:
- Mỗi step: `{current, explored: Set, frontier: Set}`
- Animation Engine: Play ▶ / Pause ⏸ / Step ⏭ / Reset ↺
- Speed slider: Chậm (1500ms) ↔ Nhanh (30ms)  
- Node colors: 🟡 Explored, 🔵 Frontier, 🟠 Current, 🟢 Final Path

![Animation hoạt động](C:\Users\LOQ\.gemini\antigravity\brain\6d26f27d-9848-4a88-866c-0f6d0ebc0d0e\.system_generated\click_feedback\click_feedback_1774680913851.png)

### ✅ Web — Edge & Weight Editor

- **Tool Edge**: Click 2 node → toggle edge (thêm/xóa kết nối)
- **Tool Trọng số**: Click 2 node kề → prompt nhập weight
- **Dynamic SVG**: Edges tự render lại khi thay đổi graph
- **Weight labels**: Hiển thị trọng số ≠ 1 trên edge

### ✅ Firmware — WebSocket Server

```diff:main.ino
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>     
#include "do_line.h"     

const char* ssid = "ESP32-Car";
const char* password = "12345678";

#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

int speed_linear = 130;
int speed_rot    = 110;
const int SPEED_MIN  = 60;
const int SPEED_MAX  = 255;
const int SPEED_STEP = 10;
const int DIAG_SCALE = 70; 
static inline int diagScale(int v){ return v * DIAG_SCALE / 100; }
static inline int clamp(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
const bool INVERT_STEER = true; 

#define SERVO_PIN 18
const int OPEN_ANGLE  = 120;
const int CLOSE_ANGLE = 175;
Servo gripper;

AsyncWebServer server(80);

// ================= CÁC BIẾN QUẢN LÝ CHẾ ĐỘ =================
volatile UIMode currentMode = MODE_MANUAL;
bool line_mode = false;
bool is_auto_running = false; // KHÓA AN TOÀN TRẠNG THÁI CHẠY

int currentPath[20]; 
int pathLength = 0;
int currentTargetNode = -1;
int currentPathIndex = 0;
int currentDir = 1; 

volatile int delivered_count = 0;
volatile float avg_time_sec = 0.0;
volatile int robot_efficiency = 100;

enum Motion { STOPPED, FWD, BWD, LEFT_TURN, RIGHT_TURN, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT };
volatile Motion curMotion = STOPPED;

void forward(); void backward(); void left(); void right(); void stopCar();
void forwardLeft(); void forwardRight(); void backwardLeft(); void backwardRight();
void gripOpen(); void gripClose();
void applyCurrentMotion();

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2500);
  gripClose();
  stopCar();

  Serial.begin(115200);

  if(!LittleFS.begin(true)){
    Serial.println("Lỗi Mount LittleFS!");
    return;
  }
  
  WiFi.softAP(ssid, password);
  Serial.print("Hotspot IP: "); Serial.println(WiFi.softAPIP());

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/setMode", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("m")) { r->send(400,"text/plain","manual"); return; }
    String m = r->getParam("m")->value();

    if (m == "line_only") {
      stopCar();
      do_line_setup();
      currentMode = MODE_LINE_ONLY;
      line_mode = true;
      is_auto_running = true; // MỞ KHÓA CHẠY
    } else if (m == "manual") {
      do_line_abort();
      stopCar();
      currentMode = MODE_MANUAL;
      line_mode = false;
      is_auto_running = false; // ĐÓNG KHÓA
    }
    r->send(200,"text/plain", m);
  });

  server.on("/deliver", HTTP_GET, [](AsyncWebServerRequest* r){
    if (r->hasParam("dir")) currentDir = r->getParam("dir")->value().toInt();
    currentPathIndex = 0; 
    
    if (r->hasParam("path")) {
      String pathStr = r->getParam("path")->value();
      pathLength = 0;
      int startIdx = 0; int commaIdx = pathStr.indexOf(',');
      while (commaIdx != -1 && pathLength < 20) {
        currentPath[pathLength++] = pathStr.substring(startIdx, commaIdx).toInt();
        startIdx = commaIdx + 1; commaIdx = pathStr.indexOf(',', startIdx);
      }
      if (startIdx < pathStr.length() && pathLength < 20) currentPath[pathLength++] = pathStr.substring(startIdx).toInt();
      if(pathLength > 1) currentTargetNode = currentPath[1];
    }
    
    stopCar();
    do_line_setup();
    currentMode = MODE_DELIVERY;
    line_mode = true;
    is_auto_running = true; // MỞ KHÓA CHẠY GIAO HÀNG
    
    r->send(200, "text/plain", "OK");
  });

  server.on("/estop", HTTP_GET, [](AsyncWebServerRequest *r){
    stopCar(); do_line_abort();
    currentMode = MODE_MANUAL; line_mode = false; is_auto_running = false;
    r->send(200, "text/plain", "E-STOP ACTIVATED");
  });

  server.on("/resume", HTTP_GET, [](AsyncWebServerRequest *r){
    if(currentMode != MODE_MANUAL) {
      stopCar(); do_line_setup();
      line_mode = true; is_auto_running = true;
    }
    r->send(200, "text/plain", "RESUMED");
  });

  server.on("/return_home", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send(200, "text/plain", "RETURNING HOME");
  });

  server.on("/api/stats", HTTP_GET, [](AsyncWebServerRequest *r){
    String json = "{";
    json += "\"delivered\":" + String(delivered_count) + ",";
    json += "\"avgTime\":" + String(avg_time_sec) + ",";
    json += "\"efficiency\":" + String(robot_efficiency) + ",";
    json += "\"chart\":{\"labels\":[\"Đơn 1\",\"Đơn 2\",\"Đơn 3\"],\"expected\":[30, 45, 60],\"actual\":[32, 42, 65]}}";
    r->send(200, "application/json", json);
  });

  server.on("/forward",    HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD;        if(currentMode==MODE_MANUAL) forward();      r->send(200,"text/plain","OK"); });
  server.on("/backward",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BWD;        if(currentMode==MODE_MANUAL) backward();     r->send(200,"text/plain","OK"); });
  server.on("/left",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=LEFT_TURN;  if(currentMode==MODE_MANUAL) left();         r->send(200,"text/plain","OK"); });
  server.on("/right",      HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=RIGHT_TURN; if(currentMode==MODE_MANUAL) right();        r->send(200,"text/plain","OK"); });
  server.on("/stop",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=STOPPED;    if(currentMode==MODE_MANUAL) stopCar();      r->send(200,"text/plain","OK"); });
  server.on("/fwd_left",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_LEFT;   if(currentMode==MODE_MANUAL) forwardLeft();  r->send(200,"text/plain","OK"); });
  server.on("/fwd_right",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_RIGHT;  if(currentMode==MODE_MANUAL) forwardRight(); r->send(200,"text/plain","OK"); });
  server.on("/back_left",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_LEFT;  if(currentMode==MODE_MANUAL) backwardLeft(); r->send(200,"text/plain","OK"); });
  server.on("/back_right", HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_RIGHT; if(currentMode==MODE_MANUAL) backwardRight();r->send(200,"text/plain","OK"); });
  server.on("/grip/open",  HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripOpen();  r->send(200,"text/plain","OK"); });
  server.on("/grip/close", HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripClose(); r->send(200,"text/plain","OK"); });
  server.on("/speed/lin/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear+SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/lin/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear-SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot+SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot-SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });

  server.begin();
}

void loop() {
  if ((currentMode == MODE_LINE_ONLY || currentMode == MODE_DELIVERY) && is_auto_running) {
    do_line_loop();
  } else {
    if (line_mode) { stopCar(); line_mode = false; }
    delay(5);
  }
}

void applyCurrentMotion(){
  switch(curMotion){
    case FWD:        forward(); break;
    case BWD:        backward(); break;
    case LEFT_TURN:  left(); break;
    case RIGHT_TURN: right(); break;
    case FWD_LEFT:   forwardLeft(); break;
    case FWD_RIGHT:  forwardRight(); break;
    case BACK_LEFT:  backwardLeft(); break;
    case BACK_RIGHT: backwardRight(); break;
    default:         stopCar(); break;
  }
}
void forward() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); analogWrite(ENA, speed_linear); analogWrite(ENB, speed_linear); }
void backward() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_linear); analogWrite(ENB, speed_linear); }
void left() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); analogWrite(ENA, speed_rot); analogWrite(ENB, speed_rot); }
void right() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_rot); analogWrite(ENB, speed_rot); }
void stopCar() { digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); analogWrite(ENA, 0); analogWrite(ENB, 0); }
void forwardLeft() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); if (!INVERT_STEER) { analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); } else { analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); } }
void forwardRight() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); if (!INVERT_STEER) { analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); } else { analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); } }
void backwardLeft() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); }
void backwardRight() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); }
void gripOpen()  { gripper.write(OPEN_ANGLE); }
void gripClose() { gripper.write(CLOSE_ANGLE); }
===
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "do_line.h"
#include "route_interpreter.h"

const char* ssid = "ESP32-Car";
const char* password = "12345678";

#define IN1 12
#define IN2 14
#define ENA 13
#define IN3 4
#define IN4 2
#define ENB 15

int speed_linear = 130;
int speed_rot    = 110;
const int SPEED_MIN  = 60;
const int SPEED_MAX  = 255;
const int SPEED_STEP = 10;
const int DIAG_SCALE = 70; 
static inline int diagScale(int v){ return v * DIAG_SCALE / 100; }
static inline int clamp(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
const bool INVERT_STEER = true; 

#define SERVO_PIN 18
const int OPEN_ANGLE  = 120;
const int CLOSE_ANGLE = 175;
Servo gripper;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= QUẢN LÝ CHẾ ĐỘ =================
volatile UIMode currentMode = MODE_MANUAL;
bool line_mode = false;
bool is_auto_running = false;

int currentPath[20]; 
int pathLength = 0;
int currentTargetNode = -1;
int currentPathIndex = 0;
int currentDir = 1; 

volatile int delivered_count = 0;
volatile float avg_time_sec = 0.0;
volatile int robot_efficiency = 100;

enum Motion { STOPPED, FWD, BWD, LEFT_TURN, RIGHT_TURN, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT };
volatile Motion curMotion = STOPPED;

void forward(); void backward(); void left(); void right(); void stopCar();
void forwardLeft(); void forwardRight(); void backwardLeft(); void backwardRight();
void gripOpen(); void gripClose();
void applyCurrentMotion();

// ================= WebSocket Broadcast =================
void wsBroadcast(const char* msg) {
  ws.textAll(msg);
}

// ================= WebSocket Event Handler =================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch(type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      client->text("{\"type\":\"WELCOME\",\"mode\":\"" + String(currentMode) + "\"}");
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo *info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        handleWsMessage(client, (char*)data);
      }
      break;
    }
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void handleWsMessage(AsyncWebSocketClient *client, char* msg) {
  StaticJsonDocument<1024> doc;
  if (deserializeJson(doc, msg)) return;
  
  const char* type = doc["type"];
  if (!type) return;
  
  if (strcmp(type, "PING") == 0) {
    client->text("{\"type\":\"PONG\"}");
  }
  else if (strcmp(type, "ROUTE") == 0) {
    // Switch to AI route mode
    stopCar();
    do_line_setup();
    currentMode = MODE_AI_ROUTE;
    line_mode = true;
    is_auto_running = true;
    route_setup();
    route_load(msg);
    Serial.println("AI Route mode activated via WebSocket");
  }
  else if (strcmp(type, "STOP") == 0 || strcmp(type, "ESTOP") == 0) {
    stopCar();
    do_line_abort();
    route_abort();
    currentMode = MODE_MANUAL;
    line_mode = false;
    is_auto_running = false;
    client->text("{\"type\":\"ACK\",\"action\":\"STOPPED\"}");
  }
  else if (strcmp(type, "RESUME") == 0) {
    if (currentMode == MODE_AI_ROUTE) {
      do_line_setup();
      line_mode = true;
      is_auto_running = true;
      client->text("{\"type\":\"ACK\",\"action\":\"RESUMED\"}");
    }
  }
}

// ================= Telemetry Timer =================
static unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

void sendTelemetry() {
  if (ws.count() == 0) return; // No clients connected
  if (currentMode == MODE_AI_ROUTE) {
    String json = route_telemetry_json();
    ws.textAll(json);
  }
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2500);
  gripClose();
  stopCar();

  Serial.begin(115200);

  if(!LittleFS.begin(true)){
    Serial.println("Lỗi Mount LittleFS!");
    return;
  }
  
  WiFi.softAP(ssid, password);
  Serial.print("Hotspot IP: "); Serial.println(WiFi.softAPIP());

  // WebSocket setup
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // Route interpreter callback
  route_set_ws_callback(wsBroadcast);

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/setMode", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("m")) { r->send(400,"text/plain","manual"); return; }
    String m = r->getParam("m")->value();

    if (m == "line_only") {
      stopCar();
      do_line_setup();
      currentMode = MODE_LINE_ONLY;
      line_mode = true;
      is_auto_running = true;
    } else if (m == "ai_route") {
      stopCar();
      do_line_setup();
      route_setup();
      currentMode = MODE_AI_ROUTE;
      line_mode = true;
      is_auto_running = true;
    } else if (m == "manual") {
      do_line_abort();
      route_abort();
      stopCar();
      currentMode = MODE_MANUAL;
      line_mode = false;
      is_auto_running = false;
    }
    r->send(200,"text/plain", m);
  });

  server.on("/deliver", HTTP_GET, [](AsyncWebServerRequest* r){
    if (r->hasParam("dir")) currentDir = r->getParam("dir")->value().toInt();
    currentPathIndex = 0; 
    
    if (r->hasParam("path")) {
      String pathStr = r->getParam("path")->value();
      pathLength = 0;
      int startIdx = 0; int commaIdx = pathStr.indexOf(',');
      while (commaIdx != -1 && pathLength < 20) {
        currentPath[pathLength++] = pathStr.substring(startIdx, commaIdx).toInt();
        startIdx = commaIdx + 1; commaIdx = pathStr.indexOf(',', startIdx);
      }
      if (startIdx < pathStr.length() && pathLength < 20) currentPath[pathLength++] = pathStr.substring(startIdx).toInt();
      if(pathLength > 1) currentTargetNode = currentPath[1];
    }
    
    stopCar();
    do_line_setup();
    currentMode = MODE_DELIVERY;
    line_mode = true;
    is_auto_running = true;
    
    r->send(200, "text/plain", "OK");
  });

  server.on("/estop", HTTP_GET, [](AsyncWebServerRequest *r){
    stopCar(); do_line_abort(); route_abort();
    currentMode = MODE_MANUAL; line_mode = false; is_auto_running = false;
    r->send(200, "text/plain", "E-STOP ACTIVATED");
  });

  server.on("/resume", HTTP_GET, [](AsyncWebServerRequest *r){
    if(currentMode != MODE_MANUAL) {
      stopCar(); do_line_setup();
      line_mode = true; is_auto_running = true;
    }
    r->send(200, "text/plain", "RESUMED");
  });

  server.on("/return_home", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send(200, "text/plain", "RETURNING HOME");
  });

  server.on("/api/stats", HTTP_GET, [](AsyncWebServerRequest *r){
    String json = "{";
    json += "\"delivered\":" + String(delivered_count) + ",";
    json += "\"avgTime\":" + String(avg_time_sec) + ",";
    json += "\"efficiency\":" + String(robot_efficiency) + ",";
    json += "\"chart\":{\"labels\":[\"Đơn 1\",\"Đơn 2\",\"Đơn 3\"],\"expected\":[30, 45, 60],\"actual\":[32, 42, 65]}}";
    r->send(200, "application/json", json);
  });

  server.on("/forward",    HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD;        if(currentMode==MODE_MANUAL) forward();      r->send(200,"text/plain","OK"); });
  server.on("/backward",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BWD;        if(currentMode==MODE_MANUAL) backward();     r->send(200,"text/plain","OK"); });
  server.on("/left",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=LEFT_TURN;  if(currentMode==MODE_MANUAL) left();         r->send(200,"text/plain","OK"); });
  server.on("/right",      HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=RIGHT_TURN; if(currentMode==MODE_MANUAL) right();        r->send(200,"text/plain","OK"); });
  server.on("/stop",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=STOPPED;    if(currentMode==MODE_MANUAL) stopCar();      r->send(200,"text/plain","OK"); });
  server.on("/fwd_left",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_LEFT;   if(currentMode==MODE_MANUAL) forwardLeft();  r->send(200,"text/plain","OK"); });
  server.on("/fwd_right",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_RIGHT;  if(currentMode==MODE_MANUAL) forwardRight(); r->send(200,"text/plain","OK"); });
  server.on("/back_left",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_LEFT;  if(currentMode==MODE_MANUAL) backwardLeft(); r->send(200,"text/plain","OK"); });
  server.on("/back_right", HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_RIGHT; if(currentMode==MODE_MANUAL) backwardRight();r->send(200,"text/plain","OK"); });
  server.on("/grip/open",  HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripOpen();  r->send(200,"text/plain","OK"); });
  server.on("/grip/close", HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripClose(); r->send(200,"text/plain","OK"); });
  server.on("/speed/lin/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear+SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/lin/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear-SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot+SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot-SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });

  server.begin();
  Serial.println("Server started. WebSocket on /ws");
}

void loop() {
  // Cleanup disconnected WS clients periodically
  static unsigned long lastCleanup = 0;
  if (millis() - lastCleanup > 1000) { ws.cleanupClients(); lastCleanup = millis(); }

  if (currentMode == MODE_AI_ROUTE && is_auto_running) {
    // AI Route mode — use Route Interpreter state machine
    route_loop();
    // Send telemetry every 200ms
    if (millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
      lastTelemetryMs = millis();
      sendTelemetry();
    }
  } else if ((currentMode == MODE_LINE_ONLY || currentMode == MODE_DELIVERY) && is_auto_running) {
    do_line_loop();
  } else {
    if (line_mode) { stopCar(); line_mode = false; }
    delay(5);
  }
}

void applyCurrentMotion(){
  switch(curMotion){
    case FWD:        forward(); break;
    case BWD:        backward(); break;
    case LEFT_TURN:  left(); break;
    case RIGHT_TURN: right(); break;
    case FWD_LEFT:   forwardLeft(); break;
    case FWD_RIGHT:  forwardRight(); break;
    case BACK_LEFT:  backwardLeft(); break;
    case BACK_RIGHT: backwardRight(); break;
    default:         stopCar(); break;
  }
}
void forward() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); analogWrite(ENA, speed_linear); analogWrite(ENB, speed_linear); }
void backward() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_linear); analogWrite(ENB, speed_linear); }
void left() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); analogWrite(ENA, speed_rot); analogWrite(ENB, speed_rot); }
void right() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_rot); analogWrite(ENB, speed_rot); }
void stopCar() { digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); analogWrite(ENA, 0); analogWrite(ENB, 0); }
void forwardLeft() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); if (!INVERT_STEER) { analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); } else { analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); } }
void forwardRight() { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); if (!INVERT_STEER) { analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); } else { analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); } }
void backwardLeft() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, diagScale(speed_linear)); analogWrite(ENB, speed_linear); }
void backwardRight() { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); analogWrite(ENA, speed_linear); analogWrite(ENB, diagScale(speed_linear)); }
void gripOpen()  { gripper.write(OPEN_ANGLE); }
void gripClose() { gripper.write(CLOSE_ANGLE); }
void gripClose() { gripper.write(CLOSE_ANGLE); }
```

Thay đổi chính:
- `AsyncWebSocket ws("/ws")` — WebSocket server trên ESP32
- `onWsEvent()` — Handle CONNECT/DISCONNECT/DATA
- `handleWsMessage()` — Parse JSON: ROUTE → `route_load()`, STOP, RESUME, PING/PONG
- `MODE_AI_ROUTE` — Gọi `route_loop()` + telemetry 200ms
- `wsBroadcast()` — Callback cho route_interpreter gửi messages
- `ws.cleanupClients()` — Dọn dẹp WS clients mỗi giây

### ✅ Firmware — Route Interpreter Bug Fixes

```diff:route_interpreter.cpp
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
extern float readDistanceCM_filtered();
extern void driveWheelLeft(float v_target, int pwm);
extern void driveWheelRight(float v_target, int pwm);
extern int pidStep(struct PID &pid, float v_target, float v_meas, float dt_s);

// External encoder variables from do_line.cpp
extern volatile long encL_count;
extern volatile long encR_count;
extern volatile long encL_total;
extern volatile long encR_total;

// External PID structs
extern struct PID pidL;
extern struct PID pidR;

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

// WebSocket send callback (set by main.ino)
typedef void (*WsSendFn)(const char* msg);
static WsSendFn wsSendCallback = nullptr;

void route_set_ws_callback(WsSendFn fn) {
  wsSendCallback = fn;
}

// ==================== Utility ====================
inline bool rt_onLine(int pin) { return digitalRead(pin) == LOW; }
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
  rt_obstacleDist = readDistanceCM_filtered();
  return rt_obstacleDist;
}

// ==================== Compute grid position ====================
// Based on initial direction (down = +row) and command history
struct GridPos {
  int row;
  int col;
  int dir; // 0=down, 1=up, 2=right, 3=left
};

static GridPos robotGridPos = {0, 0, 0};

void rt_initGridPos(int startRow, int startCol, int startDir) {
  robotGridPos.row = startRow;
  robotGridPos.col = startCol;
  robotGridPos.dir = startDir;
}

GridPos rt_getNextGridPos(GridPos pos) {
  GridPos next = pos;
  int dRow[] = {1, -1, 0, 0}; // down, up, right, left
  int dCol[] = {0, 0, 1, -1};
  next.row += dRow[pos.dir];
  next.col += dCol[pos.dir];
  return next;
}

void rt_updateDirAfterTurn(char cmd) {
  // Right turn mapping: down->left, left->up, up->right, right->down
  // Left turn mapping: down->right, right->up, up->left, left->down
  if (cmd == 'R') {
    int rightMap[] = {3, 2, 0, 1}; // down->left, up->right, right->down, left->up
    robotGridPos.dir = rightMap[robotGridPos.dir];
  } else if (cmd == 'L') {
    int leftMap[] = {2, 3, 1, 0}; // down->right, up->left, right->up, left->down
    robotGridPos.dir = leftMap[robotGridPos.dir];
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
  extern const float CIRC;
  extern const int PPR_EFFECTIVE;
  rt_distance = ((float)(totalL + totalR) / 2.0f / PPR_EFFECTIVE) * CIRC * 100.0f; // cm
  
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
        
        // Calculate obstacle grid position
        GridPos obsPos = rt_getNextGridPos(robotGridPos);
        
        // Send obstacle notification
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
        // Rising edge: entered intersection
        motorsStop();
        delay(50);
        
        // Move forward slightly to center on intersection (~2cm)
        move_forward_distance(0.02, 100);
        motorsStop();
        delay(50);
        
        currentState = RS_AT_INTERSECTION;
        wasIntersection = true;
        break;
      }
      wasIntersection = isInter;
      
      // Normal PID line following
      rt_pidLineFollow();
      break;
    }
    
    // ========== AT INTERSECTION ==========
    case RS_AT_INTERSECTION: {
      if (cmdQueueEmpty()) {
        // No more commands -> DONE
        currentState = RS_DONE;
        break;
      }
      
      char cmd = cmdQueuePop();
      Serial.printf("Intersection #%d: command '%c' (step %d/%d)\n", 
                     intersectionCount + 1, cmd, cmdExecuted, cmdTotal);
      
      // Send progress update
      String msg = "{\"type\":\"TELEMETRY\",\"state\":\"AT_INTERSECTION\","
                   "\"step\":" + String(cmdExecuted) + ",\"total\":" + String(cmdTotal) + ","
                   "\"command\":\"" + String(cmd) + "\"}";
      rt_wsSend(msg);
      
      switch (cmd) {
        case 'F':
          // Go straight through intersection
          rt_advanceGridPos();
          break;
        case 'L':
          // Turn left 90 degrees
          spin_left_deg(90.0, 180);
          motorsStop();
          delay(100);
          rt_updateDirAfterTurn('L');
          rt_advanceGridPos();
          break;
        case 'R':
          // Turn right 90 degrees
          spin_right_deg(90.0, 180);
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
      
      // Check if that was the last command
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
      
      // Wait for new route from web (or timeout)
      if (millis() - obstacleTimer > OBSTACLE_TIMEOUT_MS) {
        Serial.println("Obstacle timeout! Entering safe mode.");
        rt_wsSend("{\"type\":\"OBSTACLE_TIMEOUT\"}");
        currentState = RS_IDLE;
      }
      // Otherwise stay in OBSTACLE state, waiting for route_load() to be called
      break;
    }
    
    // ========== REROUTING ==========
    case RS_REROUTING: {
      // This state is handled by route_load() which transitions to FOLLOWING_LINE
      // Should not normally stay here
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
===
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

// WebSocket send callback (set by main.ino)
typedef void (*WsSendFn)(const char* msg);
static WsSendFn wsSendCallback = nullptr;

void route_set_ws_callback(WsSendFn fn) {
  wsSendCallback = fn;
}

// ==================== Utility ====================
inline bool rt_onLine(int pin) { return digitalRead(pin) == HIGH; }  // Match LINE_DETECT_STATE in do_line.cpp
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
// Based on initial direction (down = +row) and command history
struct GridPos {
  int row;
  int col;
  int dir; // 0=down(+Y), 1=right(+X), 2=up(-Y), 3=left(-X)  — matches do_line.cpp
};

static GridPos robotGridPos = {0, 0, 0};

void rt_initGridPos(int startRow, int startCol, int startDir) {
  robotGridPos.row = startRow;
  robotGridPos.col = startCol;
  robotGridPos.dir = startDir;
}

GridPos rt_getNextGridPos(GridPos pos) {
  GridPos next = pos;
  int dRow[] = {1, 0, -1, 0}; // 0=down(+row), 1=right(+col), 2=up(-row), 3=left(-col)
  int dCol[] = {0, 1, 0, -1};
  next.row += dRow[pos.dir];
  next.col += dCol[pos.dir];
  return next;
}

void rt_updateDirAfterTurn(char cmd) {
  // dir: 0=down, 1=right, 2=up, 3=left
  // Right turn: (dir+1)%4, Left turn: (dir+3)%4
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
  const float RT_CIRC = 2.0f * 3.1415926f * 0.0325f;  // same as CIRC in do_line.cpp
  const int RT_PPR = 60;  // same as PPR_EFFECTIVE in do_line.cpp
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
        
        // Calculate obstacle grid position
        GridPos obsPos = rt_getNextGridPos(robotGridPos);
        
        // Send obstacle notification
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
        // Rising edge: entered intersection
        motorsStop();
        delay(50);
        
        // Move forward slightly to center on intersection (~2cm)
        move_forward_distance(0.02, 100);
        motorsStop();
        delay(50);
        
        currentState = RS_AT_INTERSECTION;
        wasIntersection = true;
        break;
      }
      wasIntersection = isInter;
      
      // Normal PID line following
      rt_pidLineFollow();
      break;
    }
    
    // ========== AT INTERSECTION ==========
    case RS_AT_INTERSECTION: {
      if (cmdQueueEmpty()) {
        // No more commands -> DONE
        currentState = RS_DONE;
        break;
      }
      
      char cmd = cmdQueuePop();
      Serial.printf("Intersection #%d: command '%c' (step %d/%d)\n", 
                     intersectionCount + 1, cmd, cmdExecuted, cmdTotal);
      
      // Send progress update
      String msg = "{\"type\":\"TELEMETRY\",\"state\":\"AT_INTERSECTION\","
                   "\"step\":" + String(cmdExecuted) + ",\"total\":" + String(cmdTotal) + ","
                   "\"command\":\"" + String(cmd) + "\"}";
      rt_wsSend(msg);
      
      switch (cmd) {
        case 'F':
          // Go straight through intersection
          rt_advanceGridPos();
          break;
        case 'L':
          // Turn left 90 degrees
          spin_left_deg(90.0, 180);
          motorsStop();
          delay(100);
          rt_updateDirAfterTurn('L');
          rt_advanceGridPos();
          break;
        case 'R':
          // Turn right 90 degrees
          spin_right_deg(90.0, 180);
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
      
      // Check if that was the last command
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
      
      // Wait for new route from web (or timeout)
      if (millis() - obstacleTimer > OBSTACLE_TIMEOUT_MS) {
        Serial.println("Obstacle timeout! Entering safe mode.");
        rt_wsSend("{\"type\":\"OBSTACLE_TIMEOUT\"}");
        currentState = RS_IDLE;
      }
      // Otherwise stay in OBSTACLE state, waiting for route_load() to be called
      break;
    }
    
    // ========== REROUTING ==========
    case RS_REROUTING: {
      // This state is handled by route_load() which transitions to FOLLOWING_LINE
      // Should not normally stay here
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
```

| Bug | Fix |
|-----|-----|
| `rt_onLine()` dùng `LOW` | → `HIGH` (khớp `LINE_DETECT_STATE`) |
| `readDistanceCM_filtered()` không tồn tại | → `readDistanceCM_Fast()` |
| Direction mapping sai (0=down,1=up,2=right,3=left) | → 0=down,1=right,2=up,3=left |
| Turn logic bằng lookup table phức tạp | → `(dir+1)%4` / `(dir+3)%4` |
| `extern CIRC/PPR_EFFECTIVE` linkage error | → Local constants `RT_CIRC/RT_PPR` |
| `struct PID` duplicate definition | → Moved to shared `do_line.h` |

### ✅ Firmware — do_line.h Updated

```diff:do_line.h
#pragma once
#include <Arduino.h>

// Định nghĩa 3 chế độ hoạt động của xe
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2 };

// Biến khóa an toàn: Chỉ khi = true thì xe mới được phép lăn bánh
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void motorsStop();
===
#pragma once
#include <Arduino.h>

// Shared PID structure (used by do_line.cpp and route_interpreter.cpp)
struct PID { float Kp, Ki, Kd; float i_term; float prev_err; float out_min, out_max; };

// 4 operating modes
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2, MODE_AI_ROUTE=3 };

// Safety lock: robot only moves when true
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void motorsStop();
void motorsStop();
```

- Added `MODE_AI_ROUTE = 3`
- Moved `struct PID` to header for shared use

---

## Files Changed

| File | Action | Lines |
|------|--------|-------|
| [style.css](file:///d:/hk2nam2/ai/ai-final_project/web/style.css) | Added animation CSS | +147 |
| [script.js](file:///d:/hk2nam2/ai/ai-final_project/web/script.js) | Complete rewrite | ~490 lines |
| [index.html](file:///d:/hk2nam2/ai/ai-final_project/web/index.html) | Added controls/tools | +27 |
| [main.ino](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/main.ino) | WebSocket + Route | Rewritten |
| [do_line.h](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.h) | Shared PID + mode | Updated |
| [do_line.cpp](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp) | Removed PID struct | -1 |
| [route_interpreter.cpp](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/route_interpreter.cpp) | 6 bug fixes | Updated |

## Verified

- ✅ Web mở browser: Dashboard hiển thị đúng, 4 tools, animation controls
- ✅ Thuật toán A* mô phỏng đúng (7 bước, 7 nodes explored cho 15→9)
- ⚠️ WebSocket: Lỗi kết nối là bình thường (chưa có ESP32 thật)
- ⏳ Cần flash firmware lên ESP32 để test end-to-end
