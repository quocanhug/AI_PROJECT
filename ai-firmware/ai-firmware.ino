#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
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
AsyncWebSocket ws("/ws");

// ================= QUẢN LÝ CHẾ ĐỘ =================
volatile UIMode currentMode = MODE_MANUAL;
bool line_mode = false;
bool is_auto_running = false;

int currentPath[15]; 
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
void handleWsMessage(AsyncWebSocketClient* client, char* msg);  // Forward decl — tránh lỗi compile khi gọi trước khi định nghĩa

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
        // ★ BUG FIX #1 — WS buffer overflow:
        //   data[len]=0 có thể OOB vì buffer có đúng 'len' byte (không có room cho null).
        //   Dùng stack buffer cố định 512B — đủ chứa mọi message trong protocol này.
        char msgBuf[512];
        size_t copyLen = (len < 511) ? len : 511;
        memcpy(msgBuf, data, copyLen);
        msgBuf[copyLen] = '\0';
        handleWsMessage(client, msgBuf);
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
    // Nạp path node trực tiếp vào currentPath[] — dùng do_line_loop() đã ổn định
    stopCar();
    
    // Parse path array từ JSON: [0, 1, 6, 7, 12, ...]
    JsonArray pathArr = doc["path"].as<JsonArray>();
    pathLength = 0;
    for (JsonVariant v : pathArr) {
      // ★ BUG FIX #8 — Node validation: chặn node ngoài range [0,14] để tránh OOB access
      int node = v.as<int>();
      if (node >= 0 && node < 15 && pathLength < 15) currentPath[pathLength++] = node;
    }
    currentPathIndex = 0;
    
    // Tính hướng ban đầu từ 2 node đầu tiên
    if (doc.containsKey("initialDir")) {
      currentDir = doc["initialDir"] | 0;
    } else if (pathLength >= 2) {
      // Tự tính từ node_coords trong do_line.cpp
      extern int getTargetDirection(int, int);
      currentDir = getTargetDirection(currentPath[0], currentPath[1]);
    }
    
    do_line_setup();
    currentMode = MODE_AI_ROUTE;
    line_mode = true;
    is_auto_running = true;
    
    // Gửi ACK về Web
    String ack = "{\"type\":\"ROUTE_ACK\",\"commands\":" + String(pathLength) + "}";
    ws.textAll(ack);
    Serial.printf("AI Route: %d nodes loaded, dir=%d\n", pathLength, currentDir);
  }
  else if (strcmp(type, "STOP") == 0 || strcmp(type, "ESTOP") == 0) {
    stopCar();
    do_line_abort();
    currentMode = MODE_MANUAL;
    line_mode = false;
    is_auto_running = false;
    // ★ BUG FIX #9 — ESTOP xóa path hẳn (không cho resume sau emergency stop)
    //   STOP thường giữ path để có thể resume. ESTOP là khẩn cấp → xóa sạch.
    if (strcmp(type, "ESTOP") == 0) {
      pathLength = 0;
      currentPathIndex = 0;
    }
    client->text("{\"type\":\"ACK\",\"action\":\"STOPPED\"}");
  }
  else if (strcmp(type, "RESUME") == 0) {
    // ★ BUG FIX #7 — RESUME logic sai:
    //   Check cũ: (currentMode == MODE_AI_ROUTE) → false sau STOP vì mode đã về MANUAL!
    //   Check mới: (pathLength > 0) — có route là có thể resume, bất kể mode hiện tại.
    // ★ BUG FIX #2 — RESUME reset route:
    //   Gọi do_line_resume() thay vì do_line_setup() — giữ nguyên lastConfirmedNodeIdx
    //   và currentDir, chỉ bật lại motor/PID và căn chỉnh hướng.
    if (pathLength > 0) {
      do_line_resume();
      currentMode = MODE_AI_ROUTE;
      line_mode = true;
      is_auto_running = true;
      client->text("{\"type\":\"ACK\",\"action\":\"RESUMED\"}");
    }
  }
}

// ================= Telemetry Timer =================
static unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

// ★ Extern sensor/speed/encoder data từ do_line.cpp để gửi telemetry đầy đủ
extern float us_dist_cm;
extern float vL_ema, vR_ema;
extern volatile long encL_total, encR_total;  // ★ Dùng tính quãng đường (cm)

// Sensor pins (dùng đọc trạng thái cho telemetry)
#define TELE_L2 34
#define TELE_L1 32
#define TELE_M  33
#define TELE_R1 27
#define TELE_R2 25

void sendTelemetry() {
  if (ws.count() == 0) return;
  if (currentMode == MODE_AI_ROUTE && is_auto_running) {
    // ★ BUG FIX M1: currentPathIndex trỏ vào node ĐANG TIẾN TỚI, không phải node đang đứng.
    //   Dùng lastConfirmedNodeIdx để lấy node robot thực sự đang đứng trên.
    extern int lastConfirmedNodeIdx;
    int robotNode = (lastConfirmedNodeIdx < pathLength) ? currentPath[lastConfirmedNodeIdx] : currentPath[pathLength-1];
    
    // ★ Đọc cảm biến line (LOW = trên vạch)
    int s0 = digitalRead(TELE_L2) == LOW ? 1 : 0;
    int s1 = digitalRead(TELE_L1) == LOW ? 1 : 0;
    int s2 = digitalRead(TELE_M)  == LOW ? 1 : 0;
    int s3 = digitalRead(TELE_R1) == LOW ? 1 : 0;
    int s4 = digitalRead(TELE_R2) == LOW ? 1 : 0;
    
    // ★ BUG FIX #5 — Distance hardcoded:
    //   20.42f = CIRC*100, 60 = PPR_EFFECTIVE (macro trong do_line.cpp, không extern được)
    //   extern CIRC để tự động đúng khi thay bánh / cậu encoder.
    extern const float CIRC;  // = 2*pi*WHEEL_RADIUS_M, định nghĩa trong do_line.cpp
    float dist_cm = ((float)(encL_total + encR_total) / 2.0f) / 60.0f * CIRC * 100.0f;

    String json = "{\"type\":\"TELEMETRY\",\"state\":\"FOLLOWING_LINE\"";
    json += ",\"step\":" + String(currentPathIndex);
    json += ",\"total\":" + String(pathLength);
    json += ",\"robotNode\":" + String(robotNode);
    json += ",\"robotDir\":" + String(currentDir);
    json += ",\"speedL\":" + String(vL_ema, 3);
    json += ",\"speedR\":" + String(vR_ema, 3);
    json += ",\"distance\":" + String(dist_cm, 1);
    json += ",\"obstacle\":" + String(us_dist_cm, 1);
    json += ",\"sensors\":[" + String(s0) + "," + String(s1) + "," + String(s2) + "," + String(s3) + "," + String(s4) + "]}";
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
      currentMode = MODE_AI_ROUTE;
      line_mode = true;
      is_auto_running = true;
    } else if (m == "manual") {
      do_line_abort();
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
      while (commaIdx != -1 && pathLength < 15) {
        currentPath[pathLength++] = pathStr.substring(startIdx, commaIdx).toInt();
        startIdx = commaIdx + 1; commaIdx = pathStr.indexOf(',', startIdx);
      }
      if (startIdx < pathStr.length() && pathLength < 15) currentPath[pathLength++] = pathStr.substring(startIdx).toInt();
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
    stopCar(); do_line_abort();
    currentMode = MODE_MANUAL; line_mode = false; is_auto_running = false;
    r->send(200, "text/plain", "E-STOP ACTIVATED");
  });

  server.on("/resume", HTTP_GET, [](AsyncWebServerRequest *r){
    // ★ BUG FIX #7 — Chỉ cần pathLength > 0 (sau STOP mode là MANUAL, không phải AI_ROUTE)
    // ★ BUG FIX #2 — Dùng do_line_resume() thành do_line_setup() để giữ route state
    if (pathLength > 0) {
      do_line_resume();
      currentMode = MODE_AI_ROUTE;
      line_mode = true;
      is_auto_running = true;
      Serial.printf("[RESUME] Resuming from node %d, dir=%d\n", currentPath[currentPathIndex], currentDir);
      r->send(200, "application/json",
        "{\"status\":\"RESUMED\",\"pathIndex\":" + String(currentPathIndex) +
        ",\"robotDir\":" + String(currentDir) + "}");
    } else {
      r->send(200, "application/json", "{\"status\":\"NO_ROUTE\"}");
    }
  });

  server.on("/return_home", HTTP_GET, [](AsyncWebServerRequest *r){
    // Dừng robot và trả vị trí hiện tại để web tính đường về
    stopCar(); do_line_abort();
    int robotNode = (currentPathIndex < pathLength) ? currentPath[currentPathIndex] : 0;
    currentMode = MODE_MANUAL;
    line_mode = false;
    is_auto_running = false;
    Serial.printf("[RETURN_HOME] Robot at node %d, dir %d\n", robotNode, currentDir);
    r->send(200, "application/json",
      "{\"status\":\"STOPPED\",\"robotNode\":" + String(robotNode) + 
      ",\"robotDir\":" + String(currentDir) + "}");
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

  if ((currentMode == MODE_LINE_ONLY || currentMode == MODE_DELIVERY || currentMode == MODE_AI_ROUTE) && is_auto_running) {
    do_line_loop();
    // Gửi telemetry cho AI Route
    if (currentMode == MODE_AI_ROUTE && millis() - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
      lastTelemetryMs = millis();
      sendTelemetry();
    }
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