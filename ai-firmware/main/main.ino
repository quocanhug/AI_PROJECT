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
bool is_auto_running = false; // <--- KHÓA AN TOÀN TRẠNG THÁI CHẠY

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

  // API Cập nhật chế độ (Chỉ setup, không cho chạy bậy)
  server.on("/setMode", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("m")) { r->send(400,"text/plain","manual"); return; }
    String m = r->getParam("m")->value();

    if (m == "line_only") {
      stopCar();
      do_line_setup();
      currentMode = MODE_LINE_ONLY;
      line_mode = true;
      is_auto_running = true; // <--- MỞ KHÓA: User đã bấm nút Bắt đầu dò line
    } else if (m == "manual") {
      do_line_abort();
      stopCar();
      currentMode = MODE_MANUAL;
      line_mode = false;
      is_auto_running = false; // <--- ĐÓNG KHÓA
    }
    r->send(200,"text/plain", m);
  });

  // API Nạp lộ trình
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
    is_auto_running = true; // <--- MỞ KHÓA: User đã nạp lộ trình và yêu cầu chạy
    
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

  // Moves
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
  // Chỉ gọi hàm tự động khi đã đúng chế độ VÀ người dùng đã bấm nút Bắt Đầu
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