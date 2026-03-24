
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>
#include "do_line.h"
#include "route_interpreter.h"

// ================= WiFi AP =================
const char* ssid = "ESP32-Car";
const char* password = "12345678";

// ================= Motor pins =================
// Left motor
#define IN1 12
#define IN2 14
#define ENA 13
// Right motor
#define IN3 4
#define IN4 2
#define ENB 15

// ================= Speed =================
int speed_linear = 130;
int speed_rot    = 110;
const int SPEED_MIN  = 60;
const int SPEED_MAX  = 255;
const int SPEED_STEP = 10;

// Giảm tốc bánh phía “bên trong cua” khi đi chéo (0–100%)
const int DIAG_SCALE = 70; // 70% -> cua mượt
static inline int diagScale(int v){ return v * DIAG_SCALE / 100; }
static inline int clamp(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }

// ============ Đảo hướng steer tiến ============
const bool INVERT_STEER = true; // true: forward-left giảm bánh PHẢI

// ================= Gripper Servo =================
#define SERVO_PIN 18
const int OPEN_ANGLE  = 120;
const int CLOSE_ANGLE = 175;
Servo gripper;

// ================= Server & WebSocket =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= Mode =================
enum UIMode { MODE_MANUAL=0, MODE_LINE=1, MODE_AI_ROUTE=2 };
volatile UIMode currentMode = MODE_MANUAL;
static bool lineInited = false;
bool line_mode = false;

// Telemetry timer
unsigned long telemetryTimer = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 200;

// ================= WebSocket Send Callback =================
void wsSendToAll(const char* msg) {
  ws.textAll(msg);
}

// ================= WebSocket Event Handler =================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WS client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    // Send current state
    String hello = "{\"type\":\"HELLO\",\"mode\":\"";
    if (currentMode == MODE_MANUAL) hello += "manual";
    else if (currentMode == MODE_LINE) hello += "line";
    else hello += "ai_route";
    hello += "\"}";
    client->text(hello);
  }
  else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WS client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      // Null-terminate the data
      char* json = new char[len + 1];
      memcpy(json, data, len);
      json[len] = '\0';
      
      Serial.printf("WS recv: %s\n", json);
      
      // Parse JSON to determine message type
      StaticJsonDocument<1024> doc;
      DeserializationError err = deserializeJson(doc, json);
      if (!err) {
        const char* msgType = doc["type"] | "";
        
        if (strcmp(msgType, "ROUTE") == 0) {
          // Received route from web
          if (currentMode != MODE_AI_ROUTE) {
            // Switch to AI route mode
            stopCar();
            do_line_setup();
            route_setup();
            currentMode = MODE_AI_ROUTE;
          }
          route_load(json);
          Serial.println("AI Route mode activated");
        }
        else if (strcmp(msgType, "ABORT") == 0) {
          // Abort current route
          route_abort();
          stopCar();
          currentMode = MODE_MANUAL;
          ws.textAll("{\"type\":\"ABORTED\"}");
          Serial.println("Route aborted by user");
        }
        else if (strcmp(msgType, "RESUME") == 0) {
          // Resume after obstacle cleared
          if (currentMode == MODE_AI_ROUTE) {
            route_load(json); // Load new route if provided
          }
        }
        else if (strcmp(msgType, "SET_MODE") == 0) {
          const char* mode = doc["mode"] | "manual";
          if (strcmp(mode, "manual") == 0) {
            route_abort();
            stopCar();
            currentMode = MODE_MANUAL;
          } else if (strcmp(mode, "line") == 0) {
            stopCar();
            do_line_setup();
            currentMode = MODE_LINE;
            line_mode = true;
          }
        }
      }
      delete[] json;
    }
  }
}

// ================= UI =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="vi">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Car Control</title>
<style>
  :root{--bg:#0f172a;--card:#111827;--muted:#94a3b8;--txt:#e5e7eb;--acc:#22c55e;--rot:#3b82f6;--stop:#ef4444;--amber:#f59e0b;}
  *{box-sizing:border-box} html,body{height:100%}
  body{margin:0;font-family:ui-sans-serif,system-ui,Arial;background:radial-gradient(1200px 800px at 50% -10%, #1f2937 0%, var(--bg) 60%);
       color:var(--txt);display:flex;align-items:center;justify-content:center;padding:16px}
  .card{width:min(520px,100%);background:linear-gradient(180deg,#0b1220 0%, var(--card) 100%);
        border:1px solid #1f2937;border-radius:16px;padding:18px 16px;box-shadow:0 10px 30px rgba(0,0,0,.35)}
  h1{margin:0 0 2px;font-size:22px}
  .muted{color:var(--muted);font-size:12px;margin-bottom:12px}
  .row{display:flex;gap:10px;align-items:center;margin-bottom:10px}
  select{background:#0b1220;color:var(--txt);border:1px solid #1f2937;border-radius:10px;padding:8px 10px;font-size:14px}
  .badge{padding:6px 10px;border-radius:999px;border:1px solid #1f2937;background:#0b1220;font-size:12px}
  .controls{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:8px}
  .btn{
    appearance:none;border:0;border-radius:12px;padding:16px 8px;font-size:18px;font-weight:600;color:#0b1220;cursor:pointer;
    background:linear-gradient(180deg,#e5e7eb,#cbd5e1);box-shadow:0 4px 0 rgba(0,0,0,.25);transition:transform .05s,filter .15s,box-shadow .15s;width:100%;
    user-select:none;-webkit-user-select:none;touch-action:none;
  }
  .btn.acc{background:linear-gradient(180deg,#34d399,#22c55e)}
  .btn.rot{background:linear-gradient(180deg,#93c5fd,#3b82f6)}
  .btn.stop{background:linear-gradient(180deg,#fb7185,#ef4444);color:#fff}
  .btn.grip{background:linear-gradient(180deg,#fcd34d,#f59e0b)}
  .btn.active{transform:translateY(2px);box-shadow:0 2px 0 rgba(0,0,0,.25);filter:brightness(1.03)}
  .footer{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
  .speed{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px;align-items:center}
  .pill{grid-column:1/-1;text-align:center;background:#0b1220;border:1px solid #1f2937;border-radius:999px;padding:8px 10px;font-size:14px}
  .overlay{position:fixed;inset:0;background:rgba(0,0,0,.35);display:none;align-items:center;justify-content:center;pointer-events:none}
  .overlay.show{display:flex}
  .overlay .box{background:#0b1220;border:1px solid #1f2937;border-radius:12px;padding:12px 14px;color:var(--txt);font-size:14px}
</style>
</head>
<body>
  <div class="card">
    <h1>ESP32 Car</h1>
    <div class="muted">Hold to drive. Release to stop. Multi-touch enabled.</div>

    <div class="row">
      <label for="modeSel">Chế độ:</label>
      <select id="modeSel">
        <option value="manual">Manual</option>
        <option value="line">Line follow</option>
      </select>
      <span id="modeBadge" class="badge">mode: manual</span>
    </div>

    <div class="controls">
      <button class="btn acc hold" data-path="/fwd_left">↖</button>
      <button class="btn acc hold" data-path="/forward">↑</button>
      <button class="btn acc hold" data-path="/fwd_right">↗</button>
      <button class="btn rot hold" data-path="/left">←</button>
      <button class="btn stop" id="stopBtn" data-path="/stop">■</button>
      <button class="btn rot hold" data-path="/right">→</button>
      <button class="btn acc hold" data-path="/back_left">↙</button>
      <button class="btn acc hold" data-path="/backward">↓</button>
      <button class="btn acc hold" data-path="/back_right">↘</button>
    </div>

    <div class="footer">
      <button class="btn grip" data-path="/grip/open">OPEN</button>
      <button class="btn grip" data-path="/grip/close">CLOSE</button>
    </div>

    <div class="speed">
      <div class="pill" id="spdText">Lin: --  |  Rot: --</div>
      <button class="btn spd" data-path="/speed/lin/down">Lin −</button>
      <button class="btn spd" data-path="/speed/lin/up">Lin +</button>
      <button class="btn spd" data-path="/speed/rot/down">Rot −</button>
      <button class="btn spd" data-path="/speed/rot/up">Rot +</button>
    </div>
  </div>

  <div id="overlay" class="overlay"><div class="box">Đang ở chế độ Line follow. Điều khiển tay bị khóa.</div></div>

<script>
  document.addEventListener('contextmenu', e=>e.preventDefault());
  const overlay = document.getElementById('overlay');
  const modeSel = document.getElementById('modeSel');
  const modeBadge = document.getElementById('modeBadge');

  function uiLock(isLocked){
    overlay.classList.toggle('show', isLocked);
    document.querySelectorAll('.hold, .spd, .grip, #stopBtn')
      .forEach(b => b.disabled = isLocked);
  }
  async function send(path){ try{ await fetch(path); }catch(e){} }
  async function refreshSpeed(){
    try{ const r=await fetch('/speed'); document.getElementById('spdText').textContent=await r.text(); }catch(e){}
  }
  async function refreshMode(){
    try{
      const r = await fetch('/getMode'); const m = await r.text();
      modeSel.value = m; modeBadge.textContent = 'mode: ' + m;
      uiLock(m !== 'manual');
    }catch(e){}
  }
  modeSel.addEventListener('change', async ()=>{
    try{
      const r = await fetch('/setMode?m=' + modeSel.value);
      const m = await r.text();
      modeBadge.textContent = 'mode: ' + m;
      uiLock(m !== 'manual');
    }catch(e){}
  });

  let activeHold = { btn:null, pointerId:null };
  function guardManual(handler){
    return function(e){
      if (modeSel.value !== 'manual') { e.preventDefault(); return; }
      return handler(e);
    }
  }
  document.querySelectorAll('.hold').forEach(btn=>{
    btn.addEventListener('pointerdown', guardManual(e=>{
      e.preventDefault();
      activeHold = { btn, pointerId: e.pointerId };
      btn.classList.add('active');
      btn.setPointerCapture(e.pointerId);
      send(btn.dataset.path);
    }), {passive:false});
    const release = guardManual(e=>{
      e.preventDefault();
      if (activeHold.btn === btn && activeHold.pointerId === e.pointerId) {
        btn.classList.remove('active');
        send('/stop');
        activeHold = { btn:null, pointerId:null };
      }
      try{ btn.releasePointerCapture(e.pointerId); }catch(_){}
    });
    btn.addEventListener('pointerup', release, {passive:false});
    btn.addEventListener('pointercancel', release, {passive:false});
    btn.addEventListener('pointerleave', release, {passive:false});
  });

  document.getElementById('stopBtn').addEventListener('pointerdown', guardManual(e=>{
    e.preventDefault(); send('/stop');
  }), {passive:false});

  document.querySelectorAll('.spd').forEach(b=>{
    b.addEventListener('pointerdown', guardManual(async e=>{
      e.preventDefault(); await send(b.dataset.path); refreshSpeed();
    }), {passive:false});
  });

  document.querySelectorAll('.grip').forEach(b=>{
    b.addEventListener('pointerdown', guardManual(e=>{ e.preventDefault(); send(b.dataset.path); }), {passive:false});
  });

  refreshMode();
  refreshSpeed();
</script>
</body>
</html>
)rawliteral";

// ================= Motion state =================
enum Motion { STOPPED, FWD, BWD, LEFT_TURN, RIGHT_TURN, FWD_LEFT, FWD_RIGHT, BACK_LEFT, BACK_RIGHT };
volatile Motion curMotion = STOPPED;

// ======= Prototypes
void forward(); void backward(); void left(); void right(); void stopCar();
void forwardLeft(); void forwardRight(); void backwardLeft(); void backwardRight();
void gripOpen(); void gripClose();
void applyCurrentMotion();

// ================= Setup =================
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  gripper.setPeriodHertz(50);
  gripper.attach(SERVO_PIN, 500, 2500);
  gripClose();
  stopCar();

  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.print("Hotspot IP: "); Serial.println(WiFi.softAPIP());

  // ---- WebSocket setup ----
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  
  // Set the WS callback for route_interpreter
  extern void route_set_ws_callback(void (*fn)(const char*));
  route_set_ws_callback(wsSendToAll);

  // UI (manual control page)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req){ req->send_P(200, "text/html", index_html); });

  // Mode APIs
  server.on("/getMode", HTTP_GET, [](AsyncWebServerRequest* r){
    const char* m = "manual";
    if (currentMode == MODE_LINE) m = "line";
    else if (currentMode == MODE_AI_ROUTE) m = "ai_route";
    r->send(200, "text/plain", m);
  });

  server.on("/setMode", HTTP_GET, [](AsyncWebServerRequest* r){
    if (!r->hasParam("m")) { r->send(400,"text/plain","manual"); return; }
    String m = r->getParam("m")->value();

    if (m=="line") {
      stopCar();
      do_line_setup();
      currentMode = MODE_LINE;
      line_mode = true;
    } else if (m=="ai_route") {
      stopCar();
      do_line_setup();
      route_setup();
      currentMode = MODE_AI_ROUTE;
    } else {
      route_abort();
      stopCar();
      currentMode = MODE_MANUAL;
    }
    const char* modeStr = "manual";
    if (currentMode == MODE_LINE) modeStr = "line";
    else if (currentMode == MODE_AI_ROUTE) modeStr = "ai_route";
    r->send(200,"text/plain", modeStr);
  });

  // Moves (Manual only)
  server.on("/forward",    HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD;        if(currentMode==MODE_MANUAL) forward();      r->send(200,"text/plain","OK"); });
  server.on("/backward",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BWD;        if(currentMode==MODE_MANUAL) backward();     r->send(200,"text/plain","OK"); });
  server.on("/left",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=LEFT_TURN;  if(currentMode==MODE_MANUAL) left();         r->send(200,"text/plain","OK"); });
  server.on("/right",      HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=RIGHT_TURN; if(currentMode==MODE_MANUAL) right();        r->send(200,"text/plain","OK"); });
  server.on("/stop",       HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=STOPPED;    if(currentMode==MODE_MANUAL) stopCar();      r->send(200,"text/plain","OK"); });

  // Diagonals (Manual only)
  server.on("/fwd_left",   HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_LEFT;   if(currentMode==MODE_MANUAL) forwardLeft();  r->send(200,"text/plain","OK"); });
  server.on("/fwd_right",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=FWD_RIGHT;  if(currentMode==MODE_MANUAL) forwardRight(); r->send(200,"text/plain","OK"); });
  server.on("/back_left",  HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_LEFT;  if(currentMode==MODE_MANUAL) backwardLeft(); r->send(200,"text/plain","OK"); });
  server.on("/back_right", HTTP_GET, [](AsyncWebServerRequest *r){ curMotion=BACK_RIGHT; if(currentMode==MODE_MANUAL) backwardRight();r->send(200,"text/plain","OK"); });

  // Gripper (Manual only)
  server.on("/grip/open",  HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripOpen();  r->send(200,"text/plain","OK"); });
  server.on("/grip/close", HTTP_GET, [](AsyncWebServerRequest *r){ if(currentMode==MODE_MANUAL) gripClose(); r->send(200,"text/plain","OK"); });

  // Speed (Manual only)
  server.on("/speed/lin/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear+SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/lin/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_linear = clamp(speed_linear-SPEED_STEP, SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/up",   HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot+SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed/rot/down", HTTP_GET, [](AsyncWebServerRequest *r){ speed_rot    = clamp(speed_rot-SPEED_STEP,    SPEED_MIN, SPEED_MAX); if(currentMode==MODE_MANUAL) applyCurrentMotion(); r->send(200,"text/plain","OK"); });
  server.on("/speed",          HTTP_GET, [](AsyncWebServerRequest *r){
    String s = "Lin: " + String(speed_linear) + "  |  Rot: " + String(speed_rot);
    r->send(200,"text/plain", s);
  });

  server.begin();
  Serial.println("Server started. WebSocket at /ws");
}

void loop() {
  // WebSocket cleanup
  ws.cleanupClients();
  
  if (currentMode == MODE_LINE) {
    // Trao quyền cho do_line
    do_line_loop();
  }
  else if (currentMode == MODE_AI_ROUTE) {
    // AI Route mode: state machine
    route_loop();
    
    // Auto-return to IDLE/MANUAL when route is done
    if (route_is_done() && route_get_state() == RS_IDLE) {
      // Stay in AI_ROUTE mode, waiting for next route
    }
    
    // Send telemetry periodically
    if (millis() - telemetryTimer >= TELEMETRY_INTERVAL_MS) {
      telemetryTimer = millis();
      String tele = route_telemetry_json();
      ws.textAll(tele);
    }
  }
  else {
    // MODE_MANUAL
    if (line_mode) {
      stopCar();
      line_mode = false;
    }
    delay(5);
  }
}

// ================= Apply current motion (Manual) =================
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

// ================= Motor control (Manual) =================
void forward() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA, speed_linear);
  analogWrite(ENB, speed_linear);
}
void backward() {
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  analogWrite(ENA, speed_linear);
  analogWrite(ENB, speed_linear);
}
void left() {  // quay tại chỗ
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  analogWrite(ENA, speed_rot);
  analogWrite(ENB, speed_rot);
}
void right() { // quay tại chỗ
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  analogWrite(ENA, speed_rot);
  analogWrite(ENB, speed_rot);
}
void stopCar() {
  digitalWrite(IN1,LOW); digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW); digitalWrite(IN4,LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// ========= Diagonal steering (Manual) =========
void forwardLeft() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  if (!INVERT_STEER) { // giảm TRÁI
    analogWrite(ENA, speed_linear);
    analogWrite(ENB, diagScale(speed_linear));
  } else {             // giảm PHẢI
    analogWrite(ENA, diagScale(speed_linear));
    analogWrite(ENB, speed_linear);
  }
}
void forwardRight() {
  digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW);
  if (!INVERT_STEER) { // giảm PHẢI
    analogWrite(ENA, diagScale(speed_linear));
    analogWrite(ENB, speed_linear);
  } else {             // giảm TRÁI
    analogWrite(ENA, speed_linear);
    analogWrite(ENB, diagScale(speed_linear));
  }
}
void backwardLeft() {
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  analogWrite(ENA, diagScale(speed_linear)); // bánh PHẢI chậm hơn
  analogWrite(ENB, speed_linear);
}
void backwardRight() {
  digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH);
  analogWrite(ENA, speed_linear);
  analogWrite(ENB, diagScale(speed_linear)); // bánh TRÁI chậm hơn
}

// ================= Gripper =================
void gripOpen()  { gripper.write(OPEN_ANGLE); }
void gripClose() { gripper.write(CLOSE_ANGLE); }
