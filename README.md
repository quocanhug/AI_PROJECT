# 🤖 AI Robot Pathfinder — Hệ thống Xe Giao hàng Tự hành

> **Đồ án Trí tuệ Nhân tạo** — Xe robot 4 bánh dò line tránh vật cản, điều khiển bằng thuật toán AI qua giao diện Web.

---

## 📋 Tổng Quan

Hệ thống gồm **2 thành phần chính**:

1. **Giao diện Web (RoboControl Dashboard)** — Dashboard điều khiển & mô phỏng thuật toán AI tìm đường (BFS, DFS, UCS, A*, Greedy Best-First Search).
2. **Robot phần cứng** — Xe 4 bánh dò line (5×TCRT5000), tránh vật cản (HC-SR04), điều khiển bởi ESP32, giao tiếp Web qua Wi-Fi (WebSocket + HTTP).

### Tính năng nổi bật

- 🧠 **5 thuật toán AI** tìm đường với trực quan hóa trên bản đồ node lưới
- 📊 **So sánh hiệu năng** tất cả thuật toán trên cùng bản đồ
- 🎮 **Điều khiển thủ công** 8 hướng (D-pad) với phản hồi real-time
- 🔄 **Dynamic Re-routing** — Robot phát hiện vật cản → Web tính lại đường → Gửi route mới
- 📡 **Telemetry thời gian thực** — Tốc độ, quãng đường, cảm biến, vật cản
- 🌐 **WebSocket** giao tiếp hai chiều ESP32 ↔ Web Browser

---

## 🏗️ Kiến trúc Hệ thống

```
┌─────────────────────────────────────────────────────────────────┐
│                    KIẾN TRÚC HỆ THỐNG                          │
│                                                                 │
│  ┌──────────────────────┐        ┌──────────────────────────┐  │
│  │   WEB BROWSER (UI)   │        │     ESP32 (Firmware)     │  │
│  │                      │  WiFi  │                          │  │
│  │  • Map Editor (SVG)  │◄──────►│  • Web Server (Async)   │  │
│  │  • AI Algorithms     │  WS/   │  • WebSocket Server     │  │
│  │    (BFS,DFS,UCS,     │  HTTP  │  • Route Interpreter    │  │
│  │     A*,Greedy)       │        │  • PID Line Following   │  │
│  │  • Telemetry Display │        │  • State Machine        │  │
│  │  • Manual D-Pad      │        │  • Obstacle Handler     │  │
│  │  • Route Sender      │        │  • Encoder Odometry     │  │
│  └──────────────────────┘        └────────────┬─────────────┘  │
│                                               │ GPIO/PWM/ISR   │
│                               ┌───────────────┴──────────────┐ │
│                               │      PHẦN CỨNG ROBOT         │ │
│                               │  • 4× DC Motor + L298N      │ │
│                               │  • 5× TCRT5000 (dò line)    │ │
│                               │  • HC-SR04 (siêu âm)        │ │
│                               │  • 2× Encoder quang         │ │
│                               │  • Servo SG90 (gripper)     │ │
│                               │  • 2× LM2596 (nguồn)       │ │
│                               └──────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Cấu Trúc Thư Mục

```
AI_PROJECT/
├── web/                          # Giao diện web (chạy trên trình duyệt)
│   ├── index.html                # Trang chính — Dashboard UI (Tailwind CSS + Material Design 3)
│   ├── style.css                 # Custom styles bổ sung cho Tailwind
│   └── script.js                 # Logic: thuật toán AI, WebSocket, telemetry
│
├── ai-firmware/                  # Firmware ESP32 (Arduino framework)
│   └── main/
│       ├── main.ino              # Entry point: WiFi AP, HTTP routes, WebSocket, loop
│       ├── do_line.cpp           # PID line following, encoder ISR, obstacle avoidance
│       ├── do_line.h             # API dò line
│       ├── route_interpreter.cpp # State machine: IDLE→FOLLOW→INTERSECTION→OBSTACLE→DONE
│       └── route_interpreter.h   # API route interpreter
│
├── de_cuong_do_an_AI.md          # Đề cương đồ án chi tiết
├── TEAM_TASKS.md                 # Phân công nhiệm vụ nhóm
└── README.md                     # (File này)
```

---

## ⚡ Luồng Giao tiếp Software ↔ Hardware

### 1. Điều khiển thủ công (Manual Mode)

```
Người dùng nhấn D-Pad → HTTP GET /forward → ESP32 set motor direction → Robot di chuyển
Người dùng thả tay     → HTTP GET /stop    → ESP32 stop motors        → Robot dừng
```

### 2. Tự động (AI Route Mode)

```
Web: Vẽ bản đồ → Chọn thuật toán AI → Tìm đường → Convert path→commands
                                                         │
WebSocket: {"type":"ROUTE", "commands":["F","R","F","L"]} │
                                                         ▼
ESP32: Parse JSON → Queue commands → State Machine thực thi:
  ├── FOLLOWING_LINE: PID dò line bằng 5 mắt TCRT5000
  ├── AT_INTERSECTION: 5 mắt đều ON → dequeue lệnh → F/L/R
  ├── OBSTACLE: HC-SR04 < 15cm → dừng → gửi OBSTACLE_DETECTED
  └── DONE: hết queue → gửi COMPLETED
```

### 3. Dynamic Re-routing (Tái định tuyến)

```
ESP32 phát hiện vật cản → Gửi WS: OBSTACLE_DETECTED {position, robot_position}
        │
Web nhận → Đánh dấu vật cản trên bản đồ
        → Chạy lại thuật toán AI từ vị trí robot hiện tại
        → Gửi WS: ROUTE mới (rerouted: true)
        │
ESP32 nhận route mới → Thay queue → Tiếp tục chạy
```

### 4. Telemetry (Giám sát thời gian thực)

```
ESP32 (mỗi 200ms) → WebSocket: {"type":"TELEMETRY", "speedL":0.4, "speedR":0.38,
                                  "distance":128, "obstacle":45.2,
                                  "sensors":[0,1,1,0,0], "state":"FOLLOWING_LINE",
                                  "step":3, "total":8}
Web nhận → Cập nhật: tốc độ, quãng đường, sensor dots, trạng thái
```

---

## 🔧 Công Nghệ

### Phần cứng

| Linh kiện | Chức năng | Chi tiết |
|-----------|-----------|----------|
| **ESP32-WROOM-32** | Vi điều khiển chính | Dual-core 240MHz, WiFi 802.11 b/g/n |
| **4× DC Geared Motor** | Động cơ di chuyển | 3–6V, tỉ số ~1:48 |
| **L298N** | Driver động cơ | Dual H-Bridge, 2 kênh |
| **5× TCRT5000** | Cảm biến dò line | Hồng ngoại phản xạ, PID 5 mắt |
| **HC-SR04** | Cảm biến siêu âm | Phát hiện vật cản 2–400cm |
| **2× Encoder quang** | Đo quãng đường/tốc độ | 20 xung/vòng |
| **Servo SG90** | Gripper / quét sonar | 0°–180° |
| **2× LM2596** | Hạ áp DC-DC | 5V (logic) + 7.4V (motor) |

### Phần mềm

| Công nghệ | Mục đích |
|-----------|----------|
| **C/C++ (Arduino)** | Firmware ESP32 |
| **HTML5 / Tailwind CSS / JavaScript** | Giao diện Web (Material Design 3) |
| **WebSocket** | Giao tiếp real-time hai chiều |
| **ESPAsyncWebServer** | Web server không đồng bộ |
| **ArduinoJson** | Parse/serialize JSON |
| **Chart.js** | Biểu đồ thống kê |
| **Material Symbols** | Icon system (Google Fonts) |

---

## 🧠 Thuật Toán AI

| Thuật toán | Loại | Tối ưu? | Cấu trúc dữ liệu | Đặc điểm |
|-----------|------|---------|-------------------|----------|
| **BFS** | Uninformed | ✅ (chi phí đều) | Queue (FIFO) | Duyệt theo lớp, đảm bảo đường ngắn nhất |
| **DFS** | Uninformed | ❌ | Stack (LIFO) | Đi sâu trước, nhanh nhưng không tối ưu |
| **UCS** | Uninformed | ✅ | Priority Queue | Mở rộng node chi phí nhỏ nhất |
| **A*** | Informed | ✅ | Priority Queue | f(n) = g(n) + h(n), tối ưu nếu h admissible |
| **Greedy** | Informed | ❌ | Priority Queue | f(n) = h(n), nhanh nhưng không tối ưu |

**Heuristic**: Manhattan Distance `h(n) = |x₁-x₂| + |y₁-y₂|`

---

## 🚀 Hướng Dẫn Sử Dụng

### Cài đặt Firmware ESP32

1. Cài [Arduino IDE 2.x](https://www.arduino.cc/en/software) hoặc PlatformIO
2. Cài thư viện: `ESPAsyncWebServer`, `AsyncTCP`, `ArduinoJson`, `ESP32Servo`
3. Mở `ai-firmware/main/main.ino`
4. Chọn Board: **ESP32 Dev Module**, Upload

### Kết nối & Sử dụng

1. Bật nguồn robot → ESP32 tạo WiFi AP: **`ESP32-Car`** (pass: `12345678`)
2. Kết nối điện thoại/laptop vào WiFi `ESP32-Car`
3. Mở trình duyệt → **`http://192.168.4.1`**
4. Hoặc mở file `web/index.html` trực tiếp trên máy tính

### Giao diện Dashboard (3 chế độ)

| Chế độ | Chức năng |
|--------|-----------|
| **Thủ công** | D-pad 8 hướng + Telemetry (tốc độ, quãng đường, sensor, trạng thái) |
| **Tự động** | Bản đồ node SVG + 5 thuật toán AI + Cấu hình + Bảng lộ trình |
| **Thống kê** | KPI cards + Biểu đồ thời gian + So sánh thuật toán + Lịch sử đơn |

---

## 📡 API Reference

### HTTP Endpoints

| Endpoint | Chức năng |
|----------|-----------|
| `GET /forward` | Tiến thẳng |
| `GET /backward` | Lùi |
| `GET /left` / `/right` | Quay trái/phải |
| `GET /fwd_left` / `/fwd_right` | Tiến chéo |
| `GET /stop` | Dừng |
| `GET /setMode?m=manual\|line\|ai_route` | Đổi chế độ |
| `GET /api/stats` | Lấy thống kê |

### WebSocket Messages (`/ws`)

| Hướng | Type | Payload |
|-------|------|---------|
| Web → ESP32 | `ROUTE` | `{commands:["F","L","R"], total_steps, algorithm}` |
| Web → ESP32 | `ABORT` | `{}` |
| ESP32 → Web | `TELEMETRY` | `{state, step, total, speedL, speedR, distance, obstacle, sensors}` |
| ESP32 → Web | `OBSTACLE_DETECTED` | `{position:{row,col}, robot_position, distance_cm}` |
| ESP32 → Web | `ROUTE_ACK` | `{commands: count}` |
| ESP32 → Web | `COMPLETED` | `{intersections, distance_cm}` |

---

## 🔌 Sơ Đồ Chân ESP32

```
Motor Left:   IN1=12, IN2=14, ENA=13
Motor Right:  IN3=4,  IN4=2,  ENB=15
Line Sensors: L2=34, L1=32, M=33, R1=27, R2=25
Encoders:     ENC_L=26, ENC_R=22
HC-SR04:      TRIG=21, ECHO=19
```

---

## 📊 State Machine (Firmware)

```
              ┌──────────────────┐
              │     RS_IDLE      │ ← Chờ route từ Web
              └────────┬─────────┘
                       │ Nhận ROUTE
                       ▼
              ┌──────────────────┐
         ┌───►│ RS_FOLLOWING_LINE│◄───────┐
         │    │  PID dò line      │        │
         │    └───┬──────────┬───┘        │
         │        │          │             │
         │   5 mắt ON   HC-SR04<15cm      │
         │        │          │             │
         │        ▼          ▼             │
         │  ┌──────────┐ ┌──────────┐     │
         │  │INTERSECT │ │ OBSTACLE │     │
         │  │Dequeue:  │ │Dừng+WS  │     │
         │  │ F/L/R    │ │Chờ route │     │
         │  └────┬─────┘ └────┬─────┘     │
         │       │        NEW ROUTE        │
         └───────┘             └───────────┘

              Queue rỗng → RS_DONE → RS_IDLE
```

---

## 📚 Tài Liệu Tham Khảo

1. **Russell & Norvig** — *AI: A Modern Approach* (4th ed.) — Chương 3, 4: Search Algorithms
2. **Hart, Nilsson & Raphael** (1968) — Bài báo gốc thuật toán A*
3. **ESP32 Technical Reference** — Espressif Systems

---

*Developed for AI Course Project — 2026*