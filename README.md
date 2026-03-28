# 🤖 AI Robot Pathfinder — Hệ thống Xe Robot Giao hàng Tự hành

> **Đồ án Trí tuệ Nhân tạo** — Xe robot 4 bánh dò line tránh vật cản, điều khiển bằng thuật toán AI tìm kiếm qua giao diện Web Dashboard thời gian thực.

---

## 📋 Tổng Quan

Hệ thống **AI Robot Pathfinder** là dự án kết hợp giữa **thuật toán tìm kiếm AI** và **phần cứng robot thực tế**, gồm 2 thành phần chính:

1. **Web Dashboard (RoboControl)** — Giao diện điều khiển & mô phỏng thuật toán AI tìm đường trên bản đồ đồ thị node, hỗ trợ vẽ bản đồ, animation từng bước, so sánh hiệu năng thuật toán, gửi lộ trình & giám sát telemetry.
2. **Robot vật lý (ESP32)** — Xe 4 bánh tự hành dò line (5×TCRT5000), tránh vật cản (HC-SR04), thực thi lộ trình AI qua State Machine, giao tiếp Web bằng WebSocket + HTTP.

### Tính năng chính

| Tính năng | Mô tả |
|-----------|-------|
| 🧠 **5 thuật toán AI** | BFS, DFS, UCS, A*, Greedy Best-First với animation từng bước trên bản đồ đồ thị 20 node |
| 📊 **So sánh thuật toán** | Benchmark tất cả 5 thuật toán trên cùng bản đồ — so sánh node duyệt, số bước, thời gian tính toán |
| 🎮 **Điều khiển 8 hướng** | D-pad thủ công (Forward, Backward, Left, Right, 4 chéo) với hold-to-move |
| 🔄 **Dynamic Re-routing** | Robot phát hiện vật cản → ESP32 gửi tọa độ → Web tính đường mới → Gửi route cập nhật |
| 📡 **Telemetry real-time** | Tốc độ trái/phải, quãng đường, trạng thái 5 cảm biến, khoảng cách vật cản — cập nhật mỗi 200ms |
| 🚚 **Giao hàng đa điểm** | Hỗ trợ giao tối đa 6 đơn liên tiếp, tự tìm thứ tự tối ưu (greedy nearest-first) |
| 🌐 **Offline hoàn toàn** | Dashboard chạy local trên ESP32 (LittleFS) hoặc mở file HTML trực tiếp — không cần internet |

---

## 👥 Thành viên nhóm

| Họ tên | MSSV |
|--------|------|
| Đinh Quốc Anh | 24133003 |
| Nguyễn Chiến | 24133007 |
| Lý Gia Hân | 24133016 |
| Phan Tuấn Thanh | 24133054 |

---

## 🏗️ Kiến trúc Hệ thống

```
┌─────────────────────────────────────────────────────────────────┐
│                    KIẾN TRÚC HỆ THỐNG                          │
│                                                                 │
│  ┌──────────────────────┐        ┌──────────────────────────┐  │
│  │   WEB BROWSER (UI)   │        │     ESP32 (Firmware)     │  │
│  │                      │  WiFi  │                          │  │
│  │  • SVG Node Graph    │◄──────►│  • AsyncWebServer        │  │
│  │    (20 nodes, 4×5)   │  WS/   │  • WebSocket Server (/ws)│  │
│  │  • AI Engine (JS)    │  HTTP  │  • Route Interpreter     │  │
│  │    BFS/DFS/UCS/      │        │    State Machine         │  │
│  │    A*/Greedy         │        │  • PID Line Following    │  │
│  │  • Animation Engine  │        │  • Obstacle Detection    │  │
│  │  • Telemetry Display │        │  • Encoder Odometry      │  │
│  │  • Manual D-Pad      │        │  • LittleFS File Server  │  │
│  │  • Chart.js Stats    │        │  • Servo Gripper Control │  │
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

## 📁 Cấu trúc Dự án

```
AI_PROJECT/
├── web/                          # Giao diện Web Dashboard
│   ├── index.html                # Trang chính — 3 panel: Thủ công, Tự động, Thống kê
│   ├── style.css                 # Component styles: node animation, D-pad, toast, flow edges
│   ├── script.js                 # Core logic: 5 thuật toán AI, animation engine, WebSocket, D-pad
│   ├── tailwind.min.js           # TailwindCSS runtime (local, offline)
│   └── chart.min.js              # Chart.js (local, offline) cho biểu đồ so sánh
│
├── ai-firmware/                  # Firmware ESP32 (Arduino Framework)
│   └── main/
│       ├── main.ino              # Entry point: WiFi AP, HTTP routes, WebSocket handler, game loop
│       ├── do_line.cpp           # PID line following, encoder ISR, obstacle avoidance, spin/move
│       ├── do_line.h             # API: PID struct, UIMode enum, do_line_setup/loop/abort
│       ├── route_interpreter.cpp # AI Route state machine: IDLE→FOLLOW→INTERSECTION→OBSTACLE→DONE
│       └── route_interpreter.h   # API: RouteState enum, route_setup/load/loop/abort/telemetry
│
├── de_cuong_do_an_AI.md          # Đề cương đồ án chi tiết (784 dòng)
├── TEAM_TASKS.md                 # Phân công nhiệm vụ 4 thành viên — 2 tuần
└── README.md                     # (File này)
```

---

## 🧠 Thuật toán AI

Tất cả 5 thuật toán được triển khai bằng **JavaScript client-side** trong `script.js`, chạy trực tiếp trên trình duyệt.

| Thuật toán | Loại | Tối ưu? | Cấu trúc dữ liệu | Heuristic |
|-----------|------|---------|-------------------|-----------|
| **BFS** | Uninformed | ✅ (chi phí đều) | Queue (FIFO) | — |
| **DFS** | Uninformed | ❌ | Stack (LIFO) | — |
| **UCS** | Uninformed | ✅ | Priority Queue (sorted) | — |
| **A*** | Informed | ✅ | Priority Queue (f = g + h) | Manhattan |
| **Greedy** | Informed | ❌ | Priority Queue (f = h) | Manhattan |

**Heuristic function:** Manhattan Distance trên tọa độ SVG  
`h(n) = |x₁ - x₂| + |y₁ - y₂|`

### Đặc điểm triển khai

- **Bản đồ đồ thị**: 20 node (grid 4×5) với tọa độ SVG cố định, adjacency list có thể chỉnh sửa
- **Edge Editor**: Thêm/xóa cạnh nối, đặt trọng số tùy ý cho UCS/A*
- **Randomized neighbor order**: Shuffle thứ tự duyệt neighbor → animation trực quan hơn
- **Step history**: Ghi lại `{current, explored[], frontier[], from}` cho mỗi bước → phát lại animation
- **Multi-target**: Tìm nearest target trước (greedy traversal), hỗ trợ tối đa 6 điểm giao
- **Express mode**: Ưu tiên giao đơn cuối cùng trước

---

## ⚡ Luồng Hoạt Động

### 1. Điều khiển thủ công (Manual Mode)

```
Người dùng nhấn D-Pad → HTTP GET /forward → ESP32 set motor PWM → Robot di chuyển
Người dùng thả tay     → HTTP GET /stop    → ESP32 stop motors   → Robot dừng
```

Manual mode sử dụng **pointer events** (hold-to-move) với 8 hướng + nút dừng khẩn cấp.

### 2. AI Route Mode (Tự động)

```
Web: Chọn node đích trên SVG → Nhấn "MÔ PHỎNG" → Chạy thuật toán với animation
     Nhấn "NẠP LỘ TRÌNH" → Convert path → commands [F, L, R]
                                               │
     WebSocket: {"type":"ROUTE", "commands":["F","R","F","L"]}
                                               ▼
ESP32: route_load() → Parse JSON → cmdQueue → State Machine:
  RS_FOLLOWING_LINE:  PID dò line (5 mắt TCRT5000, 10ms loop)
  RS_AT_INTERSECTION: 5 mắt đều ON → dequeue lệnh → F/L/R (spin 90° bằng encoder)
  RS_OBSTACLE:        HC-SR04 < 15cm → dừng → gửi OBSTACLE_DETECTED
  RS_DONE:            Hết queue → gửi COMPLETED
```

### 3. Dynamic Re-routing

```
ESP32 phát hiện vật cản → Gửi WS: {"type":"OBSTACLE_DETECTED",
                                     "position":{"row":r,"col":c},
                                     "robot_position":{...}}
     │
Web nhận → obstacles.add(obsNode)
         → findPathAnimated(robotNode, target, algo)
         → pathToCommands(newPath)
         → wsSend({"type":"ROUTE", "commands":[...], "rerouted":true})
     │
ESP32 → route_load() thay queue → tiếp tục RS_FOLLOWING_LINE
```

### 4. Telemetry (mỗi 200ms)

```json
{
  "type": "TELEMETRY",
  "state": "FOLLOWING_LINE",
  "step": 3, "total": 8,
  "speedL": 0.38, "speedR": 0.40,
  "distance": 128.5,
  "obstacle": 45.2,
  "sensors": [0, 1, 1, 0, 0]
}
```

---

## 🔧 Công nghệ & Phần cứng

### Phần cứng

| Linh kiện | Chức năng | Thông số |
|-----------|-----------|----------|
| **ESP32-WROOM-32** | Vi điều khiển chính | Dual-core 240MHz, WiFi 802.11 b/g/n |
| **4× DC Geared Motor** | Bánh xe di chuyển | 3–6V, giảm tốc ~1:48 |
| **L298N** | Driver động cơ | Dual H-Bridge, 2 kênh (L: IN1/IN2/ENA, R: IN3/IN4/ENB) |
| **5× TCRT5000** | Cảm biến dò line | Hồng ngoại, PID 5 mắt, error [-4, +4] |
| **HC-SR04** | Siêu âm tránh vật cản | Đo 25Hz, EMA filter (α=0.4), ngưỡng 15cm |
| **2× Encoder quang** | Đo tốc độ & quãng đường | 20 PPR × 3 = 60 effective, ISR RISING |
| **Servo SG90** | Gripper giao hàng | 120° mở, 175° đóng, chân GPIO 18 |
| **2× LM2596** | DC-DC hạ áp | 5V (logic) + 7.4V (motor) |

### Sơ đồ chân ESP32

```
Motor Left:   IN1=12, IN2=14, ENA=13
Motor Right:  IN3=4,  IN4=2,  ENB=15
Line Sensors: L2=34, L1=32, M=33, R1=27, R2=25
Encoders:     ENC_L=26, ENC_R=22
HC-SR04:      TRIG=21, ECHO=19
Servo:        GPIO 18
```

### Phần mềm

| Công nghệ | Mục đích |
|-----------|----------|
| **C/C++ (Arduino)** | Firmware ESP32 — PID, State Machine, WebSocket Server |
| **HTML5 + TailwindCSS (local)** | Giao diện Dashboard — Material Design 3 color system |
| **JavaScript (ES6+ Vanilla)** | Thuật toán AI, Animation Engine, WebSocket client |
| **ESPAsyncWebServer** | HTTP server không đồng bộ + WebSocket handler |
| **ArduinoJson** | Parse/serialize JSON cho route & telemetry |
| **LittleFS** | Filesystem ESP32 — serve web files từ flash |
| **Chart.js (local)** | Biểu đồ so sánh thuật toán (bar chart) |
| **SVG** | Render bản đồ node graph + animation |

---

## 📊 Giao diện Dashboard

Dashboard có **3 panel** chuyển đổi qua sidebar:

### Panel 1: Thủ công 🖐️

- **D-Pad 8 hướng** — hold-to-move với pointer events, phản hồi HTTP tức thì
- **Telemetry cards** — Vận tốc (m/s), Quãng đường (cm), Trạng thái máy, Bước/Tổng
- **Sensor display** — 5 dot cảm biến line, khoảng cách sonar, trạng thái encoder

### Panel 2: Tự động 🤖

- **SVG Map** — 20 node, click để đặt Start/Target/Obstacle, kéo-thả edge & trọng số
- **5 Map Tools** — Xuất phát, Điểm giao, Vật cản, Edge, Trọng số
- **Animation Controls** — Play/Pause/Step/Reset, slider tốc độ, counter explored/frontier
- **Config Panel** — Chọn thuật toán, chế độ giao (đơn/nhiều đơn), chế độ Hỏa tốc
- **Route Table** — Bảng lộ trình, thứ tự giao đơn, nút "NẠP & CHO XE CHẠY"

### Panel 3: Thống kê 📊

- **KPI Cards** — Tổng quãng đường, Đơn thành công, Thời gian TB, Đã giao hôm nay
- **Algorithm Comparison** — Chart.js bar chart: Số bước / Node duyệt / ms cho 5 thuật toán
- **Ranking Table** — Xếp hạng 🥇🥈🥉 theo pathLen → visited → time
- **Order History** — Bảng lịch sử đơn hàng: ID, Thuật toán, Lộ trình, Bước, Trạng thái

---

## 📡 API Reference

### HTTP Endpoints (từ `main.ino`)

| Endpoint | Chức năng |
|----------|-----------|
| `GET /forward` | Tiến thẳng |
| `GET /backward` | Lùi |
| `GET /left` / `/right` | Quay trái/phải |
| `GET /fwd_left` / `/fwd_right` | Tiến chéo trái/phải |
| `GET /back_left` / `/back_right` | Lùi chéo trái/phải |
| `GET /stop` | Dừng |
| `GET /setMode?m=manual\|line_only\|ai_route` | Đổi chế độ |
| `GET /deliver?dir=1&path=0,1,6,7` | Giao hàng (Delivery mode) |
| `GET /estop` | Dừng khẩn cấp — abort tất cả |
| `GET /resume` | Tiếp tục chạy |
| `GET /return_home` | Về kho |
| `GET /grip/open` / `/grip/close` | Mở/đóng gripper |
| `GET /speed/lin/up` / `/speed/lin/down` | Tăng/giảm tốc độ thẳng (±10, range 60–255) |
| `GET /speed/rot/up` / `/speed/rot/down` | Tăng/giảm tốc độ xoay |
| `GET /api/stats` | Lấy thống kê JSON |
| `GET /` | Serve `index.html` từ LittleFS |

### WebSocket Messages (`/ws`)

| Hướng | Type | Payload |
|-------|------|---------|
| Web → ESP32 | `ROUTE` | `{commands:["F","L","R"], total_steps, algorithm, rerouted?}` |
| Web → ESP32 | `STOP` / `ESTOP` | `{}` — dừng & chuyển về Manual |
| Web → ESP32 | `RESUME` | `{}` — tiếp tục AI route |
| Web → ESP32 | `PING` | `{}` — heartbeat mỗi 5s |
| ESP32 → Web | `WELCOME` | `{mode}` — khi client kết nối |
| ESP32 → Web | `TELEMETRY` | `{state, step, total, speedL, speedR, distance, obstacle, sensors[5]}` |
| ESP32 → Web | `OBSTACLE_DETECTED` | `{position:{row,col}, robot_position:{row,col}, current_step, distance_cm}` |
| ESP32 → Web | `ROUTE_ACK` | `{commands: count}` |
| ESP32 → Web | `COMPLETED` | `{intersections, distance_cm}` |
| ESP32 → Web | `OBSTACLE_TIMEOUT` | — sau 10s không nhận route mới |
| ESP32 → Web | `PONG` | — phản hồi heartbeat |

---

## 📊 State Machine Firmware

```
              ┌──────────────────┐
              │     RS_IDLE      │ ← Chờ route từ Web
              └────────┬─────────┘
                       │ route_load() nhận JSON
                       ▼
              ┌──────────────────┐
         ┌───►│ RS_FOLLOWING_LINE│◄────────────┐
         │    │  PID dò line     │              │
         │    │  (10ms loop)     │              │
         │    └───┬──────────┬───┘              │
         │        │          │                  │
         │   5 mắt ON   HC-SR04 < 15cm         │
         │   (rising     (EMA filter)           │
         │    edge)          │                  │
         │        │          │                  │
         │        ▼          ▼                  │
         │  ┌──────────┐ ┌──────────────┐      │
         │  │AT_INTER- │ │ RS_OBSTACLE  │      │
         │  │SECTION   │ │ Dừng motor   │      │
         │  │          │ │ Gửi WS tọa độ│      │
         │  │Dequeue:  │ │ Chờ 10s      │      │
         │  │ F → thẳng│ │ timeout→IDLE │      │
         │  │ L → 90°⟲ │ └──────┬───────┘      │
         │  │ R → 90°⟳ │    route_load()       │
         │  └────┬─────┘        │               │
         │       │ (queue còn)  └───────────────┘
         └───────┘

              Queue rỗng → RS_DONE → gửi COMPLETED → RS_IDLE
```

### PID Line Following (`do_line.cpp`)

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| `Kp` | 300.0 | Hệ số tỉ lệ |
| `Ki` | 8.0 | Hệ số tích phân |
| `Kd` | 0.00 | Hệ số vi phân |
| `v_base` | 0.4 m/s | Vận tốc cơ sở |
| `v_boost` | 0.11 m/s | Bù lệch nhẹ |
| `v_hard` | 0.13 m/s | Bù lệch mạnh |
| `CTRL_DT_MS` | 10ms | Chu kỳ PID |
| `EMA_B` | 0.7 | Bộ lọc EMA vận tốc |
| `PWM_SLEW` | 8 | Slew rate giới hạn |
| `PWM_MIN_RUN` | 75 | PWM tối thiểu để motor quay |

**Bảng mã hóa error (5 mắt):**

| L2 | L1 | M | R1 | R2 | Trạng thái | Xử lý |
|----|----|----|----|----|-----------|-------|
| 0 | 0 | 1 | 0 | 0 | Đúng tâm | v_base đều |
| 0 | 1 | 1 | 0 | 0 | Lệch nhẹ trái | -v_boost, +v_boost |
| 1 | 0 | 0 | 0 | 0 | Lệch mạnh trái | -v_hard, +v_hard |
| 0 | 0 | 1 | 1 | 0 | Lệch nhẹ phải | +v_boost, -v_boost |
| 0 | 0 | 0 | 0 | 1 | Lệch mạnh phải | +v_hard, -v_hard |
| 1 | 1 | 1 | 1 | 1 | **Giao lộ** | Dequeue lệnh F/L/R |
| 0 | 0 | 0 | 0 | 0 | Mất line | Recovery: quay về phía cuối |

### Xoay góc tại giao lộ

- `spin_left_deg(90°)` / `spin_right_deg(90°)` — sử dụng encoder tính góc θ
- Công thức: `θ = (dR × CIRC / PPR - dL × CIRC / PPR) / TRACK_WIDTH`
- Dung sai: ±1.5° | PWM min: 150 | Timeout: 5s
- Sau khi quay: `move_forward_distance(0.02m)` để tái lấy line

### Tránh vật cản (HC-SR04)

- Quét tần suất: 25Hz (40ms/lần)
- EMA filter: α=0.4 → giảm nhiễu
- Hysteresis: ON ≤ 20cm (2 hit liên tiếp), OFF ≥ 25cm
- Lệnh tránh: Quay trái 40° → Tiến 20cm → Quay phải 40° → Tiến 15cm → Quay phải 50° → Tiến đến khi gặp line → Quay trái 15°

---

## 🚀 Hướng dẫn Sử dụng

### Yêu cầu phần mềm

- [Arduino IDE 2.x](https://www.arduino.cc/en/software) hoặc PlatformIO
- **Board**: ESP32 Dev Module
- **Thư viện cần cài**:
  - `ESPAsyncWebServer` + `AsyncTCP`
  - `ArduinoJson` (v6+)
  - `ESP32Servo`
  - `LittleFS` (tích hợp sẵn trong ESP32 core)

### Bước 1 — Flash Firmware

1. Mở `ai-firmware/main/main.ino` trong Arduino IDE
2. Chọn Board: **ESP32 Dev Module**
3. Upload code

### Bước 2 — Upload Web Files vào LittleFS

Upload các file trong thư mục `web/` vào LittleFS partition:
- `index.html`
- `style.css`
- `script.js`
- `tailwind.min.js`
- `chart.min.js`

> Sử dụng plugin [Arduino ESP32 LittleFS Uploader](https://github.com/lorol/arduino-esp32fs-plugin) hoặc PlatformIO `uploadfs`

### Bước 3 — Kết nối & Sử dụng

1. Bật nguồn robot → ESP32 tạo WiFi AP: **`ESP32-Car`** (mật khẩu: `12345678`)
2. Kết nối điện thoại/laptop vào WiFi `ESP32-Car`
3. Mở trình duyệt → **`http://192.168.4.1`**

> **Hoặc** mở file `web/index.html` trực tiếp trên máy tính để dùng phần mô phỏng thuật toán (offline, không cần robot).

### Hướng dẫn nhanh trên Dashboard

| Bước | Hành động |
|------|-----------|
| 1 | Chuyển sang tab **Tự động** (🤖) |
| 2 | Click tool **📌 Xuất phát** → click node trên bản đồ |
| 3 | Click tool **📍 Điểm giao** → click node(s) đích (tối đa 6 đơn) |
| 4 | Chọn thuật toán (A*, BFS, DFS, UCS, Greedy) |
| 5 | Nhấn **⏯ MÔ PHỎNG THUẬT TOÁN** → xem animation từng bước |
| 6 | Nhấn **🚀 NẠP LỘ TRÌNH & CHO XE CHẠY** → gửi route cho ESP32 |
| 7 | Theo dõi trạng thái robot trên panel Telemetry |

---

## 🔌 Chế độ Hoạt động (UIMode enum)

| Mode | Giá trị | Mô tả |
|------|---------|-------|
| `MODE_MANUAL` | 0 | D-pad HTTP → motor trực tiếp |
| `MODE_LINE_ONLY` | 1 | PID dò line liên tục, tự xử lý giao lộ |
| `MODE_DELIVERY` | 2 | Dò line theo path node, gripper open/close tại đích |
| `MODE_AI_ROUTE` | 3 | Route Interpreter state machine — nhận lệnh F/L/R từ Web |

---

## 📐 Thông số Cơ khí (từ code)

| Tham số | Giá trị | Đơn vị |
|---------|---------|--------|
| Bán kính bánh xe | 32.5 | mm |
| Chu vi bánh | 204.2 | mm |
| Khoảng cách 2 bánh (track width) | 115.0 | mm |
| Xung encoder / vòng (effective) | 60 | pulses |
| Tốc độ linear mặc định | 130 | PWM (0-255) |
| Tốc độ rotation mặc định | 110 | PWM |
| Tỉ lệ chéo (diagonal scale) | 70% | |
| Góc quay tại giao lộ | 85° | (85 thay vì 90 để bù trượt) |

---

## 📚 Tài liệu Tham khảo

1. **Russell, S. & Norvig, P.** (2021). *Artificial Intelligence: A Modern Approach* (4th ed.) — Chương 3, 4: Search Algorithms
2. **Hart, P.E., Nilsson, N.J. & Raphael, B.** (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Trans. SSC*
3. **Espressif Systems.** *ESP32 Technical Reference Manual*
4. **ESPAsyncWebServer** — [github.com/me-no-dev/ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
5. **Pathfinding Visualizer** — [clementmihailescu.github.io](https://clementmihailescu.github.io/Pathfinding-Visualizer/) (tham khảo UX)

---

*Đồ án Trí tuệ Nhân tạo — Học kỳ 2, 2026*