# 🤖 AI Delivery Robot — Xe Robot Giao hàng Tự hành

> **Đồ án môn học: Trí tuệ Nhân tạo**
> Xây dựng Hệ thống Xe Robot Giao hàng Tự hành Dò Line Tránh Vật cản
> Ứng dụng Thuật toán Tìm kiếm AI (BFS / DFS / UCS / A* / Greedy)
> với Giao diện Web Điều khiển và Mô phỏng Real-time

---

## 📋 Mục lục

- [Giới thiệu](#-giới-thiệu)
- [Tính năng](#-tính-năng)
- [Kiến trúc hệ thống](#-kiến-trúc-hệ-thống)
- [Phần cứng & GPIO](#-phần-cứng--gpio)
- [Cấu trúc mã nguồn](#-cấu-trúc-mã-nguồn)
- [Thuật toán AI](#-thuật-toán-ai)
- [Logic dò line & Điều hướng](#-logic-dò-line--điều-hướng)
- [Giao diện Web](#-giao-diện-web)
- [Hướng dẫn cài đặt](#-hướng-dẫn-cài-đặt)
- [Sơ đồ bản đồ Node](#-sơ-đồ-bản-đồ-node)
- [Giao thức WebSocket](#-giao-thức-websocket)
- [HTTP REST Endpoints](#-http-rest-endpoints)
- [Thành viên nhóm](#-thành-viên-nhóm)

---

## 🎯 Giới thiệu

Dự án xây dựng một hệ thống xe robot 4WD giao hàng tự hành kết hợp:

1. **Giao diện Web** mô phỏng bản đồ 2D với 5 thuật toán tìm kiếm AI, animation trực quan từng bước
2. **Xe robot thực tế** dò line trên lưới sàn **3×5 (15 node)**, rẽ đúng hướng tại giao lộ, tránh vật cản

Hệ thống giao tiếp **hai chiều real-time** qua WebSocket:
- Web tính đường → gửi array node cho Robot
- Robot chạy dò line → phản hồi telemetry (sensor, tốc độ, vị trí) mỗi 200ms
- Robot phát hiện vật cản → báo Web → Web tái định tuyến tự động

---

## ✨ Tính năng

### Giao diện Web
| Tính năng | Mô tả |
|---|---|
| 🧠 **5 Thuật toán AI** | BFS, DFS, UCS, A*, Greedy Best-First — chạy hoàn toàn client-side |
| 🎬 **Animation trực quan** | Từng bước: node explored (vàng), frontier (xanh dương), path (xanh lá), flow edges có animation |
| 📊 **So sánh thuật toán** | Chart.js bar chart + bảng ranking 🥇🥈🥉 so sánh bước, node duyệt, chi phí, thời gian (100 lần đo) |
| 🗺️ **Map Editor** | Đặt xuất phát, điểm giao (tối đa 6), vật cản, thêm/xóa cạnh, đặt trọng số edge |
| 📡 **Telemetry real-time** | Vận tốc (L/R EMA), trạng thái robot, bước hiện tại, 5 sensor dots, khoảng cách sonar |
| 🔄 **Dynamic Re-routing** | Robot gặp vật cản → web thêm node vào obstacles → tính đường mới → gửi ROUTE mới |
| 🏭 **Về kho** | Gửi ESTOP → tính đường từ `robotCurrentNode` → start node → gửi ROUTE mới |
| ⏭ **Tiếp tục** | Resume từ vị trí robot hiện tại (lấy phần còn lại của path cũ hoặc tính đường mới) |
| 🎮 **Manual D-Pad** | 8 hướng (hold-to-move), gripper servo, tốc độ linear/rotation — qua HTTP endpoints |
| 🚀 **Express Mode** | Giao đơn cuối cùng được thêm vào trước (ưu tiên đơn hỏa tốc) |

### Robot Firmware
| Tính năng | Mô tả |
|---|---|
| 📏 **PID Line Following** | 3 mắt giữa (L1, M, R1) điều khiển tốc độ 2 bánh; L2/R2 xử lý lệch nặng |
| ✚ **Intersection Detection** | `L2 && R2` = giao lộ; debounce 500ms; centering 3cm trước khi rẽ |
| 🔄 **Encoder-based Turning** | `spin_left/right_deg()` dùng encoder + góc × 1.5; tìm line tối đa 10cm sau rẽ |
| 🔃 **Initial Turn** | Xoay chuẩn hướng ngay khi nhận ROUTE mới (trước khi dò line) |
| 🚧 **Obstacle Sensing** | HC-SR04 median filter (3 lần), hysteresis 25cm ON / 30cm OFF, cần 2 lần liên tiếp |
| 🚗 **Recovery Logic** | Mất line → quay về phía `last_seen`; chưa từng thấy line → bò chậm tìm line |
| 🖐️ **Manual Control** | D-pad 8 hướng + gripper + speed adjust qua HTTP khi `MODE_MANUAL` |
| 📶 **WebSocket Telemetry** | Gửi sensor/speed/position mỗi 200ms khi `MODE_AI_ROUTE` đang chạy |

---

## 🏗 Kiến trúc hệ thống

```
┌──────────────────────────────────────────────────┐
│              WEB BROWSER (Client)                │
│                                                  │
│  ┌────────────┐  ┌──────────────────────────┐   │
│  │ Map Editor │  │ AI Search Engine          │   │
│  │ • Start    │  │ • BFS, DFS, UCS, A*,     │   │
│  │ • Targets  │  │   Greedy                  │   │
│  │ • Obstacles│  │ • Animation Engine        │   │
│  │ • Edges/W  │  │ • Algorithm Comparison    │   │
│  └────────────┘  └──────────────────────────┘   │
│  ┌────────────────────────────────────────────┐  │
│  │ Dashboard: Telemetry · D-pad · Gripper     │  │
│  └────────────────────────────────────────────┘  │
└─────────────────────┬────────────────────────────┘
                      │ WebSocket ws://.../ws
                      │ HTTP REST /deliver /estop …
                      │ WiFi AP (192.168.4.1)
                      ▼
┌──────────────────────────────────────────────────┐
│            ESP32-WROOM-32 (Firmware)             │
│                                                  │
│  ┌────────────────┐  ┌───────────────────────┐  │
│  │ ai-firmware.ino│  │ do_line.cpp            │  │
│  │ • WiFi AP      │  │ • PID line following   │  │
│  │ • WebSocket    │  │ • Intersection detect  │  │
│  │ • HTTP routes  │  │ • Encoder turning      │  │
│  │ • Telemetry    │  │ • Obstacle handling    │  │
│  │ • Mode control │  │ • Path navigation      │  │
│  └────────────────┘  └───────────────────────┘  │
└─────────────────────┬────────────────────────────┘
                      │ GPIO / PWM / ISR / pulseIn
                      ▼
┌──────────────────────────────────────────────────┐
│  PHẦN CỨNG                                       │
│  • 4× DC Motor (L298N)      • 5× TCRT5000       │
│  • Servo SG90 (gripper)     • HC-SR04 (sonar)    │
│  • 2× Encoder (ISR RISING)  • 2× LM2596 (5V/7V) │
└──────────────────────────────────────────────────┘
```

---

## 🔧 Phần cứng & GPIO

| Thành phần | Linh kiện | GPIO |
|---|---|---|
| Vi điều khiển | ESP32-WROOM-32 | — |
| Motor trái (×2) | DC Geared Motor | IN1=12, IN2=14, ENA=13 |
| Motor phải (×2) | DC Geared Motor | IN3=4, IN4=2, ENB=15 |
| Driver | L298N Dual H-Bridge | — |
| Cảm biến dò line | 5× TCRT5000 | L2=34, L1=32, M=33, R1=27, R2=25 |
| Cảm biến siêu âm | HC-SR04 | Trig=21, Echo=19 |
| Encoder | 2× Encoder quang (ISR RISING) | Left=26, Right=22 |
| Servo gripper | SG90 | Pin=18 (Open=120°, Close=175°) |
| Nguồn | 2× Pack 18650 (3S) + LM2596 | — |

> **Lưu ý**: Motor pins được định nghĩa ở CẢ HAI `ai-firmware.ino` và `do_line.cpp`. Khi sửa pin phải cập nhật cả hai file.

---

## 📁 Cấu trúc mã nguồn

```
ai-final_project/
├── README.md                    # Tài liệu dự án (file này)
├── AGENTS.md                    # Hướng dẫn cho AI coding assistant
├── de_cuong_do_an_AI.md         # Đề cương đồ án
│
└── ai-firmware/                 # Firmware ESP32 + Web files
    ├── ai-firmware.ino          # Main: WiFi AP, WebSocket, HTTP endpoints, telemetry, mode switching
    ├── do_line.h                # Header: PID struct, UIMode enum, function declarations
    ├── do_line.cpp              # Core: PID dò line, giao lộ, rẽ encoder, obstacle, initial turn
    └── data/                   # Web files (upload lên LittleFS)
        ├── index.html           # Dashboard UI (3 tabs: Thủ công / Tự động / Thống kê)
        ├── script.js            # AI algorithms, animation, edge editor, WebSocket handler
        ├── style.css            # Custom CSS (node colors/states, robot indicator, animations)
        ├── tailwind.min.js      # TailwindCSS runtime (451KB, local — không có internet)
        └── chart.min.js         # Chart.js (~205KB, để vẽ biểu đồ so sánh)
```

---

## 🧠 Thuật toán AI

5 thuật toán tìm kiếm chạy **hoàn toàn client-side** (JavaScript, không cần server):

| Thuật toán | Loại | Cấu trúc dữ liệu | Tối ưu? | Ghi chú |
|---|---|---|---|---|
| **BFS** | Uninformed | Queue (FIFO) | ✅ (đồng nhất) | Duyệt theo lớp |
| **DFS** | Uninformed | Stack (LIFO) | ❌ | Không ổn định |
| **UCS** | Uninformed | Priority Queue (g) | ✅ (mọi trọng số) | Như BFS khi w=1 |
| **A*** | Informed | Priority Queue (g+h) | ✅ (h admissible) | Khuyến nghị cho robot |
| **Greedy** | Informed | Priority Queue (h) | ❌ | Nhanh nhưng không đảm bảo |

**Heuristic**: Manhattan Distance `h(n) = |x₁-x₂| + |y₁-y₂|`

**Multi-target**: Greedy nearest-by-traversal — chạy thuật toán từ vị trí hiện tại, target nào tìm thấy trước → đi đến đó trước. Express Mode: ưu tiên target cuối cùng được thêm vào.

**Direction mapping** (JS ↔ C++):
```
JS:  0=up,   1=right, 2=down,  3=left
C++: 0=down, 1=right, 2=up,    3=left

JS_TO_CPP_DIR = [2, 1, 0, 3]
CPP_TO_JS_DIR = [2, 1, 0, 3]
```

---

## 🔩 Logic dò line & Điều hướng

### PID Line Following (do_line.cpp)

```
Cấu hình sensor: L2(34) | L1(32) | M(33) | R1(27) | R2(25)
                  ←────── ngoài ──────────────── ngoài ──→

• 3 mắt giữa (L1, M, R1) → điều khiển v_base / v_boost / v_hard
• L2, R2 → phát hiện giao lộ (L2 && R2) + xử lý lệch nặng
• LOW = trên vạch (TCRT5000 active-low)
```

**Bảng quyết định:**
| Trạng thái sensor | Hành động |
|---|---|
| M=1 (chỉ M) | Tiến thẳng `v_base` |
| L1=1, M=1 | Soft left: L−boost, R+boost |
| L1=1, M=0 | Hard left: L−hard, R+hard |
| R1=1, M=1 | Soft right: L+boost, R−boost |
| R1=1, M=0 | Hard right: L+hard, R−hard |
| L2=1 only | Quay phải tìm line |
| R2=1 only | Quay trái tìm line |
| Mất hết (chưa có line) | Bò chậm `v_search` |
| Mất hết (đã từng có) | Recovery: quay về `last_seen` |
| L2=1 AND R2=1 | **GIAO LỘ** → xử lý rẽ |

### Giao lộ & Rẽ

```
1. Trigger: L2 && R2 (debounce 500ms)
2. Centering: tiến 3cm vào tâm giao lộ
3. Dừng hẳn (300ms)
4. Tính diff = (targetDir - currentDir + 4) % 4
   • diff == 0 → THẲNG (chỉ centering nhẹ, tiếp tục PID)
   • diff == 1 → TRÁI: spin_left_deg(62°)
   • diff == 3 → PHẢI: spin_right_deg(62°)
   • diff == 2 → U-TURN: spin_right_deg(125°)
5. Sau rẽ: tiến tối đa 10cm tìm line
6. Cập nhật currentDir, currentPathIndex++
```

> ⚠️ `spin_*_deg(deg)` quay `deg × 1.5` thực tế do hệ số nội bộ.

### Initial Turn

Khi `do_line_setup()` được gọi (nhận ROUTE mới), `needs_initial_turn = true`. Vòng loop đầu tiên sẽ:
1. Xoay xe về hướng của đoạn đầu tiên trong path
2. Tiến tìm line (tối đa 10cm)
3. Đặt `currentPathIndex = 1`, chạy PID bình thường

### Obstacle Detection

```
• HC-SR04 polling mỗi 25ms (không blocking trong loop)
• readDistanceCM_filtered(): median 3 lần (~30ms blocking khi đọc)
• ON: < 25cm, cần 2 lần liên tiếp (obs_hit >= 2)
• OFF: > 30cm (hysteresis)
• MODE_AI_ROUTE: dừng + gửi OBSTACLE_DETECTED → Web reroute
• Mode khác: avoidObstacle() vật lý (spin + forward)
```

---

## 🖥 Giao diện Web

### 3 Tab chính

| Tab | Chức năng |
|---|---|
| 🖐️ **Thủ công** | D-pad 8 hướng (hold-to-move), nút "Về điểm xuất phát", sensor dots (5 mắt), sonar display, vận tốc real-time |
| 🤖 **Tự động** | Map editor, chọn thuật toán + chế độ giao, animation, nút NẠP LỘ TRÌNH, VỀ KHO, TIẾP TỤC, E-STOP |
| 📊 **Thống kê** | KPIs (đơn giao, thời gian TB, hiệu suất), biểu đồ so sánh 5 thuật toán, bảng phân tích chi tiết |

### Công cụ bản đồ (Tab Tự động)

| Tool | ID | Mô tả |
|---|---|---|
| 📌 Xuất phát | `toolStart` | Click chọn node làm điểm bắt đầu / kho |
| 📍 Điểm giao | `toolTarget` | Click toggle điểm giao hàng (tối đa 6 ở multi-mode) |
| 🚫 Vật cản | `toolObstacle` | Click toggle vật cản (loại khỏi đồ thị khi tìm đường) |
| 🔗 Edge | `toolEdge` | Click 2 node để thêm/xóa cạnh |
| ⚖️ Trọng số | `toolWeight` | Click 2 node kề để đặt trọng số edge (hiện qua `prompt`) |

### Các nút điều khiển chính

| Nút | Action | Mô tả |
|---|---|---|
| ⏯ MÔ PHỎNG | `animStart()` | Tính path + auto-play animation từng bước |
| 🚀 NẠP LỘ TRÌNH | `deliverBtn` click | Gửi `{type:"ROUTE", path:[...], initialDir:N}` qua WebSocket |
| 🏭 VỀ KHO | `returnHome()` | ESTOP → tính path về start node → gửi ROUTE |
| ⏭ TIẾP TỤC | `resumeRoute()` | Gửi phần còn lại của path (hoặc tính đường mới) |
| ⚠️ E-STOP | `sendCommand('/estop')` | HTTP `/estop` + WebSocket `ESTOP` → dừng ngay |
| 📊 So sánh | `compareAllBtn` | Chạy 100 lần benchmark 5 thuật toán, vẽ chart |

---

## 🚀 Hướng dẫn cài đặt

### Yêu cầu
- **Arduino IDE 2.x** (hoặc PlatformIO)
- **Board**: ESP32 Dev Module (ESP32-WROOM-32)
- **Libraries** (cài qua Library Manager):
  - `ESPAsyncWebServer` + `AsyncTCP`
  - `ESP32Servo`
  - `ArduinoJson` (v6)
  - `LittleFS` (built-in với ESP32 core)
- **Plugin**: [ESP32 LittleFS Data Upload](https://github.com/lorol/arduino-esp32littlefs-plugin) cho Arduino IDE 1.x / [arduino-littlefs-upload](https://github.com/earlephilhower/arduino-littlefs-upload) cho IDE 2.x

### Các bước

```bash
# 1. Clone project
git clone <repo-url>

# 2. Mở ai-firmware/ai-firmware.ino trong Arduino IDE

# 3. Cài Board Manager
#    Tools → Board → ESP32 Dev Module

# 4. Cài thư viện (Library Manager)
#    ESPAsyncWebServer, AsyncTCP, ESP32Servo, ArduinoJson

# 5. Chọn Partition Scheme
#    Tools → Partition Scheme → Default 4MB with spiffs (1.2MB APP / 1.5MB SPIFFS)
#    (cần ~762KB cho web files)

# 6. Upload web files lên LittleFS
#    Tools → ESP32 LittleFS Data Upload  (upload toàn bộ thư mục data/)

# 7. Upload firmware
#    Sketch → Upload

# 8. Kết nối WiFi
#    SSID: ESP32-Car  |  Password: 12345678

# 9. Mở trình duyệt
#    http://192.168.4.1
```

### Kích thước web files (~762KB tổng)
| File | Kích thước |
|---|---|
| tailwind.min.js | ~451KB |
| chart.min.js | ~205KB |
| script.js | ~75KB |
| index.html | ~38KB |
| style.css | ~10KB |

---

## 📐 Sơ đồ bản đồ Node

```
Node IDs (3 hàng × 5 cột):      Track vật lý:

 0 ── 1 ── 2 ── 3 ── 4           ●─────●─────●─────●─────●
 │    │    │    │    │            │     │     │     │     │
 5 ── 6 ── 7 ── 8 ── 9           ●─────●─────●─────●─────●
 │    │    │    │    │            │     │     │     │     │
10 ──11 ──12 ──13 ──14           ●─────●─────●─────●─────●

Tổng: 15 node  |  Mỗi ● = giao lộ  |  Khoảng cách ~25cm/node

Tọa độ SVG (x, y) — dùng trong cả firmware (node_coords) và JS (coords):
  Hàng 0: (30,30)  (100,30)  (170,30)  (240,30)  (310,30)
  Hàng 1: (30,100) (100,100) (170,100) (240,100) (310,100)
  Hàng 2: (30,170) (100,170) (170,170) (240,170) (310,170)
```

---

## 📡 Giao thức WebSocket

### Web → ESP32

| Message | Payload | Mô tả |
|---|---|---|
| `ROUTE` | `{path: [10,5,0,1], initialDir: 2}` | Gửi lộ trình — array node IDs + hướng ban đầu (C++ convention) |
| `STOP` | `{}` | Dừng thường — hủy lộ trình, về MODE_MANUAL |
| `ESTOP` | `{}` | Dừng khẩn cấp — gọi `do_line_abort()` ngay lập tức |
| `RESUME` | `{}` | Resume nếu MODE_AI_ROUTE đang active |
| `PING` | `{}` | Heartbeat mỗi 5 giây |

### ESP32 → Web

| Message | Payload | Mô tả |
|---|---|---|
| `WELCOME` | `{mode: N}` | Gửi khi client mới kết nối |
| `ROUTE_ACK` | `{commands: N}` | Xác nhận đã nạp route (N = số node) |
| `TELEMETRY` | `{state, step, total, robotNode, robotDir, speedL, speedR, obstacle, sensors[5]}` | Mỗi 200ms khi MODE_AI_ROUTE |
| `OBSTACLE_DETECTED` | `{robotNode, obstacleNode, robotDir, current_step, distance_cm}` | Khi phát hiện vật cản hoặc rẽ thất bại |
| `COMPLETED` | `{robotNode, robotDir}` | Đã đến đích thành công |
| `PONG` | `{}` | Phản hồi heartbeat |

---

## 🌐 HTTP REST Endpoints

| Endpoint | Method | Mô tả |
|---|---|---|
| `/` | GET | Serve `index.html` từ LittleFS |
| `/setMode?m=manual\|line_only\|ai_route` | GET | Chuyển chế độ hoạt động |
| `/deliver?dir=N&path=0,1,6` | GET | Nạp lộ trình qua HTTP (fallback khi WS không sẵn) |
| `/estop` | GET | Dừng khẩn cấp |
| `/resume` | GET | Resume từ vị trí hiện tại |
| `/return_home` | GET | Dừng + trả về vị trí hiện tại (web tự tính đường về) |
| `/api/stats` | GET | JSON: delivered, avgTime, efficiency |
| `/forward` `/backward` `/left` `/right` `/stop` | GET | Manual movement |
| `/fwd_left` `/fwd_right` `/back_left` `/back_right` | GET | Diagonal movement |
| `/grip/open` `/grip/close` | GET | Điều khiển gripper servo |
| `/speed/lin/up\|down` `/speed/rot/up\|down` | GET | Tăng/giảm tốc độ manual |

---

## 🔄 Dynamic Re-routing Flow

```
Robot đang chạy MODE_AI_ROUTE      Web Browser
         │                               │
         │ HC-SR04 < 25cm (×2)          │
         │ → DỪNG + do_line_abort()      │
         │                               │
         ├──── OBSTACLE_DETECTED ──────→ │
         │  {robotNode, obstacleNode,     │
         │   robotDir, distance_cm}       │
         │                               ├── obstacles.add(obstacleNode)
         │                               ├── updateNodeVisuals()
         │                               ├── findPathAnimated(robotNode → target)
         │                               ├── animRunSingle() — animation
         │                               │
         │ ←───────── ROUTE ─────────── │
         │  {path:[...], initialDir:N,   │
         │   rerouted:true}              │
         │                               │
         ├── do_line_setup() (initial turn)
         ├── Tiếp tục dò line           │
         │                               │
         ├──────── COMPLETED ──────────→ │
         │  {robotNode, robotDir}        │
         │                               └── Cập nhật mũi tên tại node đích
```

---

## 🛡 Chế độ hoạt động (UIMode)

| Mode | Giá trị | Mô tả |
|---|---|---|
| `MODE_MANUAL` | 0 | Điều khiển tay qua D-pad HTTP |
| `MODE_LINE_ONLY` | 1 | Dò line + đi thẳng qua giao lộ (test) |
| `MODE_DELIVERY` | 2 | Chạy path + điều khiển gripper servo |
| `MODE_AI_ROUTE` | 3 | Chạy path + telemetry WebSocket + báo vật cản |

---

## 👥 Thành viên nhóm

| Họ tên | MSSV |
|---|---|
| Đinh Quốc Anh | 24133003 |
| Nguyễn Chiến | 24133007 |
| Lý Gia Hân | 24133016 |
| Phan Tuấn Thanh | 24133054 |

---

## 📚 Tài liệu tham khảo

1. Russell, S. & Norvig, P. (2021). *Artificial Intelligence: A Modern Approach* (4th ed.) — Chương 3, 4: Search Algorithms
2. Hart, P.E. et al. (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" — Bài báo gốc A*
3. Espressif Systems. *ESP32 Technical Reference Manual*
4. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) — Async WebSocket cho ESP32
5. [ArduinoJson](https://arduinojson.org/) — JSON serialization cho embedded
6. [Chart.js](https://www.chartjs.org/) — Biểu đồ so sánh thuật toán

---

*Đồ án Trí tuệ Nhân tạo — 2026*