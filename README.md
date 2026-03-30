# 🤖 AI Delivery Robot — Xe Robot Giao hàng Tự hành

> **Đồ án môn học: Trí tuệ Nhân tạo**
> Xây dựng Hệ thống Xe Robot Giao hàng Tự hành Dò Line Tránh Vật cản
> Ứng dụng Thuật toán Tìm kiếm AI (BFS / DFS / UCS / A* / Greedy)
> với Giao diện Web Điều khiển và Mô phỏng

---

## 📋 Mục lục

- [Giới thiệu](#-giới-thiệu)
- [Tính năng](#-tính-năng)
- [Kiến trúc hệ thống](#-kiến-trúc-hệ-thống)
- [Phần cứng](#-phần-cứng)
- [Cấu trúc mã nguồn](#-cấu-trúc-mã-nguồn)
- [Thuật toán AI](#-thuật-toán-ai)
- [Giao diện Web](#-giao-diện-web)
- [Hướng dẫn cài đặt](#-hướng-dẫn-cài-đặt)
- [Sơ đồ bản đồ Node](#-sơ-đồ-bản-đồ-node)
- [Giao thức WebSocket](#-giao-thức-websocket)
- [Thành viên nhóm](#-thành-viên-nhóm)

---

## 🎯 Giới thiệu

Dự án xây dựng một hệ thống xe robot 4WD giao hàng tự hành kết hợp:

1. **Giao diện Web** mô phỏng bản đồ 2D với 5 thuật toán tìm kiếm AI, animation trực quan từng bước
2. **Xe robot thực tế** dò line trên lưới sàn 4×5 (20 node), rẽ đúng hướng tại giao lộ, tránh vật cản

Hệ thống giao tiếp **hai chiều real-time** qua WebSocket: Web tính đường → gửi cho Robot → Robot phản hồi telemetry/vật cản → Web tái định tuyến.

---

## ✨ Tính năng

### Giao diện Web
| Tính năng | Mô tả |
|---|---|
| 🧠 **5 Thuật toán AI** | BFS, DFS, UCS, A*, Greedy Best-First Search — chạy client-side |
| 🎬 **Animation trực quan** | Hiển thị từng bước: node explored (vàng), frontier (xanh), path (xanh lá), flow edges |
| 📊 **So sánh thuật toán** | Biểu đồ Chart.js + bảng ranking 🥇🥈🥉 so sánh số bước, node duyệt, thời gian |
| 🗺️ **Map Editor** | Đặt xuất phát, điểm giao (tối đa 6), vật cản, thêm/xóa cạnh, trọng số edge |
| 📡 **Telemetry real-time** | Vận tốc, quãng đường, trạng thái cảm biến (5 mắt), khoảng cách vật cản |
| 🔄 **Dynamic Re-routing** | Robot gặp vật cản → web tự động tính đường mới → gửi lại cho robot |
| 🏭 **Về kho / Tiếp tục** | Tính đường từ vị trí hiện tại về start node, hoặc resume route đã dừng |

### Robot Firmware
| Tính năng | Mô tả |
|---|---|
| 📏 **PID Line Following** | 3 mắt giữa (L1, M, R1) + velocity PID (encoder feedback) |
| ✚ **Intersection Detection** | 2 mắt ngoài (L2, R2) phát hiện giao lộ, debounce 500ms |
| 🔄 **Encoder-based Turning** | Xoay tại chỗ bằng encoder, centering 3cm trước rẽ, tìm line sau rẽ |
| 🚧 **Obstacle Sensing** | HC-SR04 median filter, hysteresis 25cm/30cm, latched detection |
| 🖐️ **Manual Control** | D-pad 8 hướng, gripper servo, speed adjust — qua HTTP endpoints |
| 📶 **WebSocket Telemetry** | Gửi sensor/speed/position mỗi 200ms khi đang chạy AI route |

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
│  │ • Edges    │  │ • Algorithm Comparison    │   │
│  └────────────┘  └──────────────────────────┘   │
│  ┌──────────────────────────────────────────┐   │
│  │ Dashboard: Telemetry, D-pad, Gripper     │   │
│  └──────────────────────────────────────────┘   │
└─────────────────────┬────────────────────────────┘
                      │ WebSocket / HTTP
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
                      │ GPIO / PWM / Interrupt
                      ▼
┌──────────────────────────────────────────────────┐
│  PHẦN CỨNG                                       │
│  • 4× DC Motor (L298N)      • 5× TCRT5000       │
│  • Servo SG90 (gripper)     • HC-SR04 (sonar)    │
│  • 2× Encoder (ISR)         • 2× LM2596 (5V/7V) │
└──────────────────────────────────────────────────┘
```

---

## 🔧 Phần cứng

| Thành phần | Linh kiện | GPIO |
|---|---|---|
| Vi điều khiển | ESP32-WROOM-32 | — |
| Motor trái | DC Geared Motor × 2 | IN1=12, IN2=14, ENA=13 |
| Motor phải | DC Geared Motor × 2 | IN3=4, IN4=2, ENB=15 |
| Driver | L298N Dual H-Bridge | — |
| Cảm biến dò line | 5× TCRT5000 | L2=34, L1=32, M=33, R1=27, R2=25 |
| Cảm biến siêu âm | HC-SR04 | Trig=21, Echo=19 |
| Encoder | 2× Encoder quang | Left=26, Right=22 |
| Servo | SG90 (gripper) | Pin=18 |
| Nguồn | 2× Pack 18650 (3S) | — |
| Hạ áp | 2× LM2596 (5V + 7.4V) | — |

---

## 📁 Cấu trúc mã nguồn

```
ai-final_project/
├── README.md                    # Tài liệu dự án (file này)
├── agents.md                    # Hướng dẫn cho AI coding assistant
├── de_cuong_do_an_AI.md         # Đề cương đồ án
│
└── ai-firmware/                 # Firmware ESP32 + Web files
    ├── ai-firmware.ino          # Main: WiFi, WebSocket, HTTP, telemetry
    ├── do_line.h                # Header: PID struct, UIMode enum
    ├── do_line.cpp              # Core: PID dò line, giao lộ, rẽ, obstacle
    └── data/                    # Web files (LittleFS)
        ├── index.html           # Dashboard UI (3 tabs)
        ├── script.js            # AI algorithms + animation + WebSocket
        ├── style.css            # Custom CSS (node states, animations)
        ├── tailwind.min.js      # TailwindCSS runtime
        └── chart.min.js         # Chart.js library
```

---

## 🧠 Thuật toán AI

5 thuật toán tìm kiếm chạy **hoàn toàn client-side** trên trình duyệt:

| Thuật toán | Loại | Cấu trúc dữ liệu | Tối ưu? |
|---|---|---|---|
| **BFS** | Uninformed | Queue (FIFO) | ✅ (đều bước) |
| **DFS** | Uninformed | Stack (LIFO) | ❌ |
| **UCS** | Uninformed | Priority Queue | ✅ (có trọng số) |
| **A*** | Informed | Priority Queue | ✅ (h admissible) |
| **Greedy** | Informed | Priority Queue | ❌ |

**Heuristic**: Manhattan Distance `h(n) = |x₁-x₂| + |y₁-y₂|`

**Multi-target delivery**: Greedy nearest-by-traversal — chạy thuật toán từ vị trí hiện tại, target nào tìm thấy trước → đi đến đó trước.

---

## 🖥 Giao diện Web

### 3 Tab chính

| Tab | Chức năng |
|---|---|
| 🖐️ **Thủ công** | D-pad 8 hướng, gripper, speed control, sensor display, telemetry |
| 🤖 **Tự động** | Map editor, algorithm selection, animation, route loading, E-STOP |
| 📊 **Thống kê** | KPIs (đơn giao, thời gian TB, hiệu suất), algorithm comparison chart |

### Công cụ bản đồ

| Tool | Mô tả |
|---|---|
| 📌 Xuất phát | Click chọn node xuất phát (kho) |
| 📍 Điểm giao | Click toggle điểm giao hàng (tối đa 6) |
| 🚫 Vật cản | Click toggle vật cản |
| 🔗 Edge | Click 2 node để thêm/xóa cạnh |
| ⚖️ Trọng số | Click 2 node kề để đặt trọng số edge |

---

## 🚀 Hướng dẫn cài đặt

### Yêu cầu
- Arduino IDE 2.x (hoặc PlatformIO)
- Board: ESP32-WROOM-32
- Libraries: `ESPAsyncWebServer`, `AsyncTCP`, `ESP32Servo`, `ArduinoJson`, `LittleFS`
- Plugin: **ESP32 Sketch Data Upload** (LittleFS)

### Các bước

```bash
# 1. Clone project
git clone <repo-url>

# 2. Mở ai-firmware/ai-firmware.ino trong Arduino IDE

# 3. Cài đặt Board Manager
#    Tools → Board → ESP32 Dev Module

# 4. Cài thư viện (Library Manager)
#    ESPAsyncWebServer, AsyncTCP, ESP32Servo, ArduinoJson

# 5. Upload web files lên LittleFS
#    Tools → ESP32 Sketch Data Upload (upload thư mục data/)

# 6. Upload firmware
#    Sketch → Upload

# 7. Kết nối WiFi
#    SSID: ESP32-Car  |  Password: 12345678

# 8. Mở trình duyệt
#    http://192.168.4.1
```

### Partition Scheme
- Chọn: **Default 4MB with spiffs (1.2MB APP / 1.5MB SPIFFS)**
- Tổng web files: ~762KB (tailwind 451KB + chart 205KB + script 62KB + html 34KB + css 10KB)

---

## 📐 Sơ đồ bản đồ Node

```
Node IDs:           Track thực tế:

 0 ── 1 ── 2 ── 3 ── 4       ●─────●─────●─────●─────●
 │    │    │    │    │        │     │     │     │     │
 5 ── 6 ── 7 ── 8 ── 9       ●─────●─────●─────●─────●
 │    │    │    │    │        │     │     │     │     │
10 ──11 ──12 ──13 ──14       ●─────●─────●─────●─────●
 │    │    │    │    │        │     │     │     │     │
15 ──16 ──17 ──18 ──19       ●─────●─────●─────●─────●

Grid: 4 hàng × 5 cột
Khoảng cách giữa 2 node kề: ~25cm
Mỗi ● = giao lộ (intersection)
Mỗi ─ = line trên sàn
```

---

## 📡 Giao thức WebSocket

### Web → ESP32

| Message | Payload | Mô tả |
|---|---|---|
| `ROUTE` | `{path: [15,10,5], initialDir: 2}` | Gửi lộ trình (mảng node ID) |
| `STOP` | `{}` | Dừng bình thường |
| `ESTOP` | `{}` | Dừng khẩn cấp |
| `RESUME` | `{}` | Tiếp tục route đã dừng |
| `PING` | `{}` | Heartbeat (mỗi 5s) |

### ESP32 → Web

| Message | Payload | Mô tả |
|---|---|---|
| `ROUTE_ACK` | `{commands: N}` | Xác nhận đã nạp route |
| `TELEMETRY` | `{step, total, robotNode, robotDir, speedL, speedR, obstacle, sensors}` | Dữ liệu cảm biến (mỗi 200ms) |
| `OBSTACLE_DETECTED` | `{robotNode, obstacleNode, robotDir, distance_cm}` | Phát hiện vật cản |
| `COMPLETED` | `{robotNode, robotDir}` | Đã đến đích |
| `PONG` | `{}` | Phản hồi heartbeat |

---

## 🔄 Dynamic Re-routing Flow

```
Robot đang chạy          Web Browser
      │                       │
      │ HC-SR04 < 25cm        │
      │ → DỪNG                │
      │                       │
      ├──OBSTACLE_DETECTED──→│
      │  {robotNode, obsNode} │
      │                       ├── Thêm vật cản vào map
      │                       ├── Chạy lại AI từ robotNode → đích
      │                       ├── Animation đường mới
      │                       │
      │←──────ROUTE──────────┤
      │  {path:[...], dir}    │
      │                       │
      ├── Nạp path mới        │
      ├── Tiếp tục chạy       │
      │                       │
```

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
4. [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer) — WebSocket cho ESP32
5. [ArduinoJson](https://arduinojson.org/) — JSON serialization cho embedded

---

*Đồ án Trí tuệ Nhân tạo — 2026*