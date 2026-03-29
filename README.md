# 🤖 AI Delivery Robot — Xe Robot Giao hàng Tự hành

> **Đồ án môn Trí tuệ Nhân tạo** — Ứng dụng thuật toán tìm kiếm AI (BFS, DFS, UCS, A\*, Greedy) điều khiển xe robot dò line giao hàng tự động trên lưới bản đồ 4×5.

---

## 📑 Mục lục

- [Tổng quan](#-tổng-quan)
- [Demo](#-demo)
- [Kiến trúc hệ thống](#-kiến-trúc-hệ-thống)
- [Phần cứng](#-phần-cứng)
- [Cấu trúc dự án](#-cấu-trúc-dự-án)
- [Thuật toán AI](#-thuật-toán-ai)
- [Firmware ESP32](#-firmware-esp32)
- [Web Dashboard](#-web-dashboard)
- [Giao tiếp Web ↔ ESP32](#-giao-tiếp-web--esp32)
- [Hướng dẫn cài đặt](#-hướng-dẫn-cài-đặt)
- [Hướng dẫn sử dụng](#-hướng-dẫn-sử-dụng)
- [Thông số kỹ thuật](#-thông-số-kỹ-thuật)
- [Xử lý sự cố](#-xử-lý-sự-cố)
- [Thành viên nhóm](#-thành-viên-nhóm)
- [Tài liệu tham khảo](#-tài-liệu-tham-khảo)

---

## 🌟 Tổng quan

Hệ thống gồm **2 thành phần chính**:

1. **Web Dashboard** — Giao diện trực quan trên trình duyệt, chạy trên chính ESP32 (offline, không cần internet):
   - Bản đồ lưới 4×5 (20 node) có thể tùy chỉnh (thêm/xóa cạnh, đặt vật cản, trọng số)
   - Mô phỏng animation 5 thuật toán AI tìm đường
   - So sánh hiệu năng thuật toán (biểu đồ + bảng)
   - Giao hàng đơn/đa điểm với tối ưu thứ tự (Greedy nearest)
   - Điều khiển thủ công (D-Pad 8 hướng)
   - Telemetry real-time (tốc độ, cảm biến, vật cản)

2. **Robot 4WD** — Xe dò line tự hành dùng ESP32:
   - 5 cảm biến hồng ngoại TCRT5000 (L2, L1, M, R1, R2)
   - PID line following (3 mắt giữa L1/M/R1)
   - Phát hiện giao lộ (2 mắt ngoài L2/R2)
   - Cảm biến siêu âm HC-SR04 tránh vật cản
   - Encoder đo vận tốc + quãng đường
   - Servo SG90 điều khiển gripper

### Luồng hoạt động

```
Người dùng → Vẽ bản đồ trên Web → Chọn thuật toán AI → Mô phỏng
    → Gửi lộ trình cho Robot (WebSocket) → Robot dò line + rẽ tại giao lộ
    → Phát hiện vật cản → Gửi về Web → Web tính đường mới → Robot tiếp tục
```

---

## 🎬 Demo

### Giao diện Web Dashboard

| Tab Tự động | Tab Thủ công |
|:---:|:---:|
| Bản đồ AI + animation thuật toán | D-Pad điều khiển + cảm biến |

### Thuật toán AI trên Web

- **BFS**: Duyệt theo lớp, tìm đường ngắn nhất (số bước)
- **DFS**: Đi sâu trước, nhanh nhưng không tối ưu
- **UCS**: Mở rộng chi phí nhỏ nhất, tối ưu khi có trọng số
- **A\***: f(n) = g(n) + h(n), tối ưu + nhanh nhờ heuristic
- **Greedy**: f(n) = h(n), nhanh nhưng không đảm bảo tối ưu

---

## 🏗 Kiến trúc hệ thống

```
┌──────────────────────────────────────────────────────────────┐
│                    TẦNG GIAO DIỆN (WEB)                      │
│  ┌─────────────┐  ┌────────────────┐  ┌──────────────────┐   │
│  │  Map Editor │  │ AI Algorithms  │  │   Dashboard      │   │
│  │  (SVG Grid) │  │ BFS/DFS/UCS    │  │ Telemetry +      │   │
│  │  Start/End  │  │ A*/Greedy      │  │ Manual Control   │   │
│  │  Obstacles  │  │ Animation      │  │ D-Pad + Stats    │   │
│  └─────────────┘  └────────────────┘  └──────────────────┘   │
│                         │ WebSocket (Wi-Fi AP)               │
├─────────────────────────┼────────────────────────────────────┤
│                    TẦNG XỬ LÝ (ESP32)                        │
│  ┌───────────-──┐  ┌────────────────┐  ┌──────────────────┐  │
│  │  Web Server  │  │ Route Manager  │  │ Motion Control   │  │
│  │  LittleFS    │  │ Path → Turn    │  │ PID + Encoder    │  │
│  │  WebSocket   │  │ State Machine  │  │ Obstacle Detect  │  │
│  └───────────-──┘  └────────────────┘  └──────────────────┘  │
├──────────────────────────────────────────────────────────────┤
│  TẦNG CHẤP HÀNH              │  TẦNG CẢM BIẾN                │
│  • 4× DC Motor (L298N)       │  • 5× TCRT5000 (dò line)      │
│  • Servo SG90 (gripper)      │  • HC-SR04 (siêu âm)          │ 
│                              │  • 2× Encoder (quang)         │
├──────────────────────────────────────────────────────────────┤
│  TẦNG NGUỒN: 2× Pack 18650 → LM2596 → 5V (logic) + 7.4V      │
└──────────────────────────────────────────────────────────────┘
```

---

## 🔧 Phần cứng

| Thành phần | Linh kiện | Chức năng |
|---|---|---|
| Vi điều khiển | **ESP32-WROOM-32** | Dual-core 240MHz, Wi-Fi AP |
| Động cơ | 4× DC Geared Motor 3-6V | Giảm tốc ~1:48 |
| Driver | **L298N** Dual H-Bridge | Điều khiển 4 motor |
| Cảm biến line | **5× TCRT5000** | L2, L1, M, R1, R2 |
| Siêu âm | **HC-SR04** | Phát hiện vật cản 2-400cm |
| Servo | **SG90** (0°-180°) | Gripper giao hàng |
| Encoder | 2× Encoder quang | Đo vận tốc + quãng đường |
| Nguồn | 2× Pack 18650 (3S) | 5V logic + 7.4V motor |
| Khung | Tấm Mica 2 tầng | Tầng dưới: motor; Trên: ESP32 |

### Sơ đồ chân GPIO

| Chức năng | GPIO | Ghi chú |
|---|---|---|
| Motor Left: IN1/IN2/ENA | 12/14/13 | L298N kênh A |
| Motor Right: IN3/IN4/ENB | 4/2/15 | L298N kênh B |
| Line L2/L1/M/R1/R2 | 34/32/33/27/25 | TCRT5000 (LOW = trên line) |
| Encoder Left/Right | 26/22 | Interrupt RISING |
| HC-SR04 Trig/Echo | 21/19 | Siêu âm |
| Servo Gripper | 18 | SG90 |

---

## 📁 Cấu trúc dự án

```
ai-final_project/
├── README.md                    # Tài liệu này
├── agents.md                    # Hướng dẫn cho AI assistant
├── TEAM_TASKS.md                # Phân công nhóm
├── de_cuong_do_an_AI.md         # Đề cương đồ án
│
└── ai-firmware/                 # Arduino project (ESP32)
    ├── ai-firmware.ino          # Main: WiFi AP, WebSocket, HTTP endpoints
    ├── do_line.h                # Header: PID struct, UIMode enum
    ├── do_line.cpp              # Core: PID line follow, intersection, turns
    ├── route_interpreter.h      # Header: Route state machine API
    ├── route_interpreter.cpp    # Route interpreter (legacy, chưa dùng)
    │
    └── data/                    # Web files (upload vào LittleFS)
        ├── index.html           # Dashboard HTML (~34KB)
        ├── script.js            # AI algorithms + UI logic (~58KB)
        ├── style.css            # Custom CSS (~10KB)
        ├── tailwind.min.js      # TailwindCSS runtime (~451KB)
        └── chart.min.js         # Chart.js cho biểu đồ (~205KB)
```

---

## 🧠 Thuật toán AI

### 5 thuật toán tìm đường (chạy trên trình duyệt)

| Thuật toán | Loại | Tối ưu? | Cấu trúc DL | Đặc điểm |
|---|---|---|---|---|
| **BFS** | Uninformed | ✅ (đều bước) | Queue | Duyệt theo lớp |
| **DFS** | Uninformed | ❌ | Stack | Đi sâu trước |
| **UCS** | Uninformed | ✅ | Priority Queue | Chi phí nhỏ nhất |
| **A\*** | Informed | ✅ | Priority Queue | g(n) + h(n) |
| **Greedy** | Informed | ❌ | Priority Queue | h(n) only |

**Heuristic**: Manhattan Distance — `h(n) = |x₁ - x₂| + |y₁ - y₂|`

### Tính năng nâng cao

- **Animation từng bước**: Hiển thị explored, frontier, current node
- **So sánh 5 thuật toán**: Biểu đồ + bảng xếp hạng tự động
- **Giao hàng đa điểm**: Greedy nearest-by-traversal (chạy algo, target nào tìm thấy trước → đi đó)
- **Chế độ Hỏa tốc**: Ưu tiên giao đơn cuối cùng được chọn
- **Dynamic Re-routing**: Gặp vật cản → tính đường mới tự động

---

## ⚙ Firmware ESP32

### Cấu trúc mã nguồn

#### `ai-firmware.ino` — Main Controller
- WiFi Access Point (`ESP32-Car` / `12345678`)
- AsyncWebServer + WebSocket (`/ws`)
- HTTP endpoints: `/forward`, `/left`, `/stop`, `/estop`, `/deliver`, `/setMode`, `/api/stats`
- Telemetry broadcasting (200ms interval)
- Xử lý ROUTE, STOP, ESTOP, RESUME từ WebSocket

#### `do_line.cpp` — Motion Control Core
- **PID Line Following**: 3 mắt giữa (L1, M, R1) cho dò line mượt
- **Intersection Detection**: 2 mắt ngoài (L2 hoặc R2) phát hiện giao lộ
- **Turn Logic**: Tính hướng rẽ từ node coords, diff-based turn (CW/CCW)
- **Encoder PID**: Closed-loop velocity control (EMA filtered)
- **Obstacle Detection**: HC-SR04 polling, latched obstacle flag
- **Auto-search**: Bò chậm tìm line khi `is_auto_running && !seen_line_ever`

#### Hệ hướng (Direction System)

```
C++ directions: 0=down, 1=right, 2=up, 3=left
JS directions:  0=up,   1=right, 2=down, 3=left

Mapping:  JS_TO_CPP = [2, 1, 0, 3]
          CPP_TO_JS = [2, 1, 0, 3]

Turn calculation:
  diff = (targetDir - currentDir + 4) % 4
  diff == 1 → quay TRÁI (CCW)    // 1 step counterclockwise
  diff == 3 → quay PHẢI (CW)     // 1 step clockwise
  diff == 2 → quay 180° (U-turn)
  diff == 0 → đi thẳng
```

### State Machine

```
MODE_MANUAL ──(nhận ROUTE)──→ MODE_AI_ROUTE
                                    │
                                    ├── Dò line (PID L1/M/R1)
                                    ├── Giao lộ (L2||R2) → Tính hướng → Rẽ → Tiến
                                    ├── Vật cản (HC-SR04 < 20cm) → Dừng → Báo Web
                                    └── Đích → COMPLETED → MODE_MANUAL
```

---

## 🌐 Web Dashboard

### 3 Tab chính

| Tab | Chức năng |
|---|---|
| **🤚 Thủ công** | D-Pad 8 hướng, điều chỉnh tốc độ, gripper, cảm biến real-time |
| **🤖 Tự động** | Bản đồ AI, thuật toán, animation, giao hàng, so sánh |
| **📊 Thống kê** | KPI: số đơn giao, thời gian TB, hiệu suất, biểu đồ |

### Map Tools

| Tool | Biểu tượng | Chức năng |
|---|---|---|
| Xuất phát | ⭐ | Click node → đặt điểm xuất phát |
| Điểm giao | 🔴 | Click node → thêm/xóa điểm đích (tối đa 6) |
| Vật cản | 🚫 | Click node → toggle vật cản |
| Edge | 🔗 | Click 2 node → thêm/xóa cạnh |
| Trọng số | ⚖️ | Click 2 node kề → đặt chi phí tùy chỉnh |

### Robot Indicator (Mũi tên xanh)

- Click mũi tên → xoay hướng đầu xe (Lên/Phải/Xuống/Trái)
- Tự động di chuyển theo telemetry khi robot đang chạy
- Reset về start node khi: E-STOP, hoàn thành, thay đổi start node

---

## 📡 Giao tiếp Web ↔ ESP32

### WebSocket Messages

| Direction | Type | Payload | Mô tả |
|---|---|---|---|
| Web → ESP | `ROUTE` | `{path, initialDir, commands}` | Gửi lộ trình |
| Web → ESP | `ESTOP` | `{}` | Dừng khẩn cấp |
| Web → ESP | `RESUME` | `{}` | Tiếp tục |
| ESP → Web | `TELEMETRY` | `{speedL, speedR, obstacle, sensors, robotNode, robotDir}` | Dữ liệu cảm biến |
| ESP → Web | `ROUTE_ACK` | `{commands: N}` | Xác nhận nạp route |
| ESP → Web | `COMPLETED` | `{robotNode, robotDir}` | Hoàn thành lộ trình |
| ESP → Web | `OBSTACLE_DETECTED` | `{obstacleNode, robotNode, robotDir}` | Phát hiện vật cản |

### Dynamic Re-routing Flow

```
1. Robot đang chạy → HC-SR04 phát hiện vật cản < 20cm
2. Robot dừng → Gửi OBSTACLE_DETECTED (kèm vị trí) về Web
3. Web nhận → Đánh dấu vật cản trên bản đồ
4. Web chạy lại thuật toán AI từ vị trí robot hiện tại
5. Web gửi ROUTE mới (rerouted: true) cho ESP32
6. Robot nhận → Xóa route cũ → Nạp route mới → Tiếp tục
```

---

## 🚀 Hướng dẫn cài đặt

### Yêu cầu

- [Arduino IDE 2.x](https://www.arduino.cc/en/software) hoặc PlatformIO
- Board: **ESP32 Dev Module**
- Thư viện Arduino:
  - `WiFi.h` (built-in)
  - `ESPAsyncWebServer` + `AsyncTCP`
  - `ESP32Servo`
  - `ArduinoJson` (v6+)
  - `LittleFS` (built-in)

### Bước 1: Clone project

```bash
git clone <repo-url>
cd ai-final_project
```

### Bước 2: Upload Web files vào LittleFS

1. Cài plugin **ESP32 Sketch Data Upload** cho Arduino IDE
2. Đảm bảo thư mục `ai-firmware/data/` chứa: `index.html`, `script.js`, `style.css`, `tailwind.min.js`, `chart.min.js`
3. Menu: **Tools → ESP32 Sketch Data Upload** → chờ upload hoàn tất

### Bước 3: Upload Firmware

1. Mở `ai-firmware/ai-firmware.ino` trong Arduino IDE
2. Chọn Board: **ESP32 Dev Module**
3. Partition Scheme: **Default 4MB with spiffs** (hoặc tương đương)
4. Upload Speed: 921600
5. Nhấn **Upload**

### Bước 4: Kết nối

1. Kết nối WiFi: **`ESP32-Car`** / mật khẩu: **`12345678`**
2. Mở trình duyệt → **`http://192.168.4.1`**
3. Dashboard sẽ hiển thị → kiểm tra trạng thái "Trực tuyến" ở góc dưới trái

---

## 📖 Hướng dẫn sử dụng

### Chế độ Tự động (AI Route)

1. **Chọn điểm xuất phát**: Dropdown "Vị trí xuất phát" hoặc click tool ⭐ → click node
2. **Đặt điểm giao**: Click tool 🔴 → click các node đích (tối đa 6 đơn)
3. **Chọn thuật toán**: Dropdown (A\*, BFS, DFS, UCS, Greedy)
4. **Mô phỏng**: Nhấn `MÔ PHỎNG THUẬT TOÁN` → xem animation từng bước
5. **Đặt hướng xe**: Click mũi tên xanh trên bản đồ → xoay đến hướng mong muốn
6. **Đặt xe lên track**: Đặt xe phía dưới node xuất phát, cùng hướng đã chọn
7. **Gửi lộ trình**: Nhấn `NẠP LỘ TRÌNH & CHO XE CHẠY`
8. **Xe sẽ**:
   - Bò chậm tìm line → PID dò line → Phát hiện giao lộ → Rẽ đúng hướng → Tiếp tục
   - Đến đích → Dừng → Gửi COMPLETED → Mũi tên reset về start

### Chế độ Thủ công

- D-Pad 8 hướng điều khiển trực tiếp
- Nút gripper Open/Close
- Điều chỉnh tốc độ tịnh tiến / xoay

### Dừng khẩn cấp

- Nút **E-STOP** (đỏ lớn) → dừng mọi thứ, reset về manual

---

## 📊 Thông số kỹ thuật

| Thông số | Giá trị |
|---|---|
| Bản đồ | 4×5 grid = 20 node |
| Khoảng cách giữa 2 giao lộ | ~20-30cm |
| Tốc độ dò line | ~0.3-0.5 m/s |
| PID gains | Kp=300, Ki=8, Kd=0 (tune theo track) |
| Góc rẽ | 62° × 1.5 hệ số ≈ 93° thực tế |
| Ngưỡng vật cản | 20cm (HC-SR04) |
| Debounce giao lộ | 1500ms |
| Telemetry interval | 200ms |
| WebSocket timeout | Auto-reconnect 3s |
| ESP32 AP IP | `192.168.4.1` |
| Tổng dung lượng web | ~759KB (LittleFS) |

---

## 🔍 Xử lý sự cố

| Vấn đề | Nguyên nhân | Giải pháp |
|---|---|---|
| Không vào được web | Sai IP hoặc chưa kết nối WiFi | Kết nối WiFi `ESP32-Car`, truy cập `192.168.4.1` |
| Web trắng/load chậm | LittleFS chưa upload | Upload lại data folder qua Sketch Data Upload |
| Xe không chạy khi đặt xuống | Chưa thấy line | Đặt xe gần line hơn hoặc chờ xe bò chậm tìm |
| Rẽ sai hướng | Hướng ban đầu trên web không khớp thực tế | Click mũi tên xanh xoay đúng hướng xe đang hướng |
| Rẽ quá nhiều/ít | Góc spin chưa đúng | Tăng/giảm `62.0` trong `do_line.cpp` dòng ~472 |
| Mũi tên bị kẹt | Sau animation `anim.active` = true | Nhấn E-STOP hoặc click node trên bản đồ để reset |
| Mất kết nối WS | ESP32 quá tải | Kiểm tra Serial monitor, restart ESP32 |

---

## 👨‍💻 Thành viên nhóm

| Thành viên | MSSV | Vai trò |
|---|---|---|
| Đinh Quốc Anh | 24133003 | Web Dashboard & Giao tiếp |
| Nguyễn Chiến | 24133007 | Firmware ESP32 & WebSocket |
| Lý Gia Hân | 24133016 | Robot Control (PID & Sensors) |
| Phan Tuấn Thanh | 24133054 | AI Algorithms Engine |

---

## 📚 Tài liệu tham khảo

1. **Russell, S. & Norvig, P.** (2021). *Artificial Intelligence: A Modern Approach* (4th ed.) — Chương 3-4: Search Algorithms.
2. **Hart, P.E., Nilsson, N.J. & Raphael, B.** (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." — Bài báo gốc A\*.
3. **Espressif Systems.** ESP32 Technical Reference Manual.
4. **ESPAsyncWebServer** — [GitHub](https://github.com/me-no-dev/ESPAsyncWebServer)
5. **ArduinoJson** — [GitHub](https://github.com/bblanchon/ArduinoJson)

---

## 📄 License

Dự án phục vụ mục đích học tập — Đồ án môn Trí tuệ Nhân tạo.

> *Ngày cập nhật: 29/03/2026*