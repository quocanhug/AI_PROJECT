# 🤖 AI Delivery Robot — Hướng dẫn cho AI Assistant

> File này cung cấp context cho AI coding assistant khi làm việc với dự án.
> Cập nhật: 30/03/2026

---

## 📋 Tổng quan dự án

**Mục đích**: Xe robot 4WD dò line giao hàng tự hành trên lưới 4×5 (20 node), điều khiển bởi thuật toán AI tìm đường qua giao diện web.

**Tech Stack**:
- **Firmware**: C++ (Arduino Framework) trên ESP32-WROOM-32
- **Web**: Vanilla HTML/CSS/JS + TailwindCSS (runtime) + Chart.js, host trên LittleFS của ESP32
- **Giao tiếp**: WebSocket real-time + HTTP REST endpoints
- **Mạng**: ESP32 Access Point (`ESP32-Car` / `12345678`), IP `192.168.4.1`, **KHÔNG CÓ INTERNET**

---

## 📁 Cấu trúc file

```
ai-firmware/
├── ai-firmware.ino      # Main: WiFi AP, WebSocket, HTTP endpoints, telemetry, mode switching
├── do_line.h            # Header: PID struct, UIMode enum, function declarations
├── do_line.cpp          # Core logic: PID dò line, giao lộ, rẽ, encoder, obstacle
└── data/
    ├── index.html       # Dashboard: 3 tabs (Thủ công/Tự động/Thống kê), SVG grid map
    ├── script.js        # AI algorithms (BFS/DFS/UCS/A*/Greedy), animation, WebSocket
    ├── style.css        # Custom styles (node colors, animations, robot indicator)
    ├── tailwind.min.js  # TailwindCSS runtime (451KB)
    └── chart.min.js     # Chart.js cho biểu đồ so sánh
```

> **Lưu ý**: `route_interpreter.cpp/h` đã được xóa (30/03/2026). Logic routing nằm hoàn toàn trong `do_line.cpp`.

---

## 🧭 Hệ hướng (Direction System) — QUAN TRỌNG

```
C++ (firmware):   0=down, 1=right, 2=up, 3=left    ← KHÔNG theo CW chuẩn
JS  (web):        0=up,   1=right, 2=down, 3=left   ← Theo CW chuẩn

Mapping giữa JS ↔ C++:
  JS_TO_CPP_DIR = [2, 1, 0, 3]     // JS up(0) → CPP up(2)
  CPP_TO_JS_DIR = [2, 1, 0, 3]     // CPP down(0) → JS down(2)
```

### Turn calculation (trong C++)

```cpp
int diff = (targetDir - currentDir + 4) % 4;
// diff == 0 → STRAIGHT
// diff == 1 → quay TRÁI (CCW)    ← QUAN TRỌNG: diff=1 là LEFT!
// diff == 3 → quay PHẢI (CW)     ← QUAN TRỌNG: diff=3 là RIGHT!
// diff == 2 → U-TURN (180°)
```

---

## 🔌 GPIO Pin Map

```
Motor Left:   IN1=12, IN2=14, ENA=13 (PWM)
Motor Right:  IN3=4,  IN4=2,  ENB=15 (PWM)
Line sensors: L2=34, L1=32, M=33, R1=27, R2=25  (LOW = trên line)
Encoders:     Left=26, Right=22 (interrupt RISING, debounce 1500μs)
HC-SR04:      Trig=21, Echo=19
Servo:        Pin=18 (gripper: Open=120°, Close=175°)
```

---

## 🏭 Luồng hoạt động chính

### Web gửi route cho Robot

```
1. User chọn Start + Target trên bản đồ SVG
2. User chọn thuật toán (BFS/DFS/UCS/A*/Greedy)
3. Nhấn "MÔ PHỎNG" → animation từng bước
4. Nhấn "NẠP LỘ TRÌNH" → gửi WebSocket:
   { type: "ROUTE", path: [15, 10, 5, 0, 1], initialDir: 2 }
5. ESP32 nhận → nạp vào currentPath[], set MODE_AI_ROUTE
6. do_line_loop() chạy PID dò line
7. Phát hiện giao lộ (L2 && R2) → tính hướng rẽ → spin → tiếp tục
8. Đến đích → gửi COMPLETED → web hiển thị kết quả
```

### Dynamic Re-routing (vật cản bất ngờ)

```
1. HC-SR04 phát hiện vật cản < 25cm (latched sau 2 lần liên tiếp)
2. Robot dừng → gửi OBSTACLE_DETECTED { robotNode, obstacleNode, robotDir }
3. Web nhận → thêm node vào obstacles set → tính đường mới
4. Web gửi ROUTE mới { path: [...], initialDir: ... }
5. Robot nhận → nạp path mới → tiếp tục chạy
```

---

## 🔧 Logic chi tiết trong `do_line.cpp`

### PID Line Following
- **Chỉ dùng 3 mắt giữa**: L1, M, R1 → v_boost/v_hard điều chỉnh tốc độ từng bánh
- L2, R2 **chỉ** dùng để phát hiện giao lộ
- PID velocity control (encoder feedback) → EMA filtered

### Intersection Detection & Turn
- Trigger: `L2 && R2` (cả 2 mắt ngoài thấy line ngang)
- Debounce: 500ms
- Flow: Dừng → Tiến 3cm centering → Tính hướng → Spin (60° × 1.5 ≈ 90°) → Tìm line (tiến max 10cm) → PID

### Obstacle Handling
- HC-SR04 median filter (3 lần), polling mỗi 25ms
- Hysteresis: ON < 25cm, OFF > 30cm, cần 2 lần liên tiếp
- MODE_AI_ROUTE: dừng + báo web (web re-route)
- Các mode khác: physical avoidance (spin + forward sequence)

### Auto-search at Start
- Khi `is_auto_running && !seen_line_ever`: bò chậm v_search=0.2 tới khi tìm line

---

## 🌐 Logic chi tiết trong `script.js`

### AI Algorithms
- 5 thuật toán chạy client-side, trả về `{path, steps, visitedCount, cost}`
- Multi-target: greedy nearest-by-traversal (tìm target gần nhất trước)
- `steps[]` chứa explored/frontier sets cho animation

### Direction Mapping
- `JS_TO_CPP_DIR[jsDir]` khi gửi cho ESP32
- `CPP_TO_JS_DIR[cppDir]` khi nhận từ ESP32
- `getDirBetweenNodes(a, b)` tính hướng JS giữa 2 node

### WebSocket Protocol
| Message | Hướng | Mục đích |
|---|---|---|
| `PING/PONG` | Bi-directional | Heartbeat mỗi 5s |
| `ROUTE` | Web → ESP32 | Gửi path + initialDir |
| `ROUTE_ACK` | ESP32 → Web | Xác nhận đã nạp route |
| `TELEMETRY` | ESP32 → Web | Sensor data mỗi 200ms |
| `OBSTACLE_DETECTED` | ESP32 → Web | Vật cản + vị trí |
| `COMPLETED` | ESP32 → Web | Đã đến đích |
| `ESTOP` | Web → ESP32 | Dừng khẩn cấp |
| `STOP` | Web → ESP32 | Dừng bình thường |
| `RESUME` | Web → ESP32 | Tiếp tục route |

---

## ⚠️ Gotchas & Known Issues

1. **Không có Internet**: ESP32 chạy AP mode, web load từ LittleFS. KHÔNG dùng CDN.
2. **tailwind.min.js quá lớn (451KB)**: Tổng web files ~762KB trên ~1MB LittleFS.
3. **Motor pins trùng**: `ai-firmware.ino` và `do_line.cpp` đều define IN1-IN4/ENA/ENB. Sửa pin phải sửa CẢ HAI.
4. **Hệ số 1.5x trong spin**: `spin_left_deg(deg)` quay `deg × 1.5` do target calculation.
5. **Encoders debounce**: MIN_EDGE_US = 1500μs.
6. **Sonar blocking**: `readDistanceCM_filtered()` block ~30ms (3× pulseIn + delay).

---

## 🛠 Quy tắc khi sửa code

### Firmware (C++)
- Luôn kiểm tra `is_auto_running` trước khi cho motor chạy
- Kiểm tra `g_line_enabled` trong mọi vòng lặp blocking (spin, move_forward)
- Khi sửa GPIO → sửa cả `ai-firmware.ino` và `do_line.cpp`
- Dùng `extern` để chia sẻ biến giữa `.ino` và `.cpp`

### Web (JS)
- Direction mapping: kiểm tra `JS_TO_CPP_DIR`, `CPP_TO_JS_DIR`, `getDirBetweenNodes()`
- Animation state: `anim.playing` (đang phát) vs `anim.active` (đã init)
- Toast: `showToast(msg, "success"|"error"|"info")`
- Không đặt logic nặng trong `updateNodeVisuals()` (gọi rất thường xuyên)

### Upload web files
1. Files nằm trong `ai-firmware/data/`
2. Dùng **ESP32 Sketch Data Upload** plugin (LittleFS)
3. Partition scheme phải có đủ LittleFS space (~1MB)

---

## 📐 Node Coordinate Map

```
 0(30,30)   1(100,30)  2(170,30)  3(240,30)  4(310,30)
 5(30,100)  6(100,100) 7(170,100) 8(240,100) 9(310,100)
10(30,170) 11(100,170)12(170,170)13(240,170)14(310,170)
15(30,240) 16(100,240)17(170,240)18(240,240)19(310,240)

Grid: 4 rows × 5 cols, full adjacency (all neighbors connected)
Coords used in both firmware (node_coords[20][2]) and JS (coords{})
```
