# 🤖 AI Delivery Robot — Hướng dẫn cho AI Assistant

> File này cung cấp context cho AI coding assistant (Gemini, Copilot, Claude, etc.) khi làm việc với dự án.

---

## 📋 Tổng quan dự án

**Mục đích**: Xe robot 4WD dò line giao hàng tự hành trên lưới 4×5 (20 node), điều khiển bởi thuật toán AI tìm đường qua giao diện web.

**Tech Stack**:
- **Firmware**: C++ (Arduino Framework) trên ESP32-WROOM-32
- **Web**: Vanilla HTML/CSS/JS + TailwindCSS (runtime) + Chart.js, host trên LittleFS của ESP32
- **Giao tiếp**: WebSocket real-time + HTTP REST endpoints
- **Mạng**: ESP32 Access Point (`ESP32-Car` / `12345678`), IP `192.168.4.1`, **KHÔNG CÓ INTERNET**

---

## 📁 File chính & mục đích

| File | Ngôn ngữ | Chức năng |
|---|---|---|
| `ai-firmware.ino` | C++ | Main: WiFi AP, WebSocket server, HTTP endpoints, telemetry, mode switching |
| `do_line.h` | C | Header: PID struct, UIMode enum (MANUAL/LINE_ONLY/DELIVERY/AI_ROUTE) |
| `do_line.cpp` | C++ | **Core logic**: PID dò line, phát hiện giao lộ, tính hướng rẽ, encoder PID, obstacle detect |
| `route_interpreter.h/cpp` | C++ | Legacy route interpreter (state machine), **hiện KHÔNG được gọi trong main loop** |
| `data/index.html` | HTML | Dashboard: 3 tabs (Thủ công/Tự động/Thống kê), SVG grid map |
| `data/script.js` | JS | **Toàn bộ logic web**: AI algorithms (BFS/DFS/UCS/A*/Greedy), animation engine, WebSocket client, map tools |
| `data/style.css` | CSS | Custom styles (node colors, animations, robot indicator) |
| `data/tailwind.min.js` | JS | TailwindCSS runtime (451KB — cân nhắc thay bằng CSS thuần nếu heap ESP32 quá tải) |
| `data/chart.min.js` | JS | Chart.js cho biểu đồ so sánh thuật toán |

---

## 🧭 Hệ hướng (Direction System) — QUAN TRỌNG

Đây là điểm dễ nhầm nhất trong codebase:

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
// diff == 0 → STRAIGHT (không rẽ)
// diff == 1 → quay TRÁI (counterclockwise)   ← QUAN TRỌNG: không phải RIGHT!
// diff == 3 → quay PHẢI (clockwise)           ← QUAN TRỌNG: không phải LEFT!
// diff == 2 → U-TURN (180°)
```

> ⚠️ **Lưu ý**: Vì C++ dirs `0,1,2,3` = `down,right,up,left` KHÔNG phải thứ tự CW, nên `diff=1` là CCW (LEFT) và `diff=3` là CW (RIGHT). Đây là ngược với trực giác.

---

## 🔌 GPIO Pin Map

```
Motor Left:   IN1=12, IN2=14, ENA=13 (PWM)
Motor Right:  IN3=4,  IN4=2,  ENB=15 (PWM)
Line sensors: L2=34, L1=32, M=33, R1=27, R2=25  (LOW = trên line)
Encoders:     Left=26, Right=22 (interrupt RISING)
HC-SR04:      Trig=21, Echo=19
Servo:        Pin=18 (gripper: Open=120°, Close=175°)
```

---

## 🏭 Logic chính trong `do_line.cpp`

### PID Line Following
- **Chỉ dùng 3 mắt giữa**: L1, M, R1
- L2, R2 **chỉ** dùng để phát hiện giao lộ (vạch ngang)
- PID velocity control trên encoder (EMA filtered)

### Intersection Detection
- Trigger: `L2 || R2` (mắt ngoài thấy line ngang)
- Debounce: 1500ms giữa 2 lần phát hiện
- Tại giao lộ: Dừng → Tính hướng → Rẽ → Tiến 8cm → Tiếp tục dò line

### Path Index Logic
- `currentPath[]` chứa danh sách node (ví dụ: [15, 10, 5, 0])
- Tại mỗi giao lộ:
  1. **Tính hướng TRƯỚC**: `getTargetDirection(path[index], path[index+1])`
  2. **Rẽ theo diff**
  3. **Tăng index SAU**: `currentPathIndex++`
  4. Tiến 8cm qua vùng giao lộ

### Obstacle Handling
- HC-SR04 polling mỗi 60ms
- Latched flag: 3 lần liên tiếp < 20cm → dừng + báo web
- Gửi `OBSTACLE_DETECTED` qua WebSocket kèm `robotNode` và `robotDir`

### Auto-search at Start
- Khi `is_auto_running && !seen_line_ever`:  xe bò chậm (v_search) tới khi tìm thấy line
- Xe được đặt vài cm trước node xuất phát

---

## 🌐 Logic chính trong `script.js`

### AI Algorithms
- Tất cả 5 thuật toán chạy client-side (trong browser)
- Trả về `{path, steps, visitedCount, cost}`
- `steps[]` chứa từng bước explore/frontier để animation

### Animation Engine
- Multi-segment: hỗ trợ giao hàng đa điểm
- Auto-play khi nhấn "MÔ PHỎNG"
- Flow edges animation (SVG line drawing)
- `anim.playing` = đang step-by-step; `anim.active` = đã khởi tạo

### Robot Indicator (Mũi tên xanh)
- SVG group trên `robotLayer`
- Click để xoay hướng (0→1→2→3→0)
- `updateRobotIndicator(nodeId, dir)` di chuyển mũi tên
- `resetRobotToStart()` reset về start node (gọi khi COMPLETED, ESTOP)

### WebSocket Client
- Auto-reconnect mỗi 3 giây
- Ping/Pong mỗi 5 giây
- Handle: TELEMETRY, ROUTE_ACK, COMPLETED, OBSTACLE_DETECTED

---

## ⚠️ Gotchas & Known Issues

1. **Không có Internet**: ESP32 chạy AP mode, web load từ LittleFS. KHÔNG dùng CDN online.
2. **tailwind.min.js quá lớn (451KB)**: Có thể gây heap overflow trên ESP32. Nếu web không load → cân nhắc thay bằng CSS thuần.
3. **route_interpreter.cpp không được dùng**: Logic route đã chuyển hoàn toàn sang `do_line.cpp`. File giữ lại cho tương thích.
4. **Motor pins trùng**: `ai-firmware.ino` và `do_line.cpp` đều define IN1-IN4/ENA/ENB. Nếu sửa pin phải sửa CẢ HAI file.
5. **Hệ số 1.5x trong spin**: `spin_left_deg(deg)` thực tế quay `deg * 1.5` do target calculation: `const double target = 1.5*deg * PI / 180.0`. Khi truyền 62° → quay ~93° thực tế.
6. **Encoders debounce**: MIN_EDGE_US = 1500μs, nếu motor quay nhanh có thể miss xung.

---

## 🛠 Quy tắc khi sửa code

### Firmware (C++)
- Luôn kiểm tra `is_auto_running` trước khi cho motor chạy
- Khi thêm mode mới → cập nhật enum `UIMode` trong `do_line.h`
- Khi sửa GPIO → sửa cả `ai-firmware.ino` và `do_line.cpp`
- Dùng `extern` để chia sẻ biến giữa `.ino` và `.cpp`

### Web (JS)
- Khi sửa direction mapping → kiểm tra cả `JS_TO_CPP_DIR`, `CPP_TO_JS_DIR`, `getDirBetweenNodes()`, `pathToCommands()`
- `updateNodeVisuals()` được gọi rất thường xuyên → không đặt logic nặng trong đó
- Animation state: kiểm tra `anim.playing` (đang phát) vs `anim.active` (đã init)
- Toast: `showToast(msg, "success"|"error"|"info")`

### Upload web files
1. Đảm bảo files nằm trong `ai-firmware/data/`
2. Dùng **ESP32 Sketch Data Upload** plugin
3. Partition scheme phải có đủ SPIFFS/LittleFS space (~1MB)

---

## 📐 Node Coordinate Map

```
Node layout (4 rows × 5 cols):

 0(30,30)   1(100,30)  2(170,30)  3(240,30)  4(310,30)
 5(30,100)  6(100,100) 7(170,100) 8(240,100) 9(310,100)
10(30,170) 11(100,170)12(170,170)13(240,170)14(310,170)
15(30,240) 16(100,240)17(170,240)18(240,240)19(310,240)

Default graph: full 4×5 grid (all adjacent nodes connected)
Coords used in both firmware (node_coords[20][2]) and JS (coords{})
```

---

## 🧪 Test Checklist

- [ ] Web load thành công tại `192.168.4.1`
- [ ] WebSocket connect (chữ "Trực tuyến" xanh)
- [ ] 5 thuật toán animation chạy đúng
- [ ] So sánh thuật toán hiển thị biểu đồ
- [ ] Mũi tên xanh xoay hướng khi click
- [ ] Nạp lộ trình → ESP32 nhận ACK
- [ ] Robot dò line thẳng ổn định
- [ ] Robot phát hiện giao lộ (L2/R2)
- [ ] Robot rẽ đúng hướng (trái/phải/thẳng/U-turn)
- [ ] Robot dừng khi đến đích → COMPLETED
- [ ] Mũi tên reset khi COMPLETED / ESTOP
- [ ] Phát hiện vật cản → báo web → re-route
- [ ] E-STOP dừng mọi thứ

---

> *Cập nhật: 29/03/2026*
