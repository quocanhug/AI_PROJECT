# Xây dựng Hệ thống Xe Robot Giao hàng Tự hành AI

## Tổng quan

Dựa trên đề cương đồ án, phát triển hệ thống hiện có từ firmware (manual control + PID line following) thành hệ thống hoàn chỉnh gồm:
1. **Web Interface**: Map Editor, 5 thuật toán AI (BFS/DFS/UCS/A*/Greedy), Animation, Telemetry Dashboard
2. **Firmware ESP32**: WebSocket server, State Machine, Route Interpreter, Dynamic Re-routing

### Hiện trạng firmware
- ✅ WiFi AP mode + AsyncWebServer
- ✅ Manual control web UI (8 hướng + gripper)
- ✅ PID line following với 5 cảm biến TCRT5000
- ✅ Encoder-based distance/turns ([spin_left_deg](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#293-324), [spin_right_deg](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#325-359), [move_forward_distance](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#360-377))
- ✅ HC-SR04 obstacle detection với median filter + debounce + hysteresis
- ✅ Obstacle avoidance (hình chữ U)
- ❌ WebSocket communication
- ❌ Route interpreter / State machine
- ❌ Intersection detection → lệnh F/L/R
- ❌ Dynamic re-routing (gửi vật cản về web)
- ❌ Telemetry reporting
- ❌ Web AI algorithms + Animation

---

## Proposed Changes

### Component 1: Web Interface (HTML/CSS/JS)

Toàn bộ web interface được tạo dưới dạng file HTML/JS/CSS riêng, có thể host trên ESP32 SPIFFS hoặc mở trực tiếp trên browser (kết nối ESP32 qua WebSocket).

#### [NEW] [index.html](file:///d:/hk2nam2/ai/ai-final_project/web/index.html)

Giao diện chính gồm các tab/section:

**1. Map Editor:**
- Grid NxM (chọn size: 5×5 → 15×15)
- Click toggle: Empty ↔ Wall
- Click đặt Start (🟢) / End (🔴)
- Gán trọng số cho ô (1-9, phục vụ UCS/A*)
- Save/Load bản đồ JSON
- Random maze generator

**2. AI Search Engine (chạy client-side JS):**
- 5 thuật toán: BFS, DFS, UCS, A*, Greedy Best-First
- Heuristic: Manhattan / Euclidean (dropdown cho A*/Greedy)
- Canvas API render grid + animation từng bước
- Bảng màu: ⬜ Empty, ⬛ Wall, 🟦 Frontier, 🟨 Explored, 🟩 Path, 🟢 Start, 🔴 End
- Speed slider (slow → fast → instant)
- Real-time stats: Nodes explored, Frontier size, Path cost, Time(ms)

**3. So sánh thuật toán:**
- Nút "Compare All" → chạy 5 thuật toán → bảng so sánh
- Cột: Algorithm, Nodes Explored, Path Length, Path Cost, Time(ms), Optimal?

**4. Route Sender:**
- Convert path coordinates → commands F/L/R (dựa trên hướng robot)
- Preview commands trước khi gửi
- Nút "Send to Robot" → WebSocket JSON

**5. Telemetry Dashboard:**
- Real-time sensor data từ ESP32
- Tốc độ bánh L/R, quãng đường, bước hiện tại, trạng thái state machine
- Khoảng cách vật cản, trạng thái line sensors (5 indicator)

**6. Dynamic Re-routing:**
- Nhận OBSTACLE_DETECTED → flash đỏ ô vật cản mới
- Auto re-run AI algorithm → gửi route mới
- Event log

#### [NEW] [algorithms.js](file:///d:/hk2nam2/ai/ai-final_project/web/algorithms.js)

5 thuật toán AI triển khai trên JS:
- `bfs(grid, start, end)` → { path, explored, frontierHistory }
- `dfs(grid, start, end)` → { path, explored, frontierHistory }
- `ucs(grid, start, end)` → { path, explored, frontierHistory }
- `aStar(grid, start, end, heuristic)` → { path, explored, frontierHistory }
- `greedy(grid, start, end, heuristic)` → { path, explored, frontierHistory }
- `PriorityQueue` class (min-heap)
- `manhattanDistance(a, b)`, `euclideanDistance(a, b)`

#### [NEW] [styles.css](file:///d:/hk2nam2/ai/ai-final_project/web/styles.css)

Premium dark theme design:
- Glassmorphism cards
- Gradient accents
- Micro-animations
- Responsive layout

---

### Component 2: Firmware ESP32

#### [MODIFY] [main.ino](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/main.ino)

Thay đổi chính:
- Thêm `#include <ArduinoWebsockets.h>` hoặc dùng `AsyncWebSocket` từ ESPAsyncWebServer
- Thêm WebSocket handler: `onWsEvent()` xử lý message JSON
- Thêm mode `MODE_AI_ROUTE` bên cạnh `MODE_MANUAL` và `MODE_LINE`
- Trong [loop()](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/main.ino#296-309): khi `MODE_AI_ROUTE` → gọi `route_loop()` từ route interpreter
- Thêm telemetry timer: mỗi 200ms gửi sensor data qua WebSocket

#### [NEW] [route_interpreter.h](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/route_interpreter.h)

```cpp
#pragma once
void route_setup();
void route_loop();           // State machine tick
void route_load(const char* json);  // Parse JSON route
void route_abort();
bool route_is_done();
```

#### [NEW] [route_interpreter.cpp](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/route_interpreter.cpp)

State Machine:
```
STATE_IDLE → (nhận route) → STATE_FOLLOWING_LINE
STATE_FOLLOWING_LINE:
  - PID line follow (reuse do_line logic)
  - Kiểm tra intersection (5 mắt ON)
  - Kiểm tra obstacle (HC-SR04)
  ├→ intersection → STATE_AT_INTERSECTION
  └→ obstacle → STATE_OBSTACLE

STATE_AT_INTERSECTION:
  - Tiến thêm ~2cm (encoder)
  - Dequeue lệnh:
    F → continue → STATE_FOLLOWING_LINE
    L → spin_left_deg(90) → STATE_FOLLOWING_LINE
    R → spin_right_deg(90) → STATE_FOLLOWING_LINE
    END → STATE_DONE

STATE_OBSTACLE:
  - Dừng, xác định vị trí grid
  - Gửi OBSTACLE_DETECTED qua WebSocket
  - Chờ NEW_ROUTE (timeout 10s → fallback)
  ├→ nhận route mới → STATE_REROUTING
  └→ timeout → dừng hẳn

STATE_REROUTING:
  - Load route mới, reset queue
  → STATE_FOLLOWING_LINE

STATE_DONE:
  - Dừng motor, gửi COMPLETED
  → STATE_IDLE
```

Tái sử dụng từ [do_line.cpp](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp):
- PID controller (pidL, pidR)
- [spin_left_deg()](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#293-324), [spin_right_deg()](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#325-359)
- [move_forward_distance()](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp#360-377)
- Encoder ISR
- HC-SR04 reading

#### [MODIFY] [do_line.cpp](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.cpp)

- Extract shared functions để có thể gọi từ route_interpreter
- Thêm hàm `bool isIntersection()` → return true khi 5 mắt đều ON
- Thêm hàm `float getLineError()` → return error [-4, +4] cho PID

#### [MODIFY] [do_line.h](file:///d:/hk2nam2/ai/ai-final_project/ai-firmware/main/do_line.h)

- Export thêm: `isIntersection()`, `getLineError()`, encoder vars, motor functions

---

## User Review Required

> [!IMPORTANT]
> **Cấu trúc thư mục mới:**
> ```
> ai-final_project/
> ├── ai-firmware/main/
> │   ├── main.ino          (sửa: thêm WebSocket + MODE_AI_ROUTE)
> │   ├── do_line.cpp       (sửa: extract shared functions)
> │   ├── do_line.h         (sửa: export thêm functions)
> │   ├── route_interpreter.cpp  (MỚI)
> │   └── route_interpreter.h    (MỚI)
> └── web/
>     ├── index.html        (MỚI: full web interface)
>     ├── algorithms.js     (MỚI: 5 thuật toán AI)
>     └── styles.css        (MỚI: premium dark theme)
> ```

> [!WARNING]
> **WebSocket library**: Plan sử dụng `AsyncWebSocket` tích hợp trong `ESPAsyncWebServer` (đã có sẵn). Không cần thêm library mới.

> [!IMPORTANT]
> **Thứ tự triển khai đề xuất:**
> 1. Web Interface (AI algorithms + Map Editor) → test trên browser trước
> 2. Firmware WebSocket + Route Interpreter → test giao tiếp
> 3. Tích hợp + Dynamic Re-routing

---

## Verification Plan

### Browser Testing (Web Interface)
1. Mở `web/index.html` trong browser
2. Tạo grid 5×5, đặt walls, start, end
3. Chạy từng thuật toán → verify animation + path đúng
4. "Compare All" → verify bảng so sánh hiển thị đúng
5. Test save/load bản đồ JSON
6. Test route conversion (path → F/L/R commands)

### Manual Verification (Firmware)
1. Upload firmware lên ESP32
2. Kết nối WiFi "ESP32-Car"
3. Mở web interface → kết nối WebSocket
4. Gửi route test → verify robot nhận và phản hồi
5. Test intersection detection trên track thực tế
6. Test obstacle detection + re-routing flow

> [!NOTE]
> Do đây là firmware ESP32, không thể viết unit test tự động chạy trên PC. Verification chủ yếu qua browser testing cho web và manual testing cho firmware.
