# 📋 PHÂN CÔNG NHÓM — Đồ án AI Robot Giao hàng Tự hành

> **Nhóm 4 người** | Timeline: **2 tuần** (24/03 → 07/04/2026)
> **Phần cứng**: ✅ Đã hoàn thành (robot lắp ráp xong, track có sẵn)
> **Báo cáo**: ⏸ Tạm gác lại, tập trung toàn lực vào code & test hệ thống

---

## 📊 Trạng thái hiện tại

| Module | Trạng thái |
|--------|-----------|
| ✅ Phần cứng robot (motor, sensor, encoder, nguồn) | Hoàn thành |
| ✅ Firmware cơ bản (manual control + PID line follow) | Hoàn thành |
| ✅ Code firmware route interpreter + WebSocket | Code xong, **chưa test** |
| ✅ Code web AI algorithms + Dashboard | Code xong, **chưa test** |
| ⬜ Test & debug firmware trên robot thật | Chưa làm |
| ⬜ Test & debug web interface | Chưa làm |
| ⬜ Tích hợp end-to-end (Web ↔ ESP32 ↔ Robot) | Chưa làm |
| ⬜ Track lưới line trên sàn | Cần dán/kiểm tra |

---

## 👥 PHÂN CÔNG TẬP TRUNG TÍCH HỢP — 2 TUẦN

---

### 🔵 TV1 — Web Dashboard & Giao tiếp (Web Side)

> **Tập trung**: Giao diện UI/UX chức năng, Animation, Giao tiếp WebSocket

#### Tuần 1 (24/03 – 30/03)
- [ ] Render grid Canvas: scale chuẩn, hover effect, animation tốc độ mượt (chỉnh slider)
- [ ] Hoàn thiện các tool vẽ bản đồ (Walls, Start, End, Weights, Random Maze)
- [ ] Export/Import file JSON bản đồ hoạt động trơn tru
- [ ] Viết hàm parse đường đi từ TV4 sang chuỗi lệnh `F`, `L`, `R` (Route Sender)

#### Tuần 2 (31/03 – 06/04)
- [ ] Kết nối WebSocket với ESP32: bắt bắt sự kiện `CONNECT`, `DISCONNECT`, `MESSAGE`
- [ ] Handle sự kiện từ robot (Telemetry): update UI tốc độ, khoảnh cách cảm biến real-time
- [ ] Giao tiếp luồng Dynamic Re-routing: Nhận `OBSTACLE_DETECTED` → Vẽ vật cản → Gọi TV4 tính đường đi mới → Gửi ngược `ROUTE` mới cho ESP32.

**Files**: `web/index.html`, `web/styles.css`, code WebSocket ở `web/algorithms.js`

---

### 🟢 TV2 — Firmware ESP32 & Giao tiếp (Robot Side)

> **Tập trung**: State Machine, Interpreter, WebSocket Server trên ESP32

#### Tuần 1 (24/03 – 30/03)
- [ ] Tích hợp lib WebSockets vào `main.ino`/`route_interpreter.cpp`
- [ ] Viết hàm parse JSON route gửi từ Web
- [ ] Hoàn thiện `Route Interpreter`: IDLE → FOLLOW → INTERSECTION → OBSTACLE → DONE
- [ ] Test đọc lệnh giả lập từ queue để đảm bảo logic State Machine chạy đúng

#### Tuần 2 (31/03 – 06/04)
- [ ] Bắt WebSocket message: start route, stop/abort route.
- [ ] Logic phát hiện vật cản (HC-SR04): Dừng khẩn cấp → Tính toạ độ vật cản dựa trên grid pos hiện tại → Ném JSON thông báo `OBSTACLE` qua WebSocket.
- [ ] Code payload Telemetry: Gộp data tốc độ (EMA encoder), sensor line, sonar gửi qua Web mỗi 200ms.

**Files**: `ai-firmware/main/main.ino`, `route_interpreter.cpp/h`

---

### 🟠 TV3 — Robot Control (PID & Sensors) + Track

> **Tập trung**: Điều khiển Low-level, Dò line, Quay góc chuẩn

#### Tuần 1 (24/03 – 30/03)
- [ ] Dán track lưới ô vuông trên sàn, dán các nút giao (chữ thập).
- [ ] Tune PID bám line (`Kp`, `Ki`, `Kd` trong `do_line.cpp`) sao cho robot chạy thẳng mượt, không lắc đầu.
- [ ] Tune hàm nhận diện giao lộ (5 cảm biến cùng đen). Đảm bảo không miss và không nhận sai.

#### Tuần 2 (31/03 – 06/04)
- [ ] Tune hàm `spin_left_deg()` và `spin_right_deg` (dùng Encoder): Góc quay phải đúng sát 90 độ, dừng dứt khoát tại line mới.
- [ ] Đo khoảng cách tiến vào tâm giao lộ (`move_forward_distance`) sao cho khi quay robot vẫn nằm trên tâm vạch.
- [ ] Hỗ trợ TV2 test phần chạy cứng (nhận lệnh L, R từ queue của ngã tư).

**Deliverables**: Robot bám line mượt, quẹo góc chuẩn xác tại giao lộ thực tế.

---

### 🟣 TV4 — AI Algorithms Engine 

> **Tập trung**: Cốt lõi Engine AI (Javascript)

#### Tuần 1 (24/03 – 30/03)
- [ ] Implement cấu trúc dữ liệu PriorityQueue cho Javascript tối ưu
- [ ] Code thuật toán duyệt: **BFS**, **DFS** 
- [ ] Code thuật toán có chi phí: **UCS**
- [ ] Trả về đúng format Object để TV1 gọi làm Animation `({ path, exploredHistory, frontierHistory, stats })`

#### Tuần 2 (31/03 – 06/04)
- [ ] Code thuật toán Heuristic: **A* Search**, **Greedy Best-First**
- [ ] Viết hàm tính Heuristic (Manhattan, Euclidean)
- [ ] Phối hợp TV1: Xử lí "Dynamic Re-routing" - Viết logic tiếp nhận toạ độ bắt đầu mới (vị trí robot hiện tại) và toạ độ vật cản để tìm đường đi nhánh mới nhanh nhất, báo lỗi nếu kẹt.
- [ ] Debug kịch bản MAP phức tạp để đảm bảo 5 thuật toán hoạt động không bị vòng lặp vô hạn hay mem leak trên browser.

**Files**: `web/algorithms.js`

---

## 📅 TIMELINE TÍCH HỢP TỔNG (TUẦN 2)

```
                            Tuần 2 (31/03-06/04)
         Lu Ma Tu Th Fr Sa Su              Lu Ma Tu Th Fr Sa Su

TV1 WEB  [UI Grid & Map Edit ------]      [WebSocket & Telemetry Flow -----]
TV2 FW   [State Machine Queue -----]      [WebSocket Server & Obstacle Json]
TV3 HW   [PID Tune & Track Setup --]      [Spin Encoder 90 độ & Intersection]
TV4 AI   [BFS/DFS/UCS Engine ------]      [A*/Greedy & Dynamic Rerouting --]

                                           ⚡ Deadline Tích hợp (Chạy thực nghiệm)
```

---

## 🔑 MILESTONES TÍCH HỢP

| Ngày | Milestone | Phối hợp |
|------|-----------|----------|
| **28/03 (Thứ 6)** | TV4 bàn giao core AI 5 hàm cho TV1 cắm vào giao diện render Animation | TV1 + TV4 |
| **30/03 (CN)** | TV3 chốt thông số PID + Hàm quay `spin_deg()`, chạy line vật lý tốt | TV2 + TV3 |
| **02/04 (Thứ 4)** | TV1 gửi chuỗi JSON Route (dùng AI TV4) dội sang TV2. TV2 nhận đổi state → chạy. | TV1 + TV2 + TV4 |
| **05/04 (Thứ 7)** | Test Dynamic Re-Rout: TV3 đặt vật cản → TV2 báo về → TV4 tìm mới → TV1 render & đổi → Robot chạy tiếp tục | Toàn Nhóm |
