# ĐỀ CƯƠNG ĐỒ ÁN MÔN HỌC
## Môn: Trí tuệ Nhân tạo (Artificial Intelligence)

---

## 1. Tên đề tài

**"Xây dựng Hệ thống Xe Robot Giao hàng Tự hành Dò Line Tránh Vật cản Ứng dụng Thuật toán Tìm kiếm AI (Informed & Uninformed Search) với Giao diện Web Mô phỏng và Điều khiển"**

---

## 2. Giới thiệu

### 2.1. Lý do chọn đề tài

Trong lĩnh vực Trí tuệ Nhân tạo, **thuật toán tìm kiếm (Search Algorithms)** là nền tảng cốt lõi cho hầu hết các bài toán lập kế hoạch (planning) và ra quyết định (decision-making). Từ điều hướng xe tự hành, robot trong kho Amazon, đến hệ thống GPS — tất cả đều dựa trên các thuật toán tìm đường kinh điển như BFS, DFS, UCS, A*, và Greedy Best-First Search.

Tuy nhiên, trong môi trường học thuật, sinh viên thường chỉ tiếp cận các thuật toán này trên lý thuyết hoặc qua các bài tập mô phỏng thuần túy trên máy tính. Việc **kết nối trực tiếp kết quả thuật toán AI với một robot vật lý thực tế** mang lại những giá trị vượt trội:

- **Trực quan hóa thuật toán**: Sinh viên "nhìn thấy" sự khác biệt giữa BFS (duyệt theo lớp), DFS (đi sâu trước), A* (có heuristic) qua chính hành vi thực tế của robot.
- **Cầu nối lý thuyết – thực hành**: Hiểu rằng một đường đi "tối ưu trên giấy" phải đối mặt với nhiều thách thức thực tế: sai số cảm biến, trượt bánh, vật cản bất ngờ.
- **Kỹ năng tích hợp hệ thống**: Phát triển đồng thời năng lực lập trình AI, phát triển web, và lập trình nhúng (embedded systems) — bộ kỹ năng đa ngành rất được ngành công nghiệp đánh giá cao.
- **Ứng dụng thực tiễn**: Mô hình thu nhỏ của hệ thống logistics tự động (AGV) trong kho bãi, nhà máy, bệnh viện.

### 2.2. Tính cấp thiết

| Yếu tố | Mô tả |
|---|---|
| **Thị trường AGV** | Dự kiến đạt 18 tỷ USD vào 2027 (CAGR ~14%), nhu cầu kỹ sư AI + Robotics tăng cao. |
| **Giáo dục STEM** | Xu hướng "học qua dự án" (Project-Based Learning); đồ án liên ngành AI–IoT–Robot là mô hình đào tạo hiệu quả. |
| **Nghiên cứu kế thừa** | Cơ sở để mở rộng sang các thuật toán nâng cao: Reinforcement Learning, SLAM, Multi-agent pathfinding. |

---

## 3. Mục tiêu đồ án

### 3.1. Mục tiêu tổng quát

Thiết kế, chế tạo và vận hành thành công một hệ thống gồm: **(1)** giao diện web mô phỏng bản đồ 2D với các thuật toán tìm kiếm AI, và **(2)** xe robot 4 bánh thực thi lộ trình do thuật toán AI tính toán bằng cách dò line trên mặt sàn và tránh vật cản thực tế.

### 3.2. Mục tiêu cụ thể

| STT | Mục tiêu | Chỉ tiêu đo lường |
|-----|----------|-------------------|
| 1 | **Phát triển phần mềm mô phỏng thuật toán AI** | Triển khai thành công ≥ 5 thuật toán: BFS, DFS, UCS, A*, Greedy Best-First Search trên giao diện web. |
| 2 | **Trực quan hóa quá trình tìm kiếm** | Web hiển thị animation từng bước "khám phá" của thuật toán: các node đã duyệt (explored), hàng đợi/frontier, đường đi tối ưu tìm được. |
| 3 | **So sánh hiệu năng thuật toán** | Giao diện hiển thị bảng so sánh: số node mở rộng, chi phí đường đi, thời gian thực thi, bộ nhớ sử dụng cho mỗi thuật toán trên cùng một bản đồ. |
| 4 | **Giao tiếp Web ↔ ESP32** | Truyền thành công lộ trình (chuỗi lệnh/điểm) từ web đến ESP32 qua Wi-Fi (HTTP/WebSocket) với độ tin cậy ≥ 99%. |
| 5 | **Robot di chuyển đúng lộ trình** | Robot thực hiện đúng ≥ 90% các điểm rẽ trên track thực tế tương ứng với lộ trình AI đã tính. |
| 6 | **Dò line ổn định** | Sai lệch tâm ≤ ±1.5 cm, vượt cua bán kính ≥ 15 cm mà không mất line. |
| 7 | **Tránh vật cản thực tế** | Phát hiện vật cản tĩnh ở khoảng cách 10–30 cm, xử lý (dừng + thông báo hoặc vòng tránh) thành công ≥ 95%. |
| 8 | **Đo quãng đường bằng Encoder** | Sai số ≤ 5% trên quãng đường thử nghiệm 2 m. |

---

## 4. Phạm vi đồ án

### 4.1. Trong phạm vi (In-Scope)

| Hạng mục | Mô tả |
|---|---|
| **Bản đồ** | Bản đồ 2D dạng lưới ô vuông (grid map) được tạo/chỉnh sửa trên giao diện web. Hỗ trợ đặt vật cản, điểm bắt đầu, điểm kết thúc, và các điểm giao (waypoint). |
| **Thuật toán AI** | Uninformed: BFS, DFS, UCS. Informed: A* Search, Greedy Best-First Search. Heuristic: Manhattan Distance và/hoặc Euclidean Distance. |
| **Mô phỏng** | Trực quan hóa từng bước hoạt động của thuật toán trên giao diện web (animation). |
| **Robot thực tế** | Xe 4 bánh dò line (vạch đen trên nền trắng), tránh vật cản tĩnh, trên mặt sàn phẳng trong nhà. |
| **Giao tiếp** | Wi-Fi (ESP32 AP/STA mode), HTTP hoặc WebSocket, phạm vi mạng LAN. |
| **Track thực tế** | Lưới đường (line grid) trên sàn tương ứng 1-1 với bản đồ 2D trên web. Giao lộ đánh dấu bằng pattern đặc biệt (ví dụ: chữ thập). |

### 4.2. Ngoài phạm vi (Out-of-Scope)

- Thuật toán học máy (Machine Learning), học tăng cường (Reinforcement Learning).
- Bản đồ liên tục (continuous space); chỉ dùng bản đồ rời rạc (discrete grid).
- Vật cản động (đang di chuyển).
- Hoạt động ngoài trời (outdoor).
- Nhận dạng hình ảnh / Computer Vision.
- Quy hoạch đường đi cho nhiều robot đồng thời (multi-agent).
- Truyền dữ liệu qua cloud; chỉ giao tiếp cục bộ trong mạng LAN.
- Cơ cấu nâng/hạ hàng hóa vật lý.

---

## 5. Công nghệ sử dụng

### 5.1. Phần cứng

| Thành phần | Linh kiện | Ghi chú |
|---|---|---|
| Vi điều khiển | **ESP32-WROOM-32** + PCB hỗ trợ | Dual-core 240 MHz, Wi-Fi 802.11 b/g/n |
| Động cơ | 4× DC Geared Motor (3–6V) | Giảm tốc ~1:48 |
| Bánh xe | 4× bánh cao su | Đường kính ~65 mm |
| Driver động cơ | **L298N** Dual H-Bridge | 2 kênh, mỗi kênh điều khiển 2 motor cùng phía |
| Cảm biến dò line | **5× TCRT5000** (hồng ngoại) | **Đề xuất thêm** — 5 mắt cho PID mịn (xem mục 5.3) |
| Cảm biến siêu âm | **HC-SR04** | Tầm đo 2–400 cm |
| Servo | **SG90** (0°–180°) | Xoay HC-SR04 quét trái/phải |
| Encoder | 2× Encoder quang (trên 2 bánh) | Đếm xung đo vận tốc và quãng đường |
| Nguồn | 2× Đế pin 18650 (3S) = 6 cell | ~11.1V mỗi pack |
| Hạ áp | 2× **LM2596** DC-DC Buck | Module 1: 5V (ESP32 + cảm biến), Module 2: ~7.4V (L298N + motor) |
| Công tắc | 2× công tắc gạt | Tách nguồn logic / nguồn động lực |
| Khung | Tấm Mica (2 tầng) | Tầng dưới: motor + pin; Tầng trên: ESP32 + cảm biến |
| Dây nối | Bộ dây Dupont + dây điện | Jump wire Male-Female, Male-Male |

### 5.2. Phần mềm & Ngôn ngữ lập trình

| Công nghệ | Mục đích | Chi tiết |
|---|---|---|
| **C/C++ (Arduino Framework)** | Firmware ESP32 | Arduino IDE 2.x hoặc PlatformIO; thư viện: `WiFi.h`, `WebServer.h`, `ESP32Servo.h`, `ArduinoJson.h` |
| **HTML5 / CSS3 / JavaScript (Vanilla)** | Giao diện Web | Tạo bản đồ grid, animation thuật toán, dashboard điều khiển. Sử dụng **Canvas API** hoặc **SVG** để render bản đồ. |
| **JavaScript (ES6+)** | Thuật toán AI | Triển khai BFS, DFS, UCS, A*, Greedy ngay trên trình duyệt (client-side). |
| **WebSocket** | Giao tiếp real-time | Truyền lộ trình từ web → ESP32 và nhận telemetry từ ESP32 → web. |
| **SPIFFS / LittleFS** | File system ESP32 | Lưu trữ file HTML/CSS/JS của web dashboard bên trong flash ESP32. |

### 5.3. Đề xuất thêm: Cảm biến dò line — 5 mắt TCRT5000

**Lý do cần bổ sung cảm biến line (không có trong danh sách ban đầu):**

Robot cần dò line trên mặt sàn thực → bắt buộc phải có cảm biến hồng ngoại phản xạ. Đề xuất **5 mắt TCRT5000** vì:

```
Bố trí (nhìn từ dưới, phía trước robot):

     [S1]   [S2]   [S3]   [S4]   [S5]
    Ngoài  Trong  Tâm   Trong  Ngoài
    trái   trái         phải   phải
```

| So sánh | 3 mắt | 5 mắt | 7+ mắt |
|---------|-------|-------|--------|
| Số trạng thái lệch | 3 | ≥ 9 | ≥ 15 |
| Chất lượng PID | Thô, giật | **Mịn, mượt** | Rất mịn |
| Xử lý cua gấp | Kém | **Tốt** | Rất tốt |
| GPIO ESP32 | 3 | 5 (ESP32 dư sức) | 7+ |
| Chi phí thêm | — | ~15.000 VNĐ | ~25.000+ |

**Bảng mã hóa error PID (5 mắt):**

| S1 | S2 | S3 | S4 | S5 | Trạng thái | Error |
|----|----|----|----|----|-----------|-------|
| 0 | 0 | 1 | 0 | 0 | Đúng tâm | 0 |
| 0 | 1 | 1 | 0 | 0 | Lệch nhẹ trái | +1 |
| 0 | 1 | 0 | 0 | 0 | Lệch trái | +2 |
| 1 | 1 | 0 | 0 | 0 | Lệch mạnh trái | +3 |
| 1 | 0 | 0 | 0 | 0 | Mất line trái | +4 |
| 0 | 0 | 1 | 1 | 0 | Lệch nhẹ phải | −1 |
| 0 | 0 | 0 | 1 | 0 | Lệch phải | −2 |
| 0 | 0 | 0 | 1 | 1 | Lệch mạnh phải | −3 |
| 0 | 0 | 0 | 0 | 1 | Mất line phải | −4 |
| 1 | 1 | 1 | 1 | 1 | **Giao lộ (intersection)** | Xử lý riêng |
| 0 | 0 | 0 | 0 | 0 | Mất line hoàn toàn | Giữ error cuối |

> **`1` = cảm biến phát hiện vạch đen (line).** Trạng thái "Giao lộ" (5 mắt đều ON) rất quan trọng: đó là lúc robot cần quyết định rẽ trái/phải/đi thẳng theo lộ trình AI.

### 5.4. Thư viện & Công cụ tham khảo

| Thư viện / Công cụ | Ngôn ngữ | Mục đích |
|---|---|---|
| `ArduinoJson` | C++ | Serialize/Deserialize dữ liệu JSON trên ESP32 |
| `ESPAsyncWebServer` | C++ | Web Server không đồng bộ, hỗ trợ WebSocket |
| `ESP32Servo` | C++ | Điều khiển Servo SG90 |
| Canvas API / `requestAnimationFrame` | JavaScript | Render bản đồ grid + animation thuật toán |
| `PriorityQueue` (tự triển khai) | JavaScript | Hàng đợi ưu tiên cho UCS, A*, Greedy |

---

## 6. Kiến trúc hệ thống

### 6.1. Sơ đồ kiến trúc tổng thể

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                      KIẾN TRÚC HỆ THỐNG TỔNG THỂ                          │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌───────────────────────────────────────┐                                  │
│  │      TẦNG GIAO DIỆN & AI (WEB)       │                                  │
│  │                                       │                                  │
│  │  ┌─────────┐  ┌──────────────────┐   │                                  │
│  │  │ Map     │  │ AI Search Engine │   │                                  │
│  │  │ Editor  │  │                  │   │                                  │
│  │  │         │  │ • BFS    • A*    │   │                                  │
│  │  │ • Grid  │  │ • DFS    • Greedy│   │                                  │
│  │  │ • Start │  │ • UCS           │   │                                  │
│  │  │ • End   │  │                  │   │                                  │
│  │  │ • Walls │  │ Animation Engine │   │                                  │
│  │  └─────────┘  └──────────────────┘   │                                  │
│  │                                       │                                  │
│  │  ┌──────────────────────────────────┐│                                  │
│  │  │  Dashboard: Tốc độ, Quãng đường, ││                                  │
│  │  │  Trạng thái cảm biến, So sánh   ││                                  │
│  │  └──────────────────────────────────┘│                                  │
│  └──────────────────┬────────────────────┘                                  │
│                     │  WebSocket / HTTP                                      │
│                     │  (Wi-Fi LAN)                                           │
│                     ▼                                                        │
│  ┌───────────────────────────────────────┐                                  │
│  │    TẦNG XỬ LÝ TRUNG TÂM (ESP32)     │                                  │
│  │                                       │                                  │
│  │  ┌─────────────┐  ┌────────────────┐ │                                  │
│  │  │ Route       │  │ Motion Control │ │                                  │
│  │  │ Interpreter │  │                │ │                                  │
│  │  │             │  │ • PID Line     │ │                                  │
│  │  │ • Parse     │  │   Following    │ │                                  │
│  │  │   path      │  │ • Obstacle     │ │                                  │
│  │  │ • Queue     │  │   Handling     │ │                                  │
│  │  │   commands  │  │ • Encoder      │ │                                  │
│  │  └─────────────┘  │   Odometry     │ │                                  │
│  │                    └────────────────┘ │                                  │
│  └──────────────────┬────────────────────┘                                  │
│                     │  GPIO / PWM / Interrupt                                │
│                     ▼                                                        │
│  ┌───────────────────────────────────────┐     ┌─────────────────────────┐  │
│  │      TẦNG CHẤP HÀNH (Actuators)      │     │  TẦNG CẢM BIẾN         │  │
│  │                                       │     │                         │  │
│  │  • 4× DC Motor (qua L298N)           │     │  • 5× TCRT5000 (line)  │  │
│  │  • Servo SG90 (quét HC-SR04)         │     │  • HC-SR04 (khoảng cách)│  │
│  │                                       │     │  • 2× Encoder (xung)   │  │
│  └───────────────────────────────────────┘     └─────────────────────────┘  │
│                                                                             │
│  ┌───────────────────────────────────────┐                                  │
│  │           TẦNG NGUỒN ĐIỆN            │                                  │
│  │                                       │                                  │
│  │  Pack A (3S 18650) ──▶ LM2596 → 5V  ──▶ ESP32 + Cảm biến + Servo    │  │
│  │  Pack B (3S 18650) ──▶ LM2596 → 7.4V ──▶ L298N + 4× Motor           │  │
│  │  [Switch A]              [Switch B]    (tách nguồn logic/động lực)    │  │
│  └───────────────────────────────────────┘                                  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 6.2. Luồng hoạt động chính (Data Flow)

```
┌──────────────┐     ┌──────────────────┐     ┌──────────────────────┐
│  NGƯỜI DÙNG  │     │   WEB BROWSER    │     │       ESP32          │
│              │     │                  │     │                      │
│ 1. Vẽ bản đồ│────▶│ 2. Lưu grid map │     │                      │
│    trên web  │     │    (NxM matrix)  │     │                      │
│              │     │                  │     │                      │
│ 3. Chọn thuật│────▶│ 4. Chạy thuật   │     │                      │
│    toán AI   │     │    toán tìm kiếm │     │                      │
│              │     │    + Animation   │     │                      │
│              │     │                  │     │                      │
│ 5. Nhấn "Gửi│────▶│ 6. Convert path  │     │                      │
│    cho Robot"│     │    → chuỗi lệnh: │     │                      │
│              │     │    ["F","F","R",  │────▶│ 7. Parse lệnh       │
│              │     │     "F","L","F"]  │ WS  │ 8. Thực thi tuần tự:│
│              │     │                  │     │    - Dò line tiến     │
│              │     │                  │     │    - Khi gặp giao lộ  │
│              │     │                  │     │      → rẽ theo lệnh  │
│              │     │                  │     │    - HC-SR04 phát hiện│
│              │     │ 10. Hiển thị     │◀────│      vật cản → dừng  │
│              │     │     telemetry    │  WS │ 9. Gửi trạng thái   │
│              │     │     real-time    │     │    về web             │
└──────────────┘     └──────────────────┘     └──────────────────────┘

Chú thích: WS = WebSocket
           F = Forward, R = Right, L = Left
```

### 6.3. Ánh xạ bản đồ ảo ↔ Track thực tế

Đây là điểm then chốt của hệ thống — sự tương ứng giữa bản đồ grid trên web và lưới line trên sàn:

```
  BẢN ĐỒ WEB (Grid 5×5)              TRACK THỰC TẾ (Sàn nhà)
  
  ┌───┬───┬───┬───┬───┐              ●───────●───────●───────●───────●
  │ S │   │   │   │   │              │       │       │       │       │
  ├───┼───┼───┼───┼───┤              │       │       │       │       │
  │   │ █ │ █ │   │   │              ●───────●░░░░░░░●───────●───────●
  ├───┼───┼───┼───┼───┤                      (vật cản thực)
  │   │   │   │   │   │              ●───────●───────●───────●───────●
  ├───┼───┼───┼───┼───┤              │       │       │       │       │
  │   │   │ █ │   │   │              ●───────●───────●░░░░░░░●───────●
  ├───┼───┼───┼───┼───┤              │       │       │       │       │
  │   │   │   │   │ E │              ●───────●───────●───────●───────●
  └───┘───┘───┘───┘───┘              
  
  S = Start, E = End                  ● = Giao lộ (intersection)
  █ = Vật cản (wall)                  ─ = Line trên sàn
                                      ░ = Vật cản vật lý
  
  Mỗi ô grid ↔ 1 đoạn line giữa 2 giao lộ (~30cm)
```

---

## 7. Chức năng chi tiết

### 7.1. Chức năng phía Web (AI & Giao diện)

#### 7.1.1. Editor bản đồ (Map Editor)

| Chức năng | Mô tả |
|---|---|
| Tạo grid | Người dùng chọn kích thước bản đồ (ví dụ: 5×5, 8×8, 10×10). |
| Đặt/xóa vật cản | Click vào ô để toggle wall (vật cản) / free cell. |
| Đặt điểm Start | Click chọn ô xuất phát (hiển thị icon 🟢). |
| Đặt điểm End | Click chọn ô đích (hiển thị icon 🔴). |
| Đặt waypoint | (Tùy chọn) Đặt các điểm giao trung gian mà robot phải đi qua. |
| Đặt trọng số | Cho phép gán chi phí khác nhau cho từng ô (phục vụ UCS và A*). |
| Lưu/tải bản đồ | Export/Import bản đồ dạng JSON để tái sử dụng. |

#### 7.1.2. AI Search Engine — Thuật toán tìm kiếm

| Thuật toán | Loại | Đặc điểm | Cấu trúc dữ liệu |
|---|---|---|---|
| **BFS** (Breadth-First Search) | Uninformed | Tìm đường ngắn nhất (số bước), duyệt theo lớp | Queue (FIFO) |
| **DFS** (Depth-First Search) | Uninformed | Đi sâu trước, không đảm bảo tối ưu, tốn ít bộ nhớ | Stack (LIFO) |
| **UCS** (Uniform-Cost Search) | Uninformed | Mở rộng node có chi phí nhỏ nhất, tối ưu khi có trọng số khác nhau | Priority Queue |
| **A\*** Search | Informed | f(n) = g(n) + h(n); tối ưu nếu h(n) admissible | Priority Queue |
| **Greedy Best-First** | Informed | f(n) = h(n); nhanh nhưng không đảm bảo tối ưu | Priority Queue |

**Heuristic function h(n)** cho A* và Greedy:

```
Manhattan Distance:  h(n) = |x_current - x_goal| + |y_current - y_goal|
Euclidean Distance:  h(n) = sqrt((x_current - x_goal)² + (y_current - y_goal)²)
```

> Người dùng có thể chọn loại heuristic trên giao diện để so sánh hiệu quả.

#### 7.1.3. Animation Engine — Trực quan hóa

Giao diện web hiển thị **animation từng bước** của thuật toán:

```
Bảng màu hiển thị:

  ⬜ = Ô trống (chưa duyệt)
  🟦 = Đang trong Frontier (hàng đợi/stack)
  🟨 = Đã Explored (đã mở rộng)
  🟩 = Đường đi tìm được (final path)
  ⬛ = Vật cản (wall)
  🟢 = Start
  🔴 = End
```

- Tốc độ animation có thể điều chỉnh (slider: slow → fast → instant).
- Hiển thị **bảng thống kê real-time**: Nodes explored, Frontier size, Path cost, Elapsed time.
- Nút **"So sánh tất cả"**: Chạy tất cả 5 thuật toán trên cùng bản đồ và hiển thị bảng so sánh:

| Thuật toán | Nodes explored | Path length | Path cost | Time (ms) | Tối ưu? |
|---|---|---|---|---|---|
| BFS | 24 | 8 | 8 | 3 | ✅ (đều bước) |
| DFS | 15 | 12 | 12 | 2 | ❌ |
| UCS | 20 | 8 | 8 | 4 | ✅ |
| A* | 12 | 8 | 8 | 2 | ✅ |
| Greedy | 8 | 10 | 10 | 1 | ❌ |

#### 7.1.4. Route Sender — Gửi lộ trình cho Robot

Sau khi thuật toán tìm được đường đi, hệ thống chuyển đổi lộ trình thành **chuỗi lệnh di chuyển**:

```
Path trên grid:  (0,0) → (1,0) → (2,0) → (2,1) → (2,2) → (3,2) → (4,2)
                   ↓        ↓        ↓        ↓        ↓        ↓
Commands:        START →  "F"   →  "F"   →  "R"  →  "F"  →  "L"  →  "F" → END

Trong đó:
  F = Forward (đi thẳng đến giao lộ tiếp theo)
  R = Turn Right (rẽ phải tại giao lộ)
  L = Turn Left  (rẽ trái tại giao lộ)
```

Dữ liệu gửi qua WebSocket dạng JSON:

```json
{
  "type": "ROUTE",
  "commands": ["F", "F", "R", "F", "L", "F"],
  "total_steps": 6,
  "algorithm": "A*",
  "path_cost": 6
}
```

#### 7.1.5. Telemetry Dashboard — Giám sát Robot

| Dữ liệu | Nguồn | Cập nhật |
|---|---|---|
| Tốc độ bánh trái / phải | Encoder | Real-time (200 ms) |
| Quãng đường đã đi | Encoder | Real-time |
| Bước hiện tại / tổng bước | Route Interpreter | Mỗi giao lộ |
| Trạng thái cảm biến line | 5× TCRT5000 | Real-time |
| Khoảng cách vật cản trước | HC-SR04 | Real-time |
| Chế độ: LINE_FOLLOW / TURNING / OBSTACLE / DONE | State Machine | Real-time |

### 7.2. Chức năng phía ESP32 (Firmware Robot)

#### 7.2.1. Route Interpreter — Bộ thông dịch lộ trình

```
Nhận JSON lệnh từ Web
        │
        ▼
Parse mảng commands[] → lưu vào hàng đợi (queue)
        │
        ▼
┌───────────────────────────────────────────────────┐
│  State Machine (Máy trạng thái chính)             │
│                                                     │
│  STATE_IDLE ──(nhận route)──▶ STATE_FOLLOWING_LINE │
│                                                     │
│  STATE_FOLLOWING_LINE:                              │
│    • Chạy PID dò line                              │
│    • Đọc encoder tính quãng đường                  │
│    • Liên tục kiểm tra HC-SR04                     │
│    │                                                │
│    ├─(5 mắt ON = giao lộ)──▶ STATE_AT_INTERSECTION│
│    │                                                │
│    └─(HC-SR04 < 15cm)──────▶ STATE_OBSTACLE        │
│                                                     │
│  STATE_AT_INTERSECTION:                             │
│    • Dequeue lệnh tiếp theo                        │
│    │                                                │
│    ├─ "F" → Đi thẳng qua giao lộ                  │
│    ├─ "L" → Xoay trái 90° (dùng encoder/gyro)     │
│    ├─ "R" → Xoay phải 90°                          │
│    └─ END → STATE_DONE                              │
│    │                                                │
│    └──────────────────────▶ STATE_FOLLOWING_LINE    │
│                                                     │
│  STATE_OBSTACLE:                                    │
│    • Dừng robot                                     │
│    • Servo quét trái/phải                           │
│    • Gửi cảnh báo về Web                           │
│    • Chờ vật cản được dọn HOẶC tự vòng tránh      │
│    └──────────────────────▶ STATE_FOLLOWING_LINE    │
│                                                     │
│  STATE_DONE:                                        │
│    • Dừng motor, gửi "COMPLETED" về Web            │
│    └──────────────────────▶ STATE_IDLE              │
└───────────────────────────────────────────────────┘
```

#### 7.2.2. PID Line Following — Dò line bằng PID

```cpp
// Pseudocode
float error = calculateLineError(sensors[5]);  // [-4, +4]
float P = Kp * error;
float I = Ki * integral;    // integral += error * dt
float D = Kd * (error - prevError) / dt;
float correction = P + I + D;

int speedL = BASE_SPEED + correction;
int speedR = BASE_SPEED - correction;
setMotor(LEFT,  constrain(speedL, 0, 255));
setMotor(RIGHT, constrain(speedR, 0, 255));
```

#### 7.2.3. Xử lý giao lộ (Intersection Handling)

Khi 5 mắt TCRT5000 đều phát hiện line (intersection detected):

1. Robot tiến thêm ~2 cm (encoder-based) để tâm robot nằm đúng tâm giao lộ.
2. Dequeue lệnh tiếp theo từ route queue.
3. Thực thi lệnh:
   - **"F"**: Tiếp tục PID line following.
   - **"L"**: Dừng → Xoay tại chỗ ngược chiều kim đồng hồ ~90° (sử dụng encoder đếm xung hoặc timer-based) → Tìm lại line → PID.
   - **"R"**: Dừng → Xoay tại chỗ theo chiều kim đồng hồ ~90° → Tìm lại line → PID.

#### 7.2.4. Tránh vật cản trên đường thực tế

Nếu HC-SR04 phát hiện vật cản < 15 cm khi đang dò line:

1. **Dừng ngay lập tức.**
2. Servo SG90 quét trái (150°) và phải (30°) → đo khoảng cách 2 phía.
3. **Gửi cảnh báo** về Web Dashboard: `{"type":"OBSTACLE","dist":12.5}`.
4. Hai tùy chọn xử lý (cấu hình trên Web):
   - **Chế độ an toàn**: Dừng hẳn, chờ người dùng dọn vật cản, nhấn "Resume" trên web.
   - **Chế độ tự động**: Vòng tránh bằng thuật toán hình chữ U (sử dụng encoder đo quãng đường) → tìm lại line.

#### 7.2.5. Encoder Odometry — Đo quãng đường

```
Chu vi bánh = π × 65mm ≈ 204.2 mm
Xung mỗi vòng = PPR × Gear_ratio (ví dụ: 20 × 48 = 960)
Mỗi xung ≈ 0.213 mm

Quãng đường 1 đoạn grid ≈ 300 mm → ~1408 xung
→ Robot biết khi nào đã đi hết 1 ô grid (kiểm chứng bằng intersection detection)
```

---

## 8. Timeline — Lộ trình thực hiện (4 tuần)

### Tổng quan Gantt

```
Tuần 1 ──────────────────────────────────────
│ Ngày 1-2: Nghiên cứu thuật toán AI         │
│ Ngày 3-4: Thiết kế kiến trúc + sơ đồ mạch  │
│ Ngày 5-7: Lắp ráp phần cứng + test nguồn   │
──────────────────────────────────────────────

Tuần 2 ──────────────────────────────────────
│ Ngày 1-3: Firmware cơ bản (motor, sensor,   │
│           encoder, PID line following)       │
│ Ngày 4-5: Web Map Editor + AI Algorithms    │
│ Ngày 6-7: Animation Engine cho web          │
──────────────────────────────────────────────

Tuần 3 ──────────────────────────────────────
│ Ngày 1-2: Giao tiếp WebSocket Web↔ESP32    │
│ Ngày 3-4: Route Interpreter + State Machine │
│ Ngày 5-6: Tránh vật cản + Telemetry        │
│ Ngày 7:   Tích hợp toàn bộ hệ thống        │
──────────────────────────────────────────────

Tuần 4 ──────────────────────────────────────
│ Ngày 1-3: Test tổng hợp + Fix bug          │
│ Ngày 4-5: Tối ưu PID + Test nhiều bản đồ   │
│ Ngày 6:   Quay video demo                   │
│ Ngày 7:   Viết báo cáo + Slide             │
──────────────────────────────────────────────
```

### Chi tiết từng tuần

| Tuần | Giai đoạn | Công việc chi tiết | Deliverable |
|------|-----------|-------------------|-------------|
| **Tuần 1** | **Nghiên cứu & Phần cứng** | • Nghiên cứu lý thuyết 5 thuật toán tìm kiếm (BFS, DFS, UCS, A*, Greedy). <br>• Thiết kế sơ đồ nguyên lý mạch điện, sơ đồ khối hệ thống. <br>• Lắp ráp khung Mica, gắn motor, bánh xe, encoder. <br>• Kết nối 2 bộ LM2596 (5V + 7.4V), test ổn định nguồn. <br>• Gắn 5× TCRT5000, HC-SR04, Servo lên khung. | Khung robot hoàn chỉnh, nguồn ổn định, sơ đồ nguyên lý. |
| **Tuần 2** | **Firmware & Web AI** | • Lập trình firmware ESP32: đọc 5 cảm biến line, đọc encoder (interrupt), điều khiển motor qua L298N (PWM). <br>• Tích hợp PID line following cơ bản, test trên track thẳng. <br>• Phát triển web: Map Editor (Canvas), logic 5 thuật toán AI bằng JavaScript. <br>• Xây dựng Animation Engine hiển thị quá trình tìm kiếm từng bước. | Robot bám line ổn định; Web mô phỏng 5 thuật toán hoạt động. |
| **Tuần 3** | **Tích hợp hệ thống** | • Cấu hình ESP32 Web Server + WebSocket. <br>• Lập trình Route Sender (web) + Route Interpreter (ESP32). <br>• Triển khai State Machine: giao lộ detection, xử lý lệnh F/L/R. <br>• Lập trình tránh vật cản (Servo quét + HC-SR04). <br>• Xây dựng Telemetry Dashboard (real-time sensor data). <br>• Test tích hợp: web gửi route → robot chạy đúng trên track. | Hệ thống end-to-end hoạt động: Web AI → ESP32 → Robot. |
| **Tuần 4** | **Kiểm thử & Báo cáo** | • Test hệ thống trên nhiều bản đồ khác nhau (5×5, 8×8). <br>• Test tránh vật cản thực tế trên track. <br>• Tinh chỉnh PID (Kp, Ki, Kd) cho chạy mượt. <br>• So sánh hiệu năng 5 thuật toán trên cùng bản đồ (ghi bảng kết quả). <br>• Quay video demo 3–5 phút. <br>• Viết báo cáo kỹ thuật + slide thuyết trình. | Báo cáo hoàn chỉnh, video demo, source code trên GitHub. |

---

## 9. Kết quả mong đợi

### 9.1. Sản phẩm phần mềm

| Sản phẩm | Mô tả |
|---|---|
| **Giao diện Web** | Trang web hoạt động trên trình duyệt, cho phép vẽ bản đồ, chạy mô phỏng 5 thuật toán AI với animation, so sánh hiệu năng, gửi lộ trình cho robot, và giám sát real-time. |
| **Firmware ESP32** | Chương trình C++ hoàn chỉnh: nhận lệnh WebSocket, PID line following, intersection handling, obstacle avoidance, encoder odometry, telemetry reporting. |
| **Source Code** | Upload lên GitHub với README chi tiết, cấu trúc thư mục rõ ràng, comment đầy đủ. |

### 9.2. Sản phẩm phần cứng

| Sản phẩm | Mô tả |
|---|---|
| **Robot 4WD hoàn chỉnh** | Xe robot 2 tầng Mica, 4 bánh, chạy ổn định, bám line, tránh vật cản. |
| **Track thử nghiệm** | Lưới line trên sàn (ít nhất 5×5), có giao lộ rõ ràng, có vật cản test. |

### 9.3. Tài liệu

| Sản phẩm | Mô tả |
|---|---|
| **Báo cáo đồ án** | 30–50 trang, đầy đủ: lý thuyết thuật toán, thiết kế hệ thống, kết quả thực nghiệm, so sánh thuật toán, nhận xét, hướng phát triển. |
| **Video demo** | 3–5 phút, thể hiện: vẽ bản đồ → chạy mô phỏng AI → gửi route → robot di chuyển thực tế. |
| **Slide thuyết trình** | 15–20 slide cho buổi bảo vệ. |

### 9.4. Tiêu chí đánh giá thành công

| Tiêu chí | Ngưỡng đạt |
|---|---|
| 5 thuật toán chạy đúng trên web | 5/5 thuật toán cho kết quả chính xác trên ≥ 3 bản đồ khác nhau. |
| Animation trực quan | Khách quan nhận biết được sự khác biệt giữa BFS, DFS, A* qua animation. |
| So sánh hiệu năng | Bảng so sánh đầy đủ (nodes, cost, time) cho ≥ 3 bản đồ. |
| Truyền lộ trình thành công | Lộ trình gửi từ web → ESP32 với tỷ lệ thành công ≥ 99%. |
| Robot đi đúng lộ trình | ≥ 90% các lần chạy, robot rẽ đúng tại giao lộ theo lệnh AI. |
| Tránh vật cản | Phát hiện và xử lý ≥ 95% vật cản tĩnh trên track. |

---

## 10. Tài liệu tham khảo

### 10.1. Sách giáo khoa — Thuật toán AI

1. **Russell, S. & Norvig, P.** (2021). *Artificial Intelligence: A Modern Approach* (4th ed.). Pearson.
   - **Chương 3**: Solving Problems by Searching — BFS, DFS, UCS.  
   - **Chương 4**: Informed Search — A*, Greedy Best-First Search, Heuristic Design.
   
   > ⭐ *Đây là giáo trình chuẩn cho môn AI toàn cầu; phần Search là nền tảng bắt buộc.*

2. **Ertel, W.** (2017). *Introduction to Artificial Intelligence* (2nd ed.). Springer.
   - Chương 3–4: Search algorithms with pseudocode và ví dụ minh họa.

### 10.2. Sách — Robot & Hệ thống nhúng

3. **Siegwart, R., Nourbakhsh, I.R. & Scaramuzza, D.** (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
   - Chương 3: Mobile Robot Kinematics.
   - Chương 6: Planning and Navigation.

4. **Kolban, N.** (2018). *Kolban's Book on ESP32*. Leanpub.
   - Hướng dẫn lập trình ESP32 toàn diện (Wi-Fi, GPIO, Interrupt, PWM).

### 10.3. Tài liệu kỹ thuật (Datasheets)

5. **Espressif Systems.** *ESP32 Technical Reference Manual* (v5.1).  
   [https://www.espressif.com/en/support/documents/technical-documents](https://www.espressif.com/en/support/documents/technical-documents)

6. **STMicroelectronics.** *L298N Dual Full-Bridge Driver Datasheet.*

7. **Vishay.** *TCRT5000 Reflective Optical Sensor Datasheet.*

### 10.4. Bài báo & Tài liệu nghiên cứu

8. **Hart, P.E., Nilsson, N.J. & Raphael, B.** (1968). "A Formal Basis for the Heuristic Determination of Minimum Cost Paths." *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.  
   > *Bài báo gốc về thuật toán A* — reading bắt buộc.*

9. **Cormen, T.H., Leiserson, C.E., Rivest, R.L. & Stein, C.** (2022). *Introduction to Algorithms* (4th ed.). MIT Press.
   - Chương 20–22: Graph algorithms (BFS, DFS, shortest paths).

### 10.5. Từ khóa tìm kiếm

| Chủ đề | Từ khóa tiếng Anh |
|---|---|
| Thuật toán tìm kiếm AI | `AI search algorithms visualization`, `BFS DFS A* comparison` |
| A* pathfinding | `A* pathfinding tutorial grid map`, `A* heuristic admissible` |
| Mô phỏng web | `pathfinding visualization JavaScript Canvas`, `graph search animation web` |
| ESP32 WebSocket | `ESP32 WebSocket real-time communication Arduino` |
| Robot dò line PID | `PID line follower robot 5 sensor Arduino` |
| AGV indoor navigation | `AGV line following intersection detection` |

### 10.6. Tài nguyên mở & Thư viện

10. **Pathfinding.js** — [https://github.com/qiao/PathFinding.js](https://github.com/qiao/PathFinding.js)  
    Thư viện JavaScript mã nguồn mở triển khai nhiều thuật toán pathfinding trên grid (tham khảo cấu trúc code).

11. **Pathfinding Visualizer** — [https://clementmihailescu.github.io/Pathfinding-Visualizer/](https://clementmihailescu.github.io/Pathfinding-Visualizer/)  
    Ứng dụng web trực quan hóa thuật toán pathfinding (tham khảo UX/UI).

12. **Arduino-ESP32 Core** — [https://github.com/espressif/arduino-esp32](https://github.com/espressif/arduino-esp32)  
    Framework lập trình ESP32 bằng Arduino IDE.

13. **ESPAsyncWebServer** — [https://github.com/me-no-dev/ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)  
    Web Server + WebSocket cho ESP32.

14. **RandomNerdTutorials** — [https://randomnerdtutorials.com/](https://randomnerdtutorials.com/)  
    Hướng dẫn thực hành ESP32 phong phú (WebSocket, sensors, motor control).

---

> **Ghi chú cuối:**  
> Đồ án này kết hợp **hai trụ cột kiến thức**: **(1)** Thuật toán AI tìm kiếm — phần lý thuyết cốt lõi của môn Trí tuệ Nhân tạo, và **(2)** Hệ thống Robot-IoT — phần thực hành triển khai trên phần cứng thực tế. Sự kết hợp này tạo ra một dự án có chiều sâu về mặt học thuật và có tính ứng dụng cao, phù hợp cho đồ án môn học cấp đại học.
> 
> Các thông số kỹ thuật cụ thể (Kp, Ki, Kd, kích thước grid, khoảng cách giữa các giao lộ...) sẽ được xác định chính xác trong quá trình thực nghiệm.

---

*Đề cương được soạn thảo bởi: [Tên sinh viên]*  
*GVHD: [Tên giảng viên]*  
*Ngày soạn: 17/03/2026*
