/**
 * @file do_line.h
 * @brief Module dò line và điều hướng tự động cho xe robot giao hàng.
 *
 * Khai báo các cấu trúc dữ liệu, enum chế độ vận hành,
 * và giao diện hàm công khai cho module điều khiển dò line (do_line.cpp).
 * Module này quản lý toàn bộ logic: PID line-following, xử lý giao lộ,
 * phát hiện/tránh vật cản, và phục hồi khi mất line.
 */

#pragma once
#include <Arduino.h>

/* ── Cấu trúc bộ điều khiển PID ── */
struct PID {
  float Kp, Ki, Kd;           // Hệ số tỉ lệ, tích phân, vi phân
  float i_term;                // Giá trị tích lũy thành phần I
  float prev_err;              // Sai số chu kỳ trước (dùng cho thành phần D)
  float out_min, out_max;      // Giới hạn đầu ra PWM
};

/* ── Chế độ vận hành của hệ thống ── */
enum UIMode {
  MODE_MANUAL    = 0,          // Điều khiển thủ công qua D-Pad
  MODE_LINE_ONLY = 1,          // Dò line tự do (không theo lộ trình)
  MODE_DELIVERY  = 2,          // Giao hàng theo lộ trình (có gripper)
  MODE_AI_ROUTE  = 3           // Chạy lộ trình AI từ Web Dashboard
};

/* ── Cờ an toàn: robot chỉ di chuyển khi giá trị = true ── */
extern bool is_auto_running;

/* ── Giao diện hàm công khai ── */
void do_line_setup();          // Khởi tạo phần cứng và reset trạng thái
void do_line_loop();           // Vòng lặp chính: dò line, xử lý giao lộ, PID
void do_line_abort();          // Dừng khẩn cấp và vô hiệu hóa module
void do_line_resume();         // Tiếp tục lộ trình từ vị trí đã xác nhận (không reset)
void motorsStop();             // Dừng tất cả động cơ (PWM = 0)