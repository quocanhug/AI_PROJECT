#pragma once
#include <Arduino.h>

// Cấu trúc PID dùng chung cho cả 2 hệ thống
struct PID { float Kp, Ki, Kd; float i_term; float prev_err; float out_min, out_max; };

// 4 chế độ hoạt động của xe
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2, MODE_AI_ROUTE=3 };

// Khóa an toàn: Chỉ khi = true thì xe mới nhúc nhích
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void motorsStop();