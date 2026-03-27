#pragma once
#include <Arduino.h>

// Định nghĩa 3 chế độ hoạt động của xe
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2 };

// Biến khóa an toàn: Chỉ khi = true thì xe mới được phép lăn bánh
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void motorsStop();