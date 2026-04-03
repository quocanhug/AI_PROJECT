#pragma once
#include <Arduino.h>

// PID controller structure
struct PID { float Kp, Ki, Kd; float i_term; float prev_err; float out_min, out_max; };

// 4 operating modes
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2, MODE_AI_ROUTE=3 };

// Safety lock: robot only moves when true
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void do_line_resume();  // Resume mà không reset route state (dùng cho RESUME command)
void motorsStop();
void wsBroadcast(const char *msg);  // ★ FIX L3: Khai bao tap trung, tranh extern rai rac