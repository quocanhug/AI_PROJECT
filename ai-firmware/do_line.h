#pragma once
#include <Arduino.h>

// Shared PID structure (used by do_line.cpp and route_interpreter.cpp)
struct PID { float Kp, Ki, Kd; float i_term; float prev_err; float out_min, out_max; };

// 4 operating modes
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2, MODE_AI_ROUTE=3 };

// Safety lock: robot only moves when true
extern bool is_auto_running;

void do_line_setup();
void do_line_loop();
void do_line_abort();
void motorsStop();