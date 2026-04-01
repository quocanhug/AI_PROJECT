#pragma once
#include <Arduino.h>

// Định nghĩa struct PID nếu chưa có
struct PID { 
    float Kp, Ki, Kd; 
    float i_term; 
    float prev_err; 
    float out_min, out_max; 
};

// Định nghĩa các chế độ
enum UIMode { MODE_MANUAL=0, MODE_LINE_ONLY=1, MODE_DELIVERY=2, MODE_AI_ROUTE=3 };

// Khai báo các hàm để main.ino có thể thấy
void do_line_setup();
void do_line_loop();
void do_line_abort();
void do_line_resume();
void motorsStop();

// Khai báo các biến extern để main.ino truy cập telemetry
extern float us_dist_cm;
extern float vL_ema, vR_ema;