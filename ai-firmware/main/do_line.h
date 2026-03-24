#pragma once
#include <Arduino.h>

// ---- Core line following API ----
void do_line_setup();
void do_line_loop();
void do_line_abort();  // dừng ngay mọi hành vi trong do_line

// ---- Motor control ----
void motorsStop();
void driveWheelLeft(float v_target, int pwm);
void driveWheelRight(float v_target, int pwm);
void motorWriteLR_signed(int pwmL, int pwmR);

// ---- Movement functions (encoder-based) ----
void spin_left_deg(double deg, int pwmMax);
void spin_right_deg(double deg, int pwmMax);
void move_forward_distance(double dist_m, int pwmAbs);
bool move_forward_distance_until_line(double dist_m, int pwmAbs);

// ---- Sensor reading ----
float readDistanceCM();
float readDistanceCM_filtered();

// ---- PID ----
struct PID {
  float Kp, Ki, Kd;
  float i_term;
  float prev_err;
  float out_min, out_max;
};
int pidStep(PID &pid, float v_target, float v_meas, float dt_s);
float ticksToVel(long ticks, float dt_s);

// ---- Encoder data (defined in do_line.cpp) ----
extern volatile long encL_count;
extern volatile long encR_count;
extern volatile long encL_total;
extern volatile long encR_total;

// ---- Speed params (defined in do_line.cpp) ----
extern float v_base;

// ---- Mechanical constants ----
extern const float CIRC;
extern const int PPR_EFFECTIVE;
