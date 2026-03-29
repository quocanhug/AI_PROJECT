#ifndef DO_LINE_H
#define DO_LINE_H

#include <Arduino.h>

enum UIMode {
  MODE_MANUAL,
  MODE_LINE_ONLY,
  MODE_DELIVERY,
  MODE_AI_ROUTE
};
// Định nghĩa struct PID
struct PID {
  float Kp, Ki, Kd;
  float i_term, prev_err;
  float out_min, out_max;
};

// Khai báo các hàm để file khác có thể gọi
void do_line_setup();
void do_line_loop();
void do_line_abort();
int getTargetDirection(int nodeA, int nodeB);
void motorsStop();

#endif