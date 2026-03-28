#pragma once
#include <Arduino.h>

// Các trạng thái của máy nước AI
enum RouteState {
  RS_IDLE = 0,
  RS_FOLLOWING_LINE,
  RS_AT_INTERSECTION,
  RS_OBSTACLE,
  RS_REROUTING,
  RS_DONE
};

void route_setup();
void route_loop();
void route_load(const char* json);
void route_abort();
bool route_is_done();
const char* route_state_str();
RouteState route_get_state();
int route_current_step();
int route_total_steps();
String route_telemetry_json();

// Callback để đẩy dữ liệu Telemetry lên Web
typedef void (*WsSendFn)(const char* msg);
void route_set_ws_callback(WsSendFn fn);