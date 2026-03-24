#pragma once
#include <Arduino.h>

// ==================== Route Interpreter API ====================

// State Machine States
enum RouteState {
  RS_IDLE = 0,
  RS_FOLLOWING_LINE,
  RS_AT_INTERSECTION,
  RS_OBSTACLE,
  RS_REROUTING,
  RS_DONE
};

// Initialize route interpreter
void route_setup();

// Main loop tick — call from loop() when in AI_ROUTE mode
void route_loop();

// Load a new route from JSON: {"type":"ROUTE","commands":["F","L","R",...]}
void route_load(const char* json);

// Abort current route execution
void route_abort();

// Check if route is complete
bool route_is_done();

// Get current state as string
const char* route_state_str();

// Get current state enum
RouteState route_get_state();

// Get current step / total steps
int route_current_step();
int route_total_steps();

// Get telemetry JSON string (caller must use before next call)
String route_telemetry_json();
