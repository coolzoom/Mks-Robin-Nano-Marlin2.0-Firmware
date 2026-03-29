/**
 * robotbase_chassis.cpp
 *
 * Maps robotbase serial commands to Marlin G1 moves on four independent linear axes:
 *   X, Y, Z = three wheel motors; axis A (AXIS4_NAME) = fourth motor on E1 / "Z2" driver socket.
 *
 * Wheel direction table matches robotbase.ino (adjust INVERT_*_DIR in Configuration.h if needed).
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(ROBOTBASE_CHASSIS)

#include "robotbase_chassis.h"

#include "../gcode/gcode.h"
#include "../gcode/queue.h"
#include "../module/planner.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#ifndef ROBOTBASE_CRUISE_MM
  #define ROBOTBASE_CRUISE_MM 1000000.0f
#endif

#ifndef ROBOTBASE_MECANUM_FACTOR
  #define ROBOTBASE_MECANUM_FACTOR 0.70710678f
#endif

#ifndef ROBOTBASE_ROT_RADIUS_MM
  #define ROBOTBASE_ROT_RADIUS_MM 150.0f
#endif

static bool rb_ieq(const char *a, const char *b) {
  while (*a && *b) {
    if (tolower(*a++) != tolower(*b++)) return false;
  }
  return !*a && !*b;
}

static bool rb_prefix_ci(const char *a, const char *pre) {
  while (*pre) {
    if (!*a || tolower(*a++) != tolower(*pre++)) return false;
  }
  return true;
}

static void rb_send_ack() { SERIAL_ECHOLNPGM("ACK"); }

static float rb_clamp_speed(float v) {
  if (isnan(v) || v < 0.f) return 50.f;
  return _MIN(v, 200.f);
}

static float rb_clamp_dist(float v) {
  if (isnan(v) || v < 0.f) return 0.f;
  return _MIN(v, 10000.f);
}

static void rb_run_move(const int8_t sx, const int8_t sy, const int8_t sz, const int8_t sa,
                        const float mag_mm, const float feed_mm_s
) {
  const float f_mm_m = feed_mm_s * 60.f;
  const float mx = mag_mm * sx, my = mag_mm * sy, mz = mag_mm * sz, ma = mag_mm * sa;
  char line[88];
  snprintf(line, sizeof(line), "G91\nG1 X%.3f Y%.3f Z%.3f %c%.3f F%.1f",
           mx, my, mz, char(AXIS4_NAME), ma, f_mm_m);
  gcode.process_subcommands_now(line);
}

static void rb_stop() {
  planner.quick_stop();
  queue.clear();
}

static void rb_identify() { SERIAL_ECHOLNPGM("robotbase"); }

bool process_robotbase_chassis_line(const char *cmd) {
  if (!cmd) return false;

  while (*cmd == ' ' || *cmd == '\t') cmd++;
  if (!*cmd) return false;

  const char c0u = toupper(cmd[0]);
  if (c0u == 'G' || c0u == 'M' || c0u == 'T') return false;

  if (rb_ieq(cmd, "ID") || rb_ieq(cmd, "?")) {
    rb_identify();
    return true;
  }

  if (rb_ieq(cmd, "STATUS")) {
    SERIAL_ECHOPGM("move_state=");
    SERIAL_ECHOLN(planner.movesplanned() ? 1 : 0);
    return true;
  }

  if (rb_ieq(cmd, "FINDRAY")) {
    SERIAL_ECHOLNPGM("raytracking enabled");
    return true;
  }

  if (rb_ieq(cmd, "DISABLERAY")) {
    SERIAL_ECHOLNPGM("raytracking disabled");
    return true;
  }

  // From here on: motion protocol only (avoid ACK on stray host lines).
  {
    const char u = toupper(cmd[0]);
    const bool motion = !!strchr("FBRLS", u);
    const bool dist = rb_prefix_ci(cmd, "SDL") || rb_prefix_ci(cmd, "SDR")
                   || rb_prefix_ci(cmd, "FD") || rb_prefix_ci(cmd, "BD")
                   || rb_prefix_ci(cmd, "LD") || rb_prefix_ci(cmd, "RD");
    if (!motion && !dist) return false;
  }

  rb_send_ack();

  if (rb_ieq(cmd, "S")) {
    rb_stop();
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  float speed = 50.f;
  const char *colon = strchr(cmd, ':');
  char *parse_end = nullptr;

  // --- Distance modes ---
  if (rb_prefix_ci(cmd, "SDL")) {
    float d = rb_clamp_dist(strtof(cmd + 3, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    rb_run_move(-1, -1, 1, 1, d / ROBOTBASE_MECANUM_FACTOR, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  if (rb_prefix_ci(cmd, "SDR")) {
    float d = rb_clamp_dist(strtof(cmd + 3, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    rb_run_move(1, 1, -1, -1, d / ROBOTBASE_MECANUM_FACTOR, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  if (rb_prefix_ci(cmd, "FD")) {
    float d = rb_clamp_dist(strtof(cmd + 2, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    rb_run_move(-1, 1, -1, 1, d / ROBOTBASE_MECANUM_FACTOR, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  if (rb_prefix_ci(cmd, "BD")) {
    float d = rb_clamp_dist(strtof(cmd + 2, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    rb_run_move(1, -1, 1, -1, d / ROBOTBASE_MECANUM_FACTOR, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  if (rb_prefix_ci(cmd, "LD")) {
    float ang = rb_clamp_dist(strtof(cmd + 2, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    const float m = (2.f * float(M_PI) * ROBOTBASE_ROT_RADIUS_MM) * (ang / 360.f);
    rb_run_move(1, 1, 1, 1, m, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  if (rb_prefix_ci(cmd, "RD")) {
    float ang = rb_clamp_dist(strtof(cmd + 2, &parse_end));
    if (colon) speed = rb_clamp_speed(strtof(colon + 1, nullptr));
    const float m = (2.f * float(M_PI) * ROBOTBASE_ROT_RADIUS_MM) * (ang / 360.f);
    rb_run_move(-1, -1, -1, -1, m, speed);
    gcode.process_subcommands_now_P(PSTR("M400"));
    SERIAL_ECHOLNPGM("ok");
    return true;
  }

  // --- Velocity (cruise) ---
  const float cruise = ROBOTBASE_CRUISE_MM;

  if (c0u == 'F' && toupper(cmd[1]) != 'D') {
    speed = strlen(cmd) > 1 ? rb_clamp_speed(strtof(cmd + 1, nullptr)) : 50.f;
    rb_run_move(-1, 1, -1, 1, cruise, speed);
    return true;
  }

  if (c0u == 'B' && toupper(cmd[1]) != 'D') {
    speed = strlen(cmd) > 1 ? rb_clamp_speed(strtof(cmd + 1, nullptr)) : 50.f;
    rb_run_move(1, -1, 1, -1, cruise, speed);
    return true;
  }

  if (c0u == 'L' && toupper(cmd[1]) != 'D') {
    speed = strlen(cmd) > 1 ? rb_clamp_speed(strtof(cmd + 1, nullptr)) : 50.f;
    rb_run_move(1, 1, 1, 1, cruise, speed);
    return true;
  }

  if (c0u == 'R' && toupper(cmd[1]) != 'D') {
    speed = strlen(cmd) > 1 ? rb_clamp_speed(strtof(cmd + 1, nullptr)) : 50.f;
    rb_run_move(-1, -1, -1, -1, cruise, speed);
    return true;
  }

  if (c0u == 'S' && (cmd[1] == 'L' || cmd[1] == 'l')) {
    speed = strlen(cmd) > 2 ? rb_clamp_speed(strtof(cmd + 2, nullptr)) : 50.f;
    rb_run_move(-1, -1, 1, 1, cruise / ROBOTBASE_MECANUM_FACTOR, speed);
    return true;
  }

  if (c0u == 'S' && (cmd[1] == 'R' || cmd[1] == 'r')) {
    speed = strlen(cmd) > 2 ? rb_clamp_speed(strtof(cmd + 2, nullptr)) : 50.f;
    rb_run_move(1, 1, -1, -1, cruise / ROBOTBASE_MECANUM_FACTOR, speed);
    return true;
  }

  SERIAL_ECHOLNPGM("unknown");
  return true;
}


#endif // ROBOTBASE_CHASSIS
