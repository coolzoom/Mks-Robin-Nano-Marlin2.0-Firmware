/**
 * robotbase_chassis.h
 * Host protocol compatible with robotbase/robotbase.ino for Mecanum 4-motor chassis.
 */
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(ROBOTBASE_CHASSIS)

bool process_robotbase_chassis_line(const char *cmd);

#endif
