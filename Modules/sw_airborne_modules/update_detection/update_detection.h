/*
 * Copyright (C) Tomás Duro
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/stereo_detection/stereo_detection.h"
 * @author Tomás Duro
 * Receive the bounding box parameters from the stereoboard
 */

#ifndef UPDATE_DETECTION_H
#define UPDATE_DETECTION_H

#include <std.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "../state.h"
#include "modules/stereocam/stereocam.h"
#include "firmwares/rotorcraft/autopilot.h"

void update_detection_init(void);
void update_detection_periodic(void);
void update_detection(void);
void yaw_controller_init(void);
void waypoint_generator_init(void);
void altitude_controller_init(void);
void guided_mode_control_init(void);
bool change_to_guided(void);
bool change_back(void);
void selfie_drone_yaw_controller_func(void);
void waypoint_generator_func(void);
void selfie_drone_altitude_controller_func(void);
void guided_mode_control_func(void);

#endif
