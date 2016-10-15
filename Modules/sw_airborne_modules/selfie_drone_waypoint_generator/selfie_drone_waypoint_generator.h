/*
 * Copyright (C) Tomas Duro
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
 * @file "modules/selfie_drone_waypoint_generator/selfie_drone_waypoint_generator.h"
 * @author Tomas Duro
 * Calculates waypoints in order to maintain the aircraft close to the person it is following
 */

#ifndef SELFIE_DRONE_WAYPOINT_GENERATOR_H
#define SELFIE_DRONE_WAYPOINT_GENERATOR_H

// extern void waypoint_generator_init();

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

void waypoint_generator_init(void);


#endif
