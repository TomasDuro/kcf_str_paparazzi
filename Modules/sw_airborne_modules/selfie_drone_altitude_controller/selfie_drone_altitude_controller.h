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
 * @file "modules/selfie_drone_altitude_controller/selfie_drone_altitude_controller.h"
 * @author Tomas Duro
 * Altitude controller for the selfie drone
 */

#ifndef SELFIE_DRONE_ALTITUDE_CONTROLLER_H
#define SELFIE_DRONE_ALTITUDE_CONTROLLER_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

// extern bool control_switch_vertical(void);

void altitude_controller_init(void);

#endif
