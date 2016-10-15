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
 * @file "modules/selfie_drone_altitude_controller/selfie_drone_altitude_controller.c"
 * @author Tomas Duro
 * Altitude controller for the selfie drone
 */

#include "modules/selfie_drone_altitude_controller/selfie_drone_altitude_controller.h"
#include "modules/computer_vision/cv.h"
#include "../state.h"
#include <stdio.h>

// extern int activate_control_vertical = 0;
extern float vertical_increment = 0;


int Ya;
int Ha;

float current_dist;

struct image_t* selfie_drone_altitude_controller_func(struct image_t* img);
struct image_t* selfie_drone_altitude_controller_func(struct image_t* img)
{
  float error;
  float pos;
  float H = 96;
  float fov = 60;
  float horiz_pixels = 124;
  // float limit = 10;//DEG
  float pixel_step;
  float pi = 3.1415926535897932384626433832795028841971693993751058209749;
  float vert_half_angle;
  // float vertical_increment;

  pos = Ya + Ha/2;
  error = (H/2) - pos;
  pixel_step = fov / horiz_pixels;
  vert_half_angle = pixel_step * error;

  vertical_increment = current_dist * tan(vert_half_angle * pi / 180);

  return NULL; //NOT RETURNING AN IMAGE BECAUSE IT IS THE LAST MODULE
}


void altitude_controller_init(void)
{
  cv_add_to_device(&CAMERA,selfie_drone_altitude_controller_func);
}
