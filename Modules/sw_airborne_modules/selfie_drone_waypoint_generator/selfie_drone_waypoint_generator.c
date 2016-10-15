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
 * @file "modules/selfie_drone_waypoint_generator/selfie_drone_waypoint_generator.c"
 * @author Tomas Duro
 * Calculates waypoints in order to maintain the aircraft close to the person it is following
 */

#include "modules/selfie_drone_waypoint_generator/selfie_drone_waypoint_generator.h"
#include "modules/computer_vision/cv.h"
// #include "../state.h"
#include <stdio.h>
#include <math.h>

// extern int activate_control_horizontal = 0;
extern float horizontal_displacement = 0;
extern float current_dist = 2;
extern float init_vert_dist = 0;//meters


int Xa;
int Wa;
int Ya;
int Ha;


struct image_t* waypoint_generator_func(struct image_t* img);
struct image_t* waypoint_generator_func(struct image_t* img)
{
  //VARIABLES
  // float error;
  // float pos;

  static int FirstTime = 1;
  float pi = 3.1415926535897932384626433832795028841971693993751058209749;
  float init_horiz_dist = 2;//meters
  // float init_vert_dist;//meters
  float box_angle_vert_half;
  float fov = 60;
  float horiz_pixels = 124;
  float pixel_step;
  // float current_dist;
  float current_half_angle;//deg

  //SO CHAMAR NA PRIMEIRA VEZ
  if (FirstTime)
  {
    pixel_step = fov/horiz_pixels;
    box_angle_vert_half = pixel_step * Ha / 2;
    init_vert_dist = 2 * init_horiz_dist * tan (box_angle_vert_half * pi / 180);
    FirstTime = 0;
  }

  //O QUE E CHAMADO EM LOOP
  current_half_angle = (Ha / 2) * pixel_step;
  current_dist = (init_vert_dist / 2) / tan(current_half_angle * pi / 180);
  horizontal_displacement = current_dist - init_horiz_dist;

  return NULL; //NOT RETURNING AN IMAGE BECAUSE IT IS THE LAST MODULE
}


void waypoint_generator_init(void)
{
  cv_add_to_device(&CAMERA,waypoint_generator_func);
}
