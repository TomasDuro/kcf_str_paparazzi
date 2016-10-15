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
 * @file "modules/selfie_drone_guided_mode_control/guided_mode_control.c"
 * @author Tomas Duro
 * Feed the control commands to the drone through the autopilot guided mode
 */

#include "modules/selfie_drone_guided_mode_control/guided_mode_control.h"
#include "modules/computer_vision/cv.h"
#include "firmwares/rotorcraft/autopilot.h"
#include <stdio.h>

float yaw_increment_rad;
float horizontal_displacement;
float vertical_increment;

// uint8_t autopilot_mode;

struct image_t* guided_mode_control_func(struct image_t* img);
struct image_t* guided_mode_control_func(struct image_t* img)
{
  //VARIABLES
  bool yaw_control = true;
  bool horizontal_control = false;
  bool altitude_control = false;
  // SO PARA A FRENTE A PARA TRAS POR ENQUANTO
  float dx = 0;
  float dy = 0;
  float dz = 0;
  float dyaw = 0;
  //SELECTING WHICH CONTROLS TO USE
  if (yaw_control)
  {
    dyaw = yaw_increment_rad;
    printf("yaw control active\n");
  }
  if (horizontal_control)
  {
    dx = horizontal_displacement;
    printf("waypoint control active\n");
  }
  if (altitude_control)
  {
    dz = -vertical_increment;
    printf("altitude control active\n");
  }



  if (autopilot_mode == 19)
  {
    printf("auto pilot: GUIDED MODE\n");
  }
  if (autopilot_mode == 13)
  {
    printf("auto pilot: NAV MODE\n");
  }
  if (autopilot_mode == 4)
  {
    printf("auto pilot: ATTITUDE MODE\n");
  }
  // printf("auto pilot mode = %d\n",autopilot_mode);
  //A ACTIVACAO ACONTECE QUANDO ENTRO EM MODO GUIDED
  // autopilot_guided_goto_ned_relative(dx,dy,dz,dyaw);
  autopilot_guided_goto_body_relative(dx,dy,dz,dyaw);
  //NOT RETURNING AN IMAGE BECAUSE IT IS THE LAST MODULE
  return NULL;
}

void guided_mode_control_init(void)
{
  cv_add_to_device(&CAMERA,guided_mode_control_func);
}

bool change_to_guided(void)
{
  autopilot_set_mode(AP_MODE_GUIDED);
  return FALSE;
}

bool change_back(void)
{
  autopilot_set_mode(AP_MODE_NAV);
  return FALSE;
}
