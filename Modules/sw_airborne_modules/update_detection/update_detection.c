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
 * @file "modules/stereo_detection/stereo_detection.c"
 * @author Tomás Duro
 * Receive the bounding box parameters from the stereoboard
 */
#include "modules/update_detection/update_detection.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"

// BOUNDING BOX RESULT
int Xa;
int Ya;
int Wa;
int Ha;

// CONTROLLER
float vertical_increment = 0;
float heading_TommyDrone = 0;
float yaw_increment_rad = 0;
float horizontal_displacement = 0;
float current_dist = 2;
float init_vert_dist = 0;//meters

void update_detection_init(void)
{

}

void update_detection_periodic(void)
{
  if (stereocam_data.fresh) {
	  update_detection();
    stereocam_data.fresh = 0;
  }
}

void update_detection(void)
{
  Xa = (int)stereocam_data.data[0];
  Ya = (int)stereocam_data.data[1];
  if (autopilot_mode == 19)
  {
    selfie_drone_yaw_controller_func();
    waypoint_generator_func();
    selfie_drone_altitude_controller_func();
    guided_mode_control_func();
  }
  // printf("Fazes anos no dia %d de %d\n",Xa,Ya);
}

 void selfie_drone_yaw_controller_func(void)
 {
   float error;
   float pos;
   float W = 124;
   float fov = 60;
   float horiz_pixels = 124;
  //  float limit = 10;//DEG
   float pixel_step;
   float yaw_increment_deg;
   float pi = 3.1415926535897932384626433832795028841971693993751058209749;
   pos = Xa + Wa/2;
   error = pos - (W/2);
   pixel_step = (float)(fov/horiz_pixels);
   yaw_increment_deg = pixel_step * error;
   yaw_increment_rad = yaw_increment_deg * pi / 180;
 }

 void waypoint_generator_func(void)
 {
   static int FirstTime = 1;
   // printf("FirstTime = %d\n",FirstTime);
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
   //LOOP
   current_half_angle = (Ha / 2) * pixel_step;
   current_dist = (init_vert_dist / 2) / tan(current_half_angle * pi / 180);
   horizontal_displacement = current_dist - init_horiz_dist;
 }

void selfie_drone_altitude_controller_func(void)
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
}

void guided_mode_control_func(void)
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
  }
  if (horizontal_control)
  {
    dx = horizontal_displacement;
  }
  if (altitude_control)
  {
    dz = -vertical_increment;
  }
  //activates in guided mode
  // autopilot_guided_goto_ned_relative(dx,dy,dz,dyaw);
  autopilot_guided_goto_body_relative(dx,dy,dz,dyaw);
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
