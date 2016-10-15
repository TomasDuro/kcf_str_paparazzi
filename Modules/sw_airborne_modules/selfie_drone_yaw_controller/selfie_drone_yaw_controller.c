/*
 * Copyright (C) TomasDuro
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
 * @file "modules/test_control_loop/test_control_loop.c"
 * @author TomasDuro
 *
 */

 #include "modules/selfie_drone_yaw_controller/selfie_drone_yaw_controller.h"
 #include "modules/computer_vision/cv.h"
 #include "../state.h"
 #include <stdio.h>

 extern float heading_TommyDrone = 0;
 extern float yaw_increment_rad = 0;
 // extern int activate_control = 0;


 int Xa;
 int Wa;

 // bool_t test_control_loop_func(struct image_t* img);
 // bool_t test_control_loop_func(struct image_t* img)
 struct image_t* selfie_drone_yaw_controller_func(struct image_t* img);
 struct image_t* selfie_drone_yaw_controller_func(struct image_t* img)
 {
   //ARRANJAR SOLUOES PARA OS INTS
   float error;
   float pos;
   float W = 124;
   float fov = 60;
   float horiz_pixels = 124;
  //  float limit = 10;//DEG
   float pixel_step;
   float yaw_increment_deg;
  //  float yaw_increment_rad = 0;
   float pi = 3.1415926535897932384626433832795028841971693993751058209749;
  //  int32_t yaw_increment_rad_paparazzi = 0;
  //  int32_t final_increment = 0;

   //  int border;

   pos = Xa + Wa/2;
   error = pos - (W/2);
   pixel_step = (float)(fov/horiz_pixels);
   yaw_increment_deg = pixel_step * error;
   yaw_increment_rad = yaw_increment_deg * pi / 180;
  //  yaw_increment_rad_paparazzi = (int32_t)(yaw_increment_rad * 4096);


  //  if(activate_control)
  //  {
  //    heading_TommyDrone = 0;
  //    // border = abs(yaw_increment_rad); // Talvez ajude se estiver a oscilar demasiado. no library
  //   //  if(yaw_increment_deg > limit)
  //   //  {
  //   //    final_increment = limit*pi*4096/180;
  //   //    printf("cima\n");
  //   //  }
  //   //  else if(yaw_increment_deg < -limit)
  //   //  {
  //   //    final_increment = -limit*pi*4096/180;
  //   //    printf("baixo\n");
  //   //  }
  //   //  else
  //   //  {
  //   //    final_increment = yaw_increment_rad_paparazzi;
  //   //    printf("Estou dentro dos limites\n");
  //   //  }
  //   //  printf("B\n");
  //   //  heading_TommyDrone = stateGetNedToBodyEulers_i()->psi + final_increment;
  //  }
  //  else
  //  {
  //   //  printf("A\n");
  //   //  heading_TommyDrone = stateGetNedToBodyEulers_i()->psi;
  //   heading_TommyDrone = 0;
  //  }
   return NULL; //NOT RETURNING AN IMAGE BECAUSE IT IS THE LAST MODULE
 }

 // bool control_switch_yaw(void)
 // {
 //   activate_control = 1;
 //   return FALSE;
 // }

 void yaw_controller_init(void)
 {
  //  cv_add(test_control_loop_func);
   cv_add_to_device(&CAMERA,selfie_drone_yaw_controller_func);
 }
