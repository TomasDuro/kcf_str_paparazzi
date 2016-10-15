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
* @file "modules/computer_vision/video_square_draw.c"
* @author Tomas Duro
* Draw a square on a the frames from the video camera.
*/

#include "modules/computer_vision/video_square_draw.h"
#include "modules/computer_vision/cv.h"
#include <std.h>
#include <stdio.h>

int Xa;
int Ya;
int Wa;
int Ha;

extern int isalreadyinit = 1;

// bool_t video_square_draw_func(struct image_t* img);
// bool_t video_square_draw_func(struct image_t* img)
struct image_t* video_square_draw_func(struct image_t* img);
struct image_t* video_square_draw_func(struct image_t* img)
{

  if(isalreadyinit)
  {
    Xa = 53;
    Ya = 20;
    Wa = 20;
    Ha = 40;
    isalreadyinit = 0;
  }

  //LOOP TO DRAW THE SQUARE
  uint8_t *imgbuf = img->buf;
  for (uint16_t y = 0; y < img->h; y++) {
    for (uint16_t x = 0; x < img->w; x += 2) {
      if (
        ((y == Ya) && (x >= Xa) && (x <= Xa + Wa)) ||
        ((y == Ya + Ha) && (x >= Xa) && (x <= Xa + Wa)) ||
        ((x == Xa) && (y >= Ya) && (y <= Ya + Ha)) ||
        ((x == Xa + Wa) && (y >= Ya) && (y <= Ya + Ha))) {
          // UYVY IMAGE
          imgbuf[0] = 0;    // U
          imgbuf[1] = 255;  // Y
          imgbuf[2] = 0;    // V
          imgbuf[3] = 255;  // Y
        }
          // Go to the next 2 pixels
          imgbuf += 4;
        }
      }
      return img;
    }

    void video_square_draw_init(void)
    {
      cv_add_to_device(&CAMERA,video_square_draw_func);
    }
