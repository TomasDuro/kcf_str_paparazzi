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
* @file "modules/computer_vision/video_crop.c"
* @author Tomas Duro
* Crops the frames from the video camera.
*/
#include "modules/computer_vision/video_crop.h"
#include "modules/computer_vision/cv.h"
#include <stdlib.h>
#include <std.h>
// #include "subsystems/datalink/telemetry.h"
#include <stdio.h>

uint16_t Wcrop = 124;
uint16_t Hcrop = 96;
uint16_t displacement = 52;//52 parece ser bom!
struct image_t *imgCrop = NULL;
// Function
struct image_t* video_crop_func(struct image_t* img);
struct image_t* video_crop_func(struct image_t* img)
{
  uint8_t *imgbuf = img->buf;

  if (imgCrop == NULL) {
    imgCrop = malloc(sizeof(struct image_t));
    image_create(imgCrop,Wcrop,Hcrop,IMAGE_YUV422);
  }

  uint8_t *imgCropbuf = imgCrop->buf;

  for (uint16_t y = 0; y < Hcrop; y++) {
  for (uint16_t x = 0; x < Wcrop; x += 2) {
      // //CROPS A CENTERED Hcrop BY Wcrop IMAGE
      // //CROP INTO THE CORNER OF THE ORIGINAL IMAGE
      // imgbuf[y*img->w*2+x*2]   = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2];
      // imgbuf[y*img->w*2+x*2+1] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+1];
      // imgbuf[y*img->w*2+x*2+2] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+2];
      // imgbuf[y*img->w*2+x*2+3] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+3];
      // //CROP INTO NEW IMAGE
      // imgCropbuf[y*Wcrop*2+x*2]   = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2];
      // imgCropbuf[y*Wcrop*2+x*2+1] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+1];
      // imgCropbuf[y*Wcrop*2+x*2+2] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+2];
      // imgCropbuf[y*Wcrop*2+x*2+3] = imgbuf[img->w*2*((img->h-Hcrop)/2+y)+(img->w-Wcrop)+x*2+3];
      //
      //CROPS A UNCENTERED Hcrop BY Wcrop IMAGE
      //CROP INTO THE CORNER OF THE ORIGINAL IMAGE
      imgbuf[y*img->w*2+x*2]   = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2];
      imgbuf[y*img->w*2+x*2+1] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+1];
      imgbuf[y*img->w*2+x*2+2] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+2];
      imgbuf[y*img->w*2+x*2+3] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+3];
      //CROP INTO NEW IMAGE
      imgCropbuf[y*Wcrop*2+x*2]   = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2];
      imgCropbuf[y*Wcrop*2+x*2+1] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+1];
      imgCropbuf[y*Wcrop*2+x*2+2] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+2];
      imgCropbuf[y*Wcrop*2+x*2+3] = imgbuf[img->w*2*((img->h-Hcrop)/2-displacement+y)+(img->w-Wcrop)+x*2+3];
    }
  }
  return imgCrop;
}

void video_crop_init(void)
{
  cv_add_to_device(&CAMERA,video_crop_func);
}
