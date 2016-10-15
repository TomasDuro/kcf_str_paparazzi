/*
 * Copyright (C) TomásDuro
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
 * @file "modules/computer_vision/trackers/STR/STR.h"
 * @author TomásDuro
 * KCF tracker implementation.http://www.samhare.net/research/struck
 */

#ifndef STR_H
#define STR_H

#include "modules/computer_vision/cv.h"

bool_t STR_func(struct image_t* unimg);

extern void STR_init(void);

#endif
