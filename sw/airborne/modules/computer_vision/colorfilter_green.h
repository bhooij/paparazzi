/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter_green.h
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void colorfilter_init(void);
extern void CalculateSectorAverages (struct image_t *input_img,uint16_t x_end, uint8_t sector_h, uint8_t sector_w, uint8_t **output_array);
extern bool safeToGoForwards(uint8_t **input_array);
extern uint8_t freeColumn(uint8_t **input_array, int idx);
extern int8_t heading(uint8_t **input_array);
extern int16_t largestColumn(uint8_t **input_array);
extern int8_t weight(int input);

//extern bool freeColumn(int *input_array, int idx);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern int color_count;
extern int safetogo;
extern int heading_increment;

extern struct video_listener *listener;

#endif /* COLORFILTER_CV_PLUGIN_H */
