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
 * @file modules/computer_vision/colorfilter_green.c
 */

// Own header
#include "modules/computer_vision/colorfilter_green.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#include <cv.h>
#include <highgui.h>

using namespace cv;

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
int color_count = 0;

#include "subsystems/abi.h"



// Function
struct image_t *colorfilter_func(struct image_t *img);
//void sector_averager (int hor_sectors, int vert_sectors, int sector_h, int sector_w, int **input_array, float **output_array);
struct image_t *colorfilter_func(struct image_t *img)
{
  // Filter
  uint8_t Width = img.w;
  uint8_t Height = img.h/3;

  // Change the image.h to the cropped image height.
  img->h = Height;

  source = (uint8_t)img->buf;

  X = 0;
  Y = img.h*2/3;

  // Crop the image to the region of interest for the ostacle avoidance.
  img->buf = source(Rect(X,Y,Width,Height));

  // Determine the color count of the green pixels and change the image to black and white.
  // location of function: image.c line 151.
  color_count = image_yuv422_colorfilt(img, img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );

  if (COLORFILTER_SEND_OBSTACLE) {
    if (color_count > 20)
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
    }
    else
    {
      AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
    }
  }

  return img; // Colorfilter did not make a new image, but it changed the input image.
}

/*
void sector_averager (int hor_sectors, int vert_sectors, int sector_h, int sector_w, 
            int **input_array, float **output_array)
{
  float sum = 0;

  for(int i = 0; i < vert_sectors; i++){
    for(int j = 0; j<hor_sectors; j++){
      for(int k = 0; k<(sector_h); k++){
        for(int l = 0; l<(sector_w); l++){

          sum += input_array[k+i*sector_h][l+j*sector_w];

        }
      }

    output_array[i][j] = sum/(sector_h*sector_w);
    sum = 0;

    };
  }
}
*/

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
