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
uint8_t color_lum_min = 0;//105;
uint8_t color_lum_max = 115;//205;
uint8_t color_cb_min  = 0;//52;
uint8_t color_cb_max  = 120;//140;
uint8_t color_cr_min  = 0;//180;
uint8_t color_cr_max  = 130;//255;
uint8_t v_sectors               = 20;
uint8_t h_sectors               = 15; // Should be an uneven number
uint8_t sector_start            = 17;
uint8_t binary_threshold        = 130;
uint8_t sector_height, sector_width;
uint8_t center;
uint8_t margin;
uint8_t win = 11; // Should be an uneven number <= to h_sectors
//uint8_t *sector_averages;

// Result
int color_count = 0;
int safetogo = 0;

#include "subsystems/abi.h"



// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  uint16_t Height = img->h;
  uint16_t Width = img->w;
  uint16_t y_start;
  
  uint16_t *sector_averages[v_sectors];
  for (int i = 0; i < v_sectors; i++)
  {
  	sector_averages[i] = (uint16_t *)malloc(h_sectors * sizeof(uint16_t));
  }

  // Determine the color count of the green pixels and change the image to black and white.
  // location of function: image.c line 151.
  color_count = image_yuv422_colorfilt(img, img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );

  sector_height = Height/v_sectors;
  sector_width = Width/h_sectors;
  y_start = sector_height*sector_start;

  CalculateSectorAverages(img, y_start, sector_height, sector_width, sector_averages);
  //printf("%d\n",sector_averages[0][0]);

  safetogo = safeToGoForwards(sector_averages);

  //image_to_grayscale(img, img);

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

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);

}

/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 */
void CalculateSectorAverages (struct image_t *input_img, uint16_t y_start ,uint8_t sector_h, uint8_t sector_w, uint16_t **output_array)
{
  uint8_t *source = (uint8_t *)input_img->buf;
  
  uint16_t sum = 0;
  uint8_t s = 0;

  for(uint16_t y = y_start; y < input_img->h; ++y){
    for(uint16_t x = s*2*sector_w ; x < input_img->w ; x += 2) {
      sum += source[1];//input_array[i][j];
        if(x == ((s+1)*sector_w-1)) {
        break;
        }
    }
    if((y+1)%sector_h == 0){
      if (sum/(sector_h*sector_w) > binary_threshold)
      {
        //output_array[(y+1)/sector_h-1+s*sector_h] = 1;
        output_array[(y+1)/sector_h-1][s] = 1;
      }
      else{
        //output_array[(y+1)/sector_h-1+s*sector_h] = 0;
        output_array[(y+1)/sector_h-1][s] = 0;
      }
      sum = 0;
    }
    if(y == input_img->h-1 && s < (input_img->w/sector_w - 1)) {
      y = -1;
      s += 1;
    }
  }
  source += 4;
}

bool safeToGoForwards(uint16_t **input_array) 
{
  center = (h_sectors+1)/2; 
  margin  = (win-1)/2;
  //int front[c]; 
  int count = 0; 
  for (int i = center - margin; i < center + margin + 1; i++) {
    //cout << "i: " << i << endl; 
    //count += freeColumn(&input_array, i);
    for(int j = 0; j < v_sectors-sector_start; j++) {
      count += input_array[j][i];
    }
  }
  //cout << "count: " << count << endl;
  if (count == 0)
    return false;
  else
    return true;
}

// // check if column is free (green) for array[r][c]
// bool freeColumn(int *input_array, int idx) {
//   int count = 0;
//   uint8_t r;
//   r = v_sectors - sector_start;

//   for (int i = 0; i < r; i++){
//     count += input_array[i][idx];
//   }
//   // check if column is free
//   if (count == r)
//     return true;
//   else
//     return false;
// }


// // check if column is free (green) for array[r*c]
// bool freeColumn_alt(int array[r*c], int idx) {
//   int count = 0;
//   for (int i = 0; i < r; i++) {
//     count += array[i*c + idx];
//   }
//   // check if column is free
//   if (count == r)
//     return true;
//   else
//     return false;
// }

// // check if safe to go forwards for array[r*c] as input 
// bool safeToGoForwards_alt(int array[r*c]) {
//   //int front[c]; 
//   int count = 0;
//   for (int i = center - margin; i < center + margin + 1; i++) {
//     //cout << "i: " << i << endl; 
//     count += freeColumn_alt(array, i);
//   }
//   //cout << "count: " << count << endl;
//   if (count == win)
//     return true;
//   else
//     return false;
// }

// void Find_Heading(uint8_t *input_array, int sector_columns, int sector_rows)
// {
//   /*
//   functionality:
//    - 
//   */
// //first part: will determine the full columns


// int single_array[sector_columns];
// int i,j;
// int count;
// int x;

// for (j = 0; j < sector_columns; j++){   
//   count= 0;
//     for(i = 0; i < n_row; i++){
//       count = count + input_array[i + j*n_row];
//     }
       
//     if (count == n_row){
//       single_array[j] = 1;
//       //printf("3 gehaald");


//     } else {
//       single_array[j] = 0;
//     }
// }
// }