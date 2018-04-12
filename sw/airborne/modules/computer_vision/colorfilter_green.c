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
uint8_t color_lum_min       = 0;
uint8_t color_lum_max       = 130;
uint8_t color_cb_min        = 0;
uint8_t color_cb_max        = 132;
uint8_t color_cr_min        = 0;
uint8_t color_cr_max        = 131;

/*
These variables are used to determine the window size for the sector CalculateSectorAverages function
and to determine the control window size which is used by the safeToGoForwards function.
*/
uint8_t h_sectors           = 13;               // Should be an uneven number such that img->h/h_sectors is an integer.
uint8_t v_sectors           = 16;               // Should be an number such that img->w/v_sectors  is an interger.
uint8_t sector_end          = 8;                // This variable states how many colums the sector_averages array has.
uint8_t control_sector_end  = 5;
uint16_t binary_threshold   = 140;              // If the average Y value of one of the sector_averages sectors is lower than this value, the sector is considered not safe.
uint8_t win                 = 5;                // Should be an uneven number <= h_sectors.
uint16_t sector_height;                         // The height of one sector.
uint16_t sector_width;                          // The width of one sector.
uint16_t y_end;

uint8_t colorRecGreen[4]    = {90,80,70,80};
uint8_t colorRecRed[4]      = {200,30,150,20};

// Result
int heading_increment       = 0;                 // initialisatio  of the resulting heading increment determined from the heading function. This will be used for the attitude control.
int safetogo                = 0;                 // initialisation of the resulting value of the SafeToGoForwards funtion.

#include "subsystems/abi.h"

// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  // initialisation of one column of the sector_averages array.
  uint8_t *sector_averages[h_sectors];

  // Determination of the dimensions of the sectors in the control window.
  sector_height = img->w/v_sectors;
  sector_width = img->h/h_sectors;

  // Determination of the column where the CalculateSectorAverages should stop.
  y_end = sector_height*sector_end;

  // Here the memory for the full sector_averages array is allocated.
  for (int i = 0; i < (h_sectors); i++)
  {
  	sector_averages[i] = (uint8_t *)malloc((sector_end) * sizeof(uint8_t));
  }

  //Change the image to black and white. The pixels which are considered green become white and the rest becomes black.
  image_yuv422_colorfilt(img, img, color_lum_min, color_lum_max, color_cb_min, color_cb_max, color_cr_min, color_cr_max);

  //Fill the sector_averages array with ones and zeros. A one is given when the average Y values is higher than binary threshold othwise it gives a zero.
  CalculateSectorAverages(img, y_end, sector_width, sector_height, sector_averages);

/*
  for(int l = 0; l < h_sectors; ++l){
    for(int k = 0; k < sector_end; ++k){
      printf("%d ", sector_averages[l][k]);
    }
    printf("\n");
  }
*/
  //This function uses the sector_averages array to determine whether it is safe to go forward.
  safetogo = safeToGoForwards(sector_averages);

  /*This part draws rectangles on the image. A green rectangle is drawn when the value of the corresponding 
   *element in sector_averages is 1 otherwise the rectangle becomes red.
   */
  for (int i = 0; i < sector_end; i++)
    for(int j = 0; j < h_sectors; j++)
      if (sector_averages[j][i] == 1)
        image_draw_rectangle(img, i*sector_height, (i+1)*sector_height, j*sector_width, (j+1)*sector_width, colorRecGreen);
      else 
        image_draw_rectangle(img, i*sector_height, (i+1)*sector_height, j*sector_width, (j+1)*sector_width, colorRecRed);

  // here the heading increment is determined.
  heading_increment = heading(sector_averages);

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
 * This function detemines wheter an element of the sector averages array should be a zero or a one.
 * the average Y value of a sector (sector_h*sector_w) is calculated and the function immediately checks
 * if the average is larger than binary_threshold. If it is higher output_array[(x+1)/sector_w-1][s] becomes 1
 * otherwise it becomes 0.
 *
 * The image is a very long list of UYVY values. Only the Y values are used for the calculation. The function
 * scans over each row of a sector and sums up all the Y values. Skipping to the next row is done by continueing
 * the summation x*input_img->w*2 elements further. This is done until img->h skips are done. Once this value is
 * x is set back to -1 and s will be increased by 1. Then summation starts again from the top of the image, but 
 * now s*sector_h*2 further.
 */
void CalculateSectorAverages (struct image_t *input_img, uint16_t y_end, uint8_t sector_w, uint8_t sector_h, uint8_t **output_array) {

  uint8_t *source = (uint8_t *)input_img->buf;
  int sum = 0;
  int s = 0;

  for(int x = 0; x < input_img->h; ++x){
    for(int y = s*sector_h*2; y <y_end*2; y+=2) {
      sum += source[x*input_img->w*2+y+1];
      if(y+2 == (s+1)*sector_h*2) {
        break;
      }
    }
    if((x+1)%sector_w == 0) {
      if (sum/(sector_w*sector_h) > binary_threshold) {
        output_array[(x+1)/sector_w-1][s] = 1;
      }
      else {
        output_array[(x+1)/sector_w-1][s] = 0;
      }
      sum = 0;
    }
    if(x == input_img->h-1 && s < (y_end/sector_h) - 1) {
      x = -1;
      ++s;
    }
  }
}

/*
 * The SafeToGoForwards function uses the sector_averages array and the values of win and h_sectors to determine
 * which elements of the array should be used. The used elements of this function form the control window in which
 * is determined whether it is safe to continue the trajectory.
 *
 * This function again uses the freeColumn function which, as the name already implies, determines wheter a column
 * is free. If a column is free count is increased by 1. At the end of the loop is chacked if count < win and when
 * this is the case it is not safe to go forward anymore are false is returned.
 */
bool safeToGoForwards(uint8_t **input_array) 
{
 
  int count = 0; 

  for (int i = (h_sectors)/2 - (win-1)/2; i < (h_sectors)/2 + (win-1)/2 + 1; i++) {
    count += freeColumn(input_array, i, control_sector_end);
  }
  if (count < win) {
    return false;
  }
  else
    return true;
}

/*
 * The heading function returns the direction (+ or -) and a given heading increment after the largestColumn function
 * has determined on which side of the screen (left or right) the largest group of free columns is located. If the output
 * of the largestColumn function is h_sectors/2 it means that the correct direction is straight ahead and the heading will
 * not be changed.
 */
int8_t heading(uint8_t **input_array) {

  if (largestColumn(input_array) > 0) { //h_sectors/2
    // right
    return 8; 
  }
  else if (largestColumn(input_array) == 0) { //h_sectors/2
    return 0;
  }
  else {
    // left
    return -8; 
  }
}

/*
 * The freeColumn function also uses sector_averages as input, but also an index value of a certain column (idx). It then loops
 * over the column (from 0 to sector_end) and when all elements of the column are 1 a true boolean is returned otherwise it returns false.
 */
uint8_t freeColumn(uint8_t **input_array, int idx, int stop) {

  uint8_t count = 0;

  for (int i = 0; i < stop; i++){
    count += input_array[idx][i];
  }
  if (count == stop)
    return true;
  else
    return false;
}

/*
 * The largestColumn function determines the index of the column which is in the center of the largest group of free columns.
 */
int16_t largestColumn(uint8_t **input_array) {

  int count = 0;
  int maxCount = 0;
  int8_t CoG = 0; 

  for (int i = 0; i < h_sectors; i++) {
        if (freeColumn(input_array, i, sector_end) == 1) {
            ++count;
          if (count > maxCount) {
            maxCount = count;
            CoG += weight(i);
          }
        }
        else
         count = 0; 
    }  
  return CoG; //idx;   
}

int8_t weight(int input){
  if(input < h_sectors/2)
    return -1;

  else if(input > h_sectors/2)
    return 1;

  else
    return 0;
}