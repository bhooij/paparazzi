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
uint8_t v_sectors               = 13; //
uint8_t h_sectors               = 16;  // 
uint8_t sector_end              = 6;  //
uint16_t binary_threshold       = 130;
uint16_t sector_height, sector_width;
uint8_t center;
uint8_t margin;
uint8_t win = 11; // Should be an uneven number <= to v_sectors

// Result
int heading_increment = 0;
int color_count = 0;
int safetogo = 0;

#include "subsystems/abi.h"



// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  uint16_t x_end;
  
  uint8_t *sector_averages[v_sectors];
  for (int i = 0; i < (v_sectors); i++)
  {
  	sector_averages[i] = (uint8_t *)malloc((sector_end) * sizeof(uint8_t));
  }

  // Determine the color count of the green pixels and change the image to black and white.
  // location of function: image.c line 151.
  color_count = image_yuv422_colorfilt(img, img,
                                       color_lum_min, color_lum_max,
                                       color_cb_min, color_cb_max,
                                       color_cr_min, color_cr_max
                                      );

  sector_height = img->h/v_sectors;
  sector_width = img->w/h_sectors;
  x_end = sector_width*sector_end;

  CalculateSectorAverages(img, x_end, sector_height, sector_width, sector_averages);

  safetogo = safeToGoForwards(sector_averages);

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

void CalculateSectorAverages (struct image_t *input_img, uint16_t x_end, uint8_t sector_h, uint8_t sector_w, uint8_t **output_array) {
  uint8_t *source = (uint8_t *)input_img->buf;
  int sum = 0;
  int s = 0;

  for(int y = 0; y < input_img->h; ++y){
    for(int x = s*sector_w*2; x <x_end*2; x+=2) {
      sum += source[y*input_img->w*2+x+1];
      if(x+2 == (s+1)*sector_w*2) {
        break;
      }
    }
    if((y+1)%sector_h == 0) {
      if (sum/(sector_h*sector_w) > binary_threshold) {
        output_array[(y+1)/sector_h-1][s] = 1;
      }
      else {
        output_array[(y+1)/sector_h-1][s] = 0;
      }
      sum = 0;
    }
    if(y == input_img->h-1 && s < (x_end/sector_w) - 1) {
      y = -1;
      ++s;
    }
  }
}

/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 */
/*
void CalculateSectorAverages (struct image_t *input_img, uint16_t x_end ,uint8_t sector_h, uint8_t sector_w, uint8_t **output_array)
{
  uint8_t *source = (uint8_t *)input_img->buf;
  
  uint32_t sum = 0;
  uint8_t s = 0;

  for(uint16_t y = 0; y < input_img->h; ++y){
    //printf("%d\n",y);
    for(uint16_t x = s*sector_w ; x < x_end ; ++x) {
      sum += source[1];

      printf("%d",source[1]); printf(" "); printf("x: %d",x); printf(" ");
      source += 2; // go the the next pixel

      //printf("source[1]: %d, y: %d, x: %d\n",source[1],y,x);
      if(x == ((s+1)*sector_w-1)) {
        source += (input_img->w-sector_w+1)*2;
        break;
      }
    }
    printf("\n");
    if((y+1)%sector_h == 0){
      printf("sum: %d\n",sum);
      printf("average: %d\n",sum/(sector_h*sector_w));
      //printf("sector area: %d\n",sector_h*sector_w);
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
    //printf("%d,y: %d, s: %d\n",output_array[((y+1)/sector_h)-1][s],(y/sector_h)-1,s);
    if(y == input_img->h-1 && s < (input_img->w/sector_w - 1)) {
      //printf("s: %d\n",s);
      y = -1;
      s += 1;
      source -= (input_img->h*input_img->w)*2; // go back to the beginning of the image.
    }
  }
printf("The picture is processed for averages\n");
}
*/
bool safeToGoForwards(uint8_t **input_array) 
{
  center = (v_sectors+1)/2; 
  margin  = (win-1)/2;
  //int front[c]; 
  int count = 0; 
  for (int i = center - margin; i < center + margin + 1; i++) {
    //cout << "i: " << i << endl; 
    count += freeColumn(input_array, i);
    //for(int j = 0; j < sector_end; j++) {
    //  count += input_array[i][j];
    //}
  }
  //cout << "count: " << count << endl;
  if (count < 10)
    return false;
  else
    return true;
}

// check if column is free (green) for array[r][c]
bool freeColumn(uint8_t **input_array, int idx) {
  uint8_t count = 0;

  for (int i = 0; i < sector_end; i++){
    count += input_array[idx][i];
  }
  // check if column is free
  if (count == sector_end)
    return true;
  else
    return false;
}

// check for largest free space: left of right
uint8_t largestColumn(uint8_t **input_array) {
  int count = 0;
  int idx = 0; 
  int maxCount = 0; 
  for (int i = 0; i < v_sectors; i++) {
    //cout << "i: " << i << " | ";
    if (freeColumn(input_array, i) == 0) { 
      if (count > maxCount) {
        maxCount = count; 
        idx = i - maxCount/2 - maxCount % 2;  
      }
      count = 0; 
    }
    else
      count += 1; 
  }
  return idx;   
}

// decide to turn left of right or do nothing
uint8_t heading(uint8_t **input_array) {
  if (largestColumn(input_array) > v_sectors/2)
    // right
    return -10;
  else //(largestColumn(input_array) < v_sectors/2)
    //left 
    return 10; 
}