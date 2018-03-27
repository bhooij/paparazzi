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
uint8_t v_sectors               = 15; // The amount of vertical sectors of the complete image.
uint8_t h_sectors               = 15; // Should be an uneven number
uint8_t sector_end              = 3; // The start of the control sector in the image. Should be <= h_sectors
uint8_t binary_threshold        = 130;
uint8_t sector_height, sector_width;
uint8_t center;
uint8_t margin;
uint8_t win = 11; // Should be an uneven number <= to h_sectors

// Result
int color_count = 0;
int safetogo = 0;

#include "subsystems/abi.h"



// Function
struct image_t *colorfilter_func(struct image_t *img);
struct image_t *colorfilter_func(struct image_t *img)
{
  //uint16_t Height = img->h;
  //uint16_t Width = img->w;
  //printf("FOO\n");
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

/*  
  printf("img->w: %d\n",img->w);
  printf("img->h: %d\n",img->h);
  printf("x_end: %d\n",x_end);
  printf("sector_height: %d\n",sector_height);
  printf("sector_width: %d\n",sector_width);
*/

  CalculateSectorAverages(img, x_end, sector_height, sector_width, sector_averages);
  
/*
  printf("%d",sector_averages[0][0]); printf("%d",sector_averages[0][1]); printf("%d\n",sector_averages[0][2]);
  printf("%d",sector_averages[1][0]); printf("%d",sector_averages[1][1]); printf("%d\n",sector_averages[1][2]);
  printf("%d",sector_averages[2][0]); printf("%d",sector_averages[2][1]); printf("%d\n",sector_averages[2][2]);
*/

  safetogo = safeToGoForwards(sector_averages);
  printf("safetogo: %d\n",safetogo);
  //free(sector_averages);

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
  //printf("BLA\n");
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
void CalculateSectorAverages (struct image_t *input_img, uint16_t x_end ,uint8_t sector_h, uint8_t sector_w, uint8_t **output_array)
{
  uint8_t *source = (uint8_t *)input_img->buf;
  
  int sum = 0;
  uint8_t s = 0;

  for(uint16_t y = 0; y < input_img->h; ++y){
    //printf("%d\n",y);
    for(uint16_t x = s*sector_w ; x < x_end ; x += 1) {
      sum += source[1];//input_array[i][j];
      //printf("source[1]: %d, y: %d, x: %d\n",source[1],y,x);
      //sum += source[3];
        if(x == ((s+1)*sector_w-1)) {
        break;
        }
    }
    if((y+1)%sector_h == 0){
      //printf("sum: %d\n",sum);
      //printf("average: %d\n",sum/(sector_h*sector_w));
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
    }
  }
  source += 2;

//printf("The picture is processed for averages\n");
}

bool safeToGoForwards(uint8_t **input_array) 
{
  center = (v_sectors+1)/2; 
  margin  = (win-1)/2;
  //int front[c]; 
  int count = 0; 
  for (int i = center - margin; i < center + margin + 1; i++) {
    //cout << "i: " << i << endl; 
    //count += freeColumn(&input_array, i);
    for(int j = 0; j < sector_end; j++) {
      count += input_array[i][j];
    }
  }
  //cout << "count: " << count << endl;
  if (count < 33)
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