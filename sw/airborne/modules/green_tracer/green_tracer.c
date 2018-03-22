/*
 * Copyright (C) Beau Smit
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_tracer/green_tracer.c"
 * @author Beau Smit
 */

#include "modules/green_tracer/green_tracer.h"
#include "modules/computer_vision/colorfilter_green.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define GREEN_TRACER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[green_tracer->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if GREEN_TRACER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef GREEN_TRACER_LUM_MIN
#define GREEN_TRACER_LUM_MIN 41
#endif

#ifndef GREEN_TRACER_LUM_MAX
#define GREEN_TRACER_LUM_MAX 183
#endif

#ifndef GREEN_TRACER_CB_MIN
#define GREEN_TRACER_CB_MIN 53
#endif

#ifndef GREEN_TRACER_CB_MAX
#define GREEN_TRACER_CB_MAX 121
#endif

#ifndef GREEN_TRACER_CR_MIN
#define GREEN_TRACER_CR_MIN 134
#endif

#ifndef GREEN_TRACER_CR_MAX
#define GREEN_TRACER_CR_MAX 249
#endif


uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.50 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;

//

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void green_tracer_init()
{
  // Initialise the variables of the colorfilter to accept orange
  color_lum_min = GREEN_TRACER_LUM_MIN;
  color_lum_max = GREEN_TRACER_LUM_MAX;
  color_cb_min  = GREEN_TRACER_CB_MIN;
  color_cb_max  = GREEN_TRACER_CB_MAX;
  color_cr_min  = GREEN_TRACER_CR_MIN;
  color_cr_max  = GREEN_TRACER_CR_MAX;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void green_tracer_periodic()
{
  /* 
   * make pointer array which will be filled with the averages of the sectors 
   */
  sector_averages = new int *[sector_height];
  for(int j = 0; j<v_sectors; j++){
    sector_averages[j] = new int[h_sectors];
  }

  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  safeToGoForwards = color_count > tresholdColorCount;
  VERBOSE_PRINT("Color_count: %d  threshold: %d safe: %d \n", color_count, tresholdColorCount, safeToGoForwards);
  float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
  if (safeToGoForwards) {
    moveWaypointForward(WP_GOAL, moveDistance);
    moveWaypointForward(WP_TRAJECTORY, 1.00 * moveDistance);
    nav_set_heading_towards_waypoint(WP_GOAL);
    chooseRandomIncrementAvoidance();
    trajectoryConfidence += 1;
  } else {
    waypoint_set_here_2d(WP_GOAL);
    waypoint_set_here_2d(WP_TRAJECTORY);
    increase_nav_heading(&nav_heading, incrementForAvoidance);
    /*
    find_avoidance_increment();
    */
    if (trajectoryConfidence > 5) {
      trajectoryConfidence -= 4;
    } else {
      trajectoryConfidence = 1;
    }
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,	
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
                DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 30.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -30.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}

/*
 * This piece of code selectes certain parts of the array and averages the values.
 * These values are then put in an array/list which can be used for control.
 */
void CalculateSectorAverages (struct image_t *input_img, int sector_h, int sector_w, int **output_array)
{
  uint8_t image_width = input_img->w;
  uint8_t image_height = input_img->h;
  uint8_t *source = (uint8_t *)input_img->buf;

  int sum = 0;
  int s = 0;

  for(int y = 0; y < input_img->h; y++){
    for(int x = s*sector_w ; x < input_img->w ; x++) {
      sum += source[0]//input_array[i][j];
        if(j == ((s+1)*sector_w-1)) {
        break;
        }
    }
    if((i+1)%sector_h == 0){
      
      if (sum/(sector_h*sector_w) > 130)
      {
        output_array[(i+1)/sector_h-1][s] = 1;
      }
      else{
        output_array[(i+1)/sector_h-1][s] = 0;
      }
      //output_array[(i+1)/sector_h-1][s] = sum/(sector_h*sector_w);
      sum = 0;
    }
    if(i == image_height-1 && s < (image_width/sector_w - 1)) {
      i = -1;
      s += 1;
    }
  }
}