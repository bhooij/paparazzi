/*
 * Copyright (C) Beau Smit
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_tracer/green_tracer.h"
 * @author Beau Smit
 */

#ifndef GREEN_TRACER_H
#define GREEN_TRACER_H
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"
#include "colorfilter_green.h"
#include "cv.h"

extern uint8_t safetogoforwards;
extern float incrementForAvoidance;
extern float trajectoryConfidence;
//extern uint8_t safeToGoForwards(void);
extern void green_tracer_init(void);
extern void green_tracer_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
//extern void CalculateSectorAverages (struct image_t *input_img, uint8_t sector_h, uint8_t sector_w, uint8_t **output_array);
#endif

