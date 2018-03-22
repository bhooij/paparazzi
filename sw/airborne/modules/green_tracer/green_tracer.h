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

extern uint8_t safeToGoForwards;
extern float incrementForAvoidance;
extern uint16_t trajectoryConfidence;
extern uint8_t safeToGoForwards(void);
extern void green_tracer_init(void);
extern void green_tracer_periodic(void);
extern uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters);
extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern void CalculateSectorAverages (struct image_t *input_img, int sector_h, int sector_w, int **output_array);
#endif

