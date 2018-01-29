/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * OV2640 driver.
 *
 */
#ifndef __OV2640_H__
#define __OV2640_H__
#include "sensor.h"
int ov2640_init(sensor_t *sensor);
int set_framesize(sensor_t *sensor, framesize_t framesize);
#endif // __OV2640_H__
