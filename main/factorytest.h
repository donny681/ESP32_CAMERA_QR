/*
 * mygpio.h
 *
 *  Created on: 2018年3月9日
 *      Author: ai-thinker
 */

#ifndef MAIN_FACTORYTEST_H_
#define MAIN_FACTORYTEST_H_
#include "driver/gpio.h"
#define TEST_GPIO 2

void gpio_init();
void jpeg_test(uint8_t *data,size_t size);

extern int test_mode;


#endif /* MAIN_FACTORYTEST_H_ */
