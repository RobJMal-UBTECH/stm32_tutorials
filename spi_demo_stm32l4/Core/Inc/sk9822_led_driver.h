/*
 * sk9822_led_driver.h
 *
 *  Created on: Jan 11, 2022
 *      Author: robotmalate
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#ifndef INC_SK9822_LED_DRIVER_H_
#define INC_SK9822_LED_DRIVER_H_

static const uint8_t LED_START_FRAME_LEN = 4;
static const uint8_t LED_END_FRAME_LEN = 4;
static const uint8_t NUMBER_OF_LEDS = 2;

void PrepareLedBuf(uint8_t* led_buf_ptr);
void WriteToLedBuf(uint8_t* led_buf_ptr,
					uint8_t red_level,
					uint8_t green_level,
					uint8_t blue_level,
					uint8_t brightness_level);
int LinearMapping(int input_value,
					int input_start,
					int input_end,
					int output_start,
					int output_end);

#endif /* INC_SK9822_LED_DRIVER_H_ */
