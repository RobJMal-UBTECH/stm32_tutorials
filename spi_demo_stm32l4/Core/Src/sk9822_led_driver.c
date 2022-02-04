/*
 * sk9822_led_driver.c
 *
 *  Created on: Jan 11, 2022
 *      Author: robotmalate
 */

#include <sk9822_led_driver.h>

/*
 * Prepares LED buffer.
 */
void PrepareLedBuf(uint8_t* led_buf_ptr)
{
	// Prepares the start frame
	for (int i = 0; i < LED_START_FRAME_LEN; i++)
	{
		led_buf_ptr[i] = 0x00;
	}

	// Preparing the end frame
	for (int i = LED_START_FRAME_LEN + 4*NUMBER_OF_LEDS;
			i < LED_START_FRAME_LEN + 4*NUMBER_OF_LEDS + LED_END_FRAME_LEN; i++)
	{
		led_buf_ptr[i] = 0x00;
	}
}

/*
 * Writes the LED information to the led_buf
 * based on SK9822 documentation.
 */
void WriteToLedBuf(uint8_t* led_buf_ptr,
					uint8_t red_level,
					uint8_t green_level,
					uint8_t blue_level,
					uint8_t brightness_level)
{
	for (int i = 0; i < NUMBER_OF_LEDS; i++)
	{
		led_buf_ptr[LED_START_FRAME_LEN*(i+1)] = (0b11100000 | brightness_level);	// brightness buffer
		led_buf_ptr[LED_START_FRAME_LEN*(i+1) + 1] = blue_level;		// blue level
		led_buf_ptr[LED_START_FRAME_LEN*(i+1) + 2] = green_level;		// green level
		led_buf_ptr[LED_START_FRAME_LEN*(i+1) + 3] = red_level;			// red level
	}
}


