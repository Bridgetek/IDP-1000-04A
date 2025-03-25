/*
 * kinetic_rgb_led.h
 *
 *  Created on: 27 Sep 2021
 *      Author: prabhakaran.d
 */

#ifndef LDS_KINETIC_RGB_LED_KINETIC_RGB_LED_H_
#define LDS_KINETIC_RGB_LED_KINETIC_RGB_LED_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_RGB_LEDS 12

typedef enum {
	color_reg_bank_0 = 0x03,
	color_reg_bank_1 = 0x06,
} KineticColorBank_e;

typedef enum {
	fade_rate_31ms,
	fade_rate_63ms,
	fade_rate_125ms,
	fade_rate_250ms,
	fade_rate_500ms,
	fade_rate_1s,
	fade_rate_2s,
	fade_rate_4s,
} KineticFadeRate_e;

typedef enum {
	led_on = 0x08,
	led_off = 0x00,
} KineticState_e;

typedef enum {
	global_shutdown = 0x00,
	night_mode = 0x40,
	normal_mode = 0x80,
	default_settings = 0xC0,
} KineticMode_e;

int	 kinetic_rgb_led_init(uint8_t *led, uint8_t len);
void kinetic_rgb_led_fill_color(KineticColorBank_e rbank, uint8_t color);
void kinetic_rgb_led_settings(KineticMode_e mode, KineticFadeRate_e faderate);
void kinetic_rgb_led_update(uint8_t *led, uint8_t ledno, uint8_t color, KineticState_e onoff);
void kinetic_rgb_led_update_leds(uint8_t *led, uint8_t color, KineticState_e onoff);
void kinetic_rgb_led_task(uint8_t *led);
int	 kinetic_rgb_led_exit();
int	 rgb_led_init();

#endif /* LDS_KINETIC_RGB_LED_KINETIC_RGB_LED_H_ */
