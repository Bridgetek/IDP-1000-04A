/*
 * kinetic_rgb_led.c
 *
 *  Created on: 27 Sep 2021
 *      Author: prabhakaran.d
 */
#include <stdio.h>
#include <string.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>

#include "hardware/i2c.h"
#include "kinetic_rgb_led.h"
#include "kinetic_rgb_led_registers.h"

#include "bsp_debug.h"
#include "bsp_hwdefs.h"
#include "i2c_utils.h"

#define I2CBUS KTD2061_PICO_I2C_BUS

#define LED_SET(enable, color) (enable | color)

static bool _update = false;

static void i2c_write(uint8_t cmd, uint8_t *buffer, uint8_t len)
{
	uint8_t buf[33];
	buf[0] = cmd;
	memcpy(buf + 1, buffer, len);
	i2c_write_blocking(I2CBUS, KTD_ADDRESS, buf, 1 + len, false);
}

int kinetic_rgb_led_init(uint8_t *led, uint8_t len)
{
	while (len--)
		*led++ = 0;
	len = ENABLE_RESET;
	// i2c_write(KTD_CONTROL, &len, 1);

	uint8_t idbuf[2];
	if (i2c_reg_rd8(I2CBUS, KTD2061_I2C_ADDR,
					KTD_ID, idbuf, 2) < 0) {
		PR_ERROR("%s(): Unable to read reg_ID/Mon\n", __func__);
		return -1;
	}
	// printf("%s(): id_reg=0x%02x, mon_reg=0x%02x\n", __func__, idbuf[0], idbuf[1]);
	if (idbuf[0] != KTD_ID_DIE_VAL || (idbuf[1] & 0xF0) != KTD_DIE_VSN) {
		PR_ERROR("%s(): PartID fail: is 0x%02x%02x, should be 0x%02x%02x\n",
				 __func__, idbuf[0], idbuf[1], KTD_ID_DIE_VAL, KTD_ID_DIE_VAL);
		return -1;
	}
	uint8_t reset_cmd = ENABLE_RESET;
	if (i2c_reg_wr8(I2CBUS, KTD2061_I2C_ADDR,
					KTD_CONTROL, &reset_cmd, 2) < 1) {
		PR_ERROR("%s(): Unable to write reset\n", __func__);
		return -1;
	}
	return 0;
}

void kinetic_rgb_led_fill_color(KineticColorBank_e rbank, uint8_t color)
{
	uint8_t buffer[3];
	buffer[0] = color;
	buffer[1] = color;
	buffer[2] = color;
	i2c_write(rbank, buffer, 3);
}

void kinetic_rgb_led_brightness(char key)
{
	static uint8_t brightness = 0xc0;
	uint8_t		   buffer[3];
	if (brightness > 0x0 && key == '-') {
		brightness -= 5;
	}
	else if (brightness < 0xc0 && key == '+') {
		brightness += 5;
	}
	printf("LED Brightness %d\r\n", brightness);
	buffer[0] = brightness;
	buffer[1] = brightness;
	buffer[2] = brightness;
	i2c_write(color_reg_bank_0, buffer, 3);
}

void kinetic_rgb_led_settings(KineticMode_e mode, KineticFadeRate_e faderate)
{
	uint8_t regval = mode | faderate | COOL_EXTEND_FOR_90C | BRIGHT_EXTEND_ENABLE;
	i2c_write(KTD_CONTROL, &regval, 1);
}

void kinetic_rgb_led_update(uint8_t *led, uint8_t ledno, uint8_t color,
							KineticState_e onoff)
{
	uint8_t ledReg = ledno >> 1;		  // 4 cycLE
	uint8_t ledBitLoc = (ledno % 2) << 2; // <12 CYCLE
	if (led == NULL)
		return;
	led[ledReg] &= ~(0x0F << ledBitLoc);				 // TURN OFF  12 CYV
	led[ledReg] |= (LED_SET(onoff, color) << ledBitLoc); // TURNOff
	_update = 1;
}

void kinetic_rgb_led_update_leds(uint8_t *led, uint8_t color, KineticState_e onoff)
{
	uint8_t i = 0;
	for (i = 0; i < MAX_RGB_LEDS; i++)
		kinetic_rgb_led_update(led, i, color, onoff);
}

void kinetic_rgb_led_task(uint8_t *led)
{
	if (_update) {
		i2c_write(KTD_ISELA12_CONFIG, led, 6);
		_update = false;
	}
}

int kinetic_rgb_led_exit()
{
	return 0;
}

uint8_t _led[12];

int rgb_led_init()
{
	kinetic_rgb_led_init(_led, 12);
	kinetic_rgb_led_fill_color(color_reg_bank_0, 0x28);
	kinetic_rgb_led_fill_color(color_reg_bank_1, 0);
	kinetic_rgb_led_settings(normal_mode, fade_rate_31ms);
	kinetic_rgb_led_update_leds(_led, 0x5, led_off);
	kinetic_rgb_led_task(_led);
	return 0;
}
