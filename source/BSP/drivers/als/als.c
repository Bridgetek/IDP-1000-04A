#include <stdio.h>
#include <string.h>
#include <math.h>
#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include "bsp_debug.h"
#include "bsp_hwdefs.h"
#include "i2c_utils.h"
#include "als.h"
#include "bsp_util.h"

// https://en.wikipedia.org/wiki/Lux
// https://docs.microsoft.com/windows-hardware/design/whitepapers/integrating-ambient-light-sensors-with-computers-running-windows-10-creators-update
// https://www.maximintegrated.com/en/design/technical-documents/app-notes/4/4913.html

// #define ALS_USEFLOAT	1
// #define USE_I2C_UTILS 1	// something is still broken in using i2c_utils

#define LTR308_REG_CTRL	 0
#define LTR308_REG_RATE	 4
#define LTR308_REG_GAIN	 5
#define LTR308_REG_ID	 6
#define LTR308_REG_STAT	 7
#define LTR308_REG_DATA0 13 // LSB
#define LTR308_REG_DATA1 14
#define LTR308_REG_DATA2 15 // MSB 4-bits only
#define LTR308_ENABLE	 (1 << 1)
#define LTR308_SWRESET	 (1 << 4)
#define LTR308_PART_ID	 (0xB1)

static uint16_t res_msec; // ALS ADC resolution-to-conversion time
static uint8_t	gain_val; // ALS Gain Range

#ifndef USE_I2C_UTILS
static int ltr308_read(uint8_t cmd, uint8_t *buffer, uint8_t len)
{
	int ret = i2c_write_blocking(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR, &cmd, 1, true);
	ret = i2c_read_blocking(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR, buffer, len, false);
	if (ret < 0) {
		// printf("%s(): fail\n", __func__);
	}
	return ret;
}

static int ltr308_write(uint8_t cmd, uint8_t *buffer, uint8_t len)
{
	uint8_t buf[33];
	buf[0] = cmd;
	memcpy(buf + 1, buffer, len);
	int ret = i2c_write_blocking(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR, buf, 1 + len, false);
	if (ret < 0) {
		// printf("%s(): fail\n", __func__);
	}
	return ret;
}

static int ltr308_writereg(uint8_t reg, uint8_t value)
{
	uint8_t value_to_write = value;
	return ltr308_write(reg, &value_to_write, 1);
}
#endif

int als_init()
{
	uint8_t ctrl = 0x07; // Reset CMD???
	uint8_t part_id;

#ifdef USE_I2C_UTILS
	ctrl = 0x07; // Where did this constant come from?
	if (i2c_reg_wr8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_CTRL, &ctrl, 1) < 0) {
		PR_ERROR("%s(): Unable to write-reset REG_CTRL\n", __func__);
		return -1;
	}
	sleep_ms(200);
	if (i2c_reg_rd8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_CTRL, &ctrl, 1) < 0) {
		PR_ERROR("%s(): Unable to read REG_CTRL\n", __func__);
		return -2;
	}
	if (i2c_reg_rd8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_ID, &part_id, 1) < 0 ||
		part_id != LTR308_PART_ID) {
		PR_ERROR("%s(): PartID fail: is 0x%02x, should be 0x%02x\n", __func__,
				 part_id, LTR308_PART_ID);
		return -3;
	}
#else
	int ret = ltr308_writereg(LTR308_REG_CTRL, 0x07);
	sleep_ms(200);
	ret = ltr308_read(LTR308_REG_CTRL, &ctrl, 1);
	if (ret < 0) {
		printf("%s(): Unable to read REG_CTRL\n", __func__);
		return -1;
	}
	ret = ltr308_read(LTR308_REG_ID, &part_id, 1);
	if (ret < 0 || part_id != LTR308_PART_ID) {
		printf("%s(): Unable to read PartID\n", __func__);
		// printf("  LTR-308 CTRL=0x%02x, PartID=0x%02x\n", ctrl, part_id);
		return -1;
	}
#endif

	als_setres(als_20bits);
	als_setgain(als_gain_18);
	return 0;
}

int als_setres(LTR308_ALS_RES als_res)
{
	uint16_t msec_tmp;
	bool	 not_supported = false;
	// clang-format off
	switch(als_res) {
	case als_20bits: msec_tmp = 400; break;
	case als_19bits: msec_tmp = 200; break;
	case als_18bits: msec_tmp = 100; break;
	case als_17bits: msec_tmp = 50; break;
	case als_16bits: msec_tmp = 25; break;
	default:
		not_supported = true;
		break;
	}
	// clang-format on
	if (not_supported) {
		PR_ERROR("%s(): Resolution setting error\n", __func__);
		return -1;
	}
#ifdef USE_I2C_UTILS
	uint8_t als_reg_val = (als_res & 0b111) << 4;
	if (i2c_reg_wr8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_RATE, &als_reg_val, 1) < 0) {
		PR_ERROR("%s(): Resolution write fail\n", __func__);
		return -1;
	}
#else
	int ret = ltr308_writereg(LTR308_REG_RATE, (als_res & 0b111) << 4);
	if (ret < 0) {
		PR_ERROR("%s(): Resolution write fail\n", __func__);
		return -1;
	}
#endif
	res_msec = msec_tmp;
	return 0;
}

int als_setgain(LTR308_GAIN_RANGE gain_setting)
{
	uint8_t gain_tmp;
	bool	not_supported = false;
	// clang-format off
	switch(gain_setting) {
	case als_gain_1:  gain_tmp = 1; break;
	case als_gain_3:  gain_tmp = 3; break;
	case als_gain_6:  gain_tmp = 6; break;
	case als_gain_9:  gain_tmp = 9; break;
	case als_gain_18: gain_tmp = 18; break;
	default:
		not_supported = true;
		break;
	}
	// clang-format on
	if (not_supported) {
		PR_ERROR("%s(): Gain setting error\n", __func__);
		return -1;
	}
#ifdef USE_I2C_UTILS
	uint8_t gain_val = (gain_setting & 0b111);
	if (i2c_reg_wr8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_GAIN, &gain_val, 1) < 0) {
		PR_ERROR("%s(): Gain setting write fail\n", __func__);
		return -1;
	}
#else
	int ret = ltr308_writereg(LTR308_REG_GAIN, gain_setting & 0b111);
	if (ret < 0) {
		PR_ERROR("%s(): Gain setting write fail\n", __func__);
		return -1;
	}
#endif
	gain_val = gain_tmp;
	return 0;
}

/* als_getlux()
 * Read ALS sensor, calculates and passes the lux value by reference.
 * Return value:
 * -1 = error
 *  0 = no new data
 *  1 = good and updated lux_val data
 */
int als_getlux(uint16_t *lux_val)
{
	uint8_t stat, buf[4];
	int		als_data;

#ifdef USE_I2C_UTILS
	if (i2c_reg_rd8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_STAT, &stat, 1) < 0) {
		PR_ERROR("%s(): Unable to read REG_STAT\n", __func__);
		return -1;
	}
	if ((stat & 0b1000) == 0) {
		PR_WARN("%s(): REG_STAT %d no new data\n", __func__, stat);
		return 0; // no new data
	}
	if (i2c_reg_rd8(LTR308ALS_PICO_I2C_BUS, LTR308ALS_I2C_ADDR,
					LTR308_REG_DATA0, buf, 3) < 0) {
		PR_ERROR("%s(): Unable to read REG_DATA[0..2]\n", __func__);
		return -1;
	}
#else
	int ret;
	ret = ltr308_read(LTR308_REG_STAT, &stat, 1);
	if (ret < 0)
		return -1; // return error
	if ((stat & 0b1000) == 0)
		return 0; // no new data
	ret = ltr308_read(LTR308_REG_DATA0, buf, 3);
	if (ret < 0)
		return -1;
#endif

	als_data = (buf[0] | (buf[1] << 8) | (buf[2] << 16)) & 0xFFFFF; // 20-bits

#if ALS_USEFLOAT
	const float window_factor = 1.0;
	float		lux = 0.6 * als_data * window_factor / ((float)gain_val * (res_msec / 100.0));
#else
	const int window_factor = 100; // window transmissibility percent
	int		  lux = 6LL * als_data * window_factor / (10 * gain_val * res_msec);
#endif
	*lux_val = BOUND(0, lux, 65535);
	return 1; // return new_data present
}

float getPctBrightFromLuxReading(float lux)
{
	if (lux > MAXIMUM_LUX_BREAKPOINT)
		return 1.0;
	else
		return (9.9323 * log(lux) + 27.059) / 100.0;
} // getPctBrightFromLuxReading
