/*
 * bsp_app.h
 *
 *  Created on: 11 Dec 2019
 *      Author: prabhakaran.d
 */

#ifndef _BSP_APP_H_
#define _BSP_APP_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	op_normal = 0,
	op_no_comms,
	op_identify,
	op_configuring,
	op_incompatible_hub,
	op_rebooting,
} BSP_OPSTATES;

typedef struct {
	bool	 thresholded;
	uint8_t	 lcd_percent_low;
	uint8_t	 lcd_percent_high;
	uint16_t lux_threshold;
	bool	 is_log10;
	float	 log_constant;
	float	 log_multiplier;
	int		 lux_breakpoint;
} ALSconfig_t;

typedef struct {
	int8_t tz_min;
	int8_t tz_hour;
	/* HW device status = Peripherals/Sensors Context */
	uint32_t	dev_badflags;
	uint8_t		rotarysw_id;
	float		dev_temperature;
	int16_t		dev_tof_distance;
	uint16_t	dev_als_lux;
	uint8_t		dev_auto_pwm;
	ALSconfig_t als_config;

	BSP_OPSTATES op_state;

} BSPContexts_t;

extern BSPContexts_t*dev_contexts_gptr;

#endif /* _BSP_APP_H_ */
