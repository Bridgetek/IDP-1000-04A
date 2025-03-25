#ifndef _ALS_H_
#define _ALS_H_

typedef enum {
	als_gain_1 = 0,
	als_gain_3,
	als_gain_6,
	als_gain_9,
	als_gain_18,
} LTR308_GAIN_RANGE;

typedef enum {
    als_20bits = 0,
    als_19bits,
    als_18bits,
    als_17bits,
    als_16bits,
} LTR308_ALS_RES;

int als_init();
int als_setres(LTR308_ALS_RES als_res);
int als_setgain(LTR308_GAIN_RANGE gain_setting);
int als_getlux(uint16_t *lux_val);

// the point at which the function clips to 100%
#define MAXIMUM_LUX_BREAKPOINT 1254.0
float getPctBrightFromLuxReading(float lux);

#endif // _ALS_H_