#ifndef _ALS_H_
#define _ALS_H_

#define LTR308_GAIN_1  (0)
#define LTR308_GAIN_3  (1)
#define LTR308_GAIN_6  (2)
#define LTR308_GAIN_9  (3)
#define LTR308_GAIN_18 (4)

#define LTR308_RATE_25MS   (0)
#define LTR308_RATE_50MS   (1)
#define LTR308_RATE_100MS  (2)
#define LTR308_RATE_500MS  (3)
#define LTR308_RATE_1000MS (4)
#define LTR308_RATE_2000MS (5)

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

typedef enum {
	als_rate_25ms = 0,
	als_rate_50ms,
	als_rate_100ms,
	als_rate_500ms,
	als_rate_1000ms,
	als_rate_2000ms,
} LTR308_MEAS_RATE;

int als_init();
int als_setres(LTR308_ALS_RES als_res);
int als_setgain(LTR308_GAIN_RANGE gain_setting);
int als_getlux(uint16_t *lux_val);

// the point at which the function clips to 100%
#define MAXIMUM_LUX_BREAKPOINT 1254.0
float getPctBrightFromLuxReading(float lux);

#endif // _ALS_H_