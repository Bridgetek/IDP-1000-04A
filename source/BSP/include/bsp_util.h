#ifndef _PD_H_
#define _PD_H_

#include <limits.h>
#include <time.h>
#include <pico.h>
#include <pico/time.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof(array[0]))
/* to deprecate powers-of-2 rounding -- it's confusing */
// #define ROUND_UP_TO_MULTIPLE_OF(multiple_of, x)
//		(((x) + (multiple_of) - 1) & (-(multiple_of)))
#define UDIV_UP(a, b) (((a) + (b)-1) / (b))
#define ALIGN_UP(multiple_of, a) \
	(UDIV_UP(a, multiple_of) * (multiple_of))
#define SIGN(val) ((0 < val) - (val < 0))
#ifndef MIN
#define MIN(A, B) ({ __typeof__(A) __a = (A); \
		__typeof__(B) __b = (B); __a < __b ? __a : __b; })
#endif
#ifndef MAX
#define MAX(A, B) ({ __typeof__(A) __a = (A); \
		__typeof__(B) __b = (B); __a < __b ? __b : __a; })
#endif
#define BOUND(low, x, high) ({\
		__typeof__(x) __x = (x); \
		__typeof__(low) __low = (low); \
		__typeof__(high) __high = (high); \
		__x > __high ? __high : (__x < __low ? __low : __x); })
#define MAP(x, in_lo, in_hi, out_lo, out_hi) \
	(((x)-in_lo) * (out_hi - out_lo) / (in_hi - in_lo) + out_lo)

inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define systime_msec()					to_ms_since_boot(get_absolute_time())
#define msec_expired(PREV_MSEC, PERIOD) (((systime_msec() - (PREV_MSEC)) & UINT_MAX) > (PERIOD))

#endif /* _PD_H_ */
