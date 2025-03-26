/*
 * bsp_test.c
 */

#include <pico.h> // incs stdint,stdbool,stddefs,datetime_t
#include <ctype.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <hardware/adc.h>
#include <pico/stdlib.h>
#include <hardware/watchdog.h>
#include <hardware/rtc.h>

/* Drivers */
#include "i2c_utils.h"
#include "efm8_mio.h"
#include "als.h"
#include "rgbled.h"
#include "tof.h"
#include "sdcard.h"
#include "eve_app.h"

/* sys utils */
#include "cli.h"

#include "bsp_hwdefs.h"
#include "bsp_debug.h"
#include "bsp_app.h"
#include "bsp_util.h"

BSPContexts_t  dev_contexts;
BSPContexts_t* dev_contexts_gptr = &dev_contexts;

#define SW_BUILDDATE_STR __DATE__
#define SW_BUILDTIME_STR __TIME__
#define SW_TIMESTAMP_STR __DATE__ " " __TIME__
#define APP_TITLE_STR	 KGRN1 "IDP-1000-04A Application"

#define ENABLE_WATCHDOG 1
#define ENABLE_ALS		1
#define ENABLE_TOF		1
#define ENABLE_LED		1
#define ENABLE_SD       1
#define ENABLE_EVE      1

// Debugging flags & settings
int dbg_fps_msec = 0;

static uint32_t led_prev_time = 0;

int init_bsp(void);
int do_bsp_tasks(void);
int main()
{
	init_bsp();
	led_prev_time = systime_msec();

	while (true) {
		do_bsp_tasks();
		if (msec_expired(led_prev_time, 1000)) {
			led_prev_time = systime_msec();
		}
	}
}

int do_bsp_tasks()
{
	BSPContexts_t*ctx = &dev_contexts;

	if (ctx->op_state != op_rebooting) {
		watchdog_update();
		do_nonblock_cli();
	}

	if ((ctx->dev_badflags & (1 << dvr_als)) == 0) {
#ifdef ENABLE_ALS
		als_getlux(&ctx->dev_als_lux);
		float auto_percent = getPctBrightFromLuxReading(ctx->dev_als_lux);

		uint8_t tmp_percent = (uint8_t)BOUND(0, (auto_percent * 100), 100);
		uint8_t reg_pwm_val = (uint8_t)map(tmp_percent, 0, 100,
										   LCD_BRIGHTNESS_PWM_MIN,
										   LCD_BRIGHTNESS_PWM_MAX);
		ctx->dev_auto_pwm = reg_pwm_val;
#endif /* ENABLE_ALS */
	}
	else {
		ctx->dev_als_lux = 0;
		ctx->dev_auto_pwm = 0x20;
	}

#ifdef ENABLE_TOF
	if ((ctx->dev_badflags & (1 << dvr_tof)) == 0) {
		if (ctx->op_state != op_rebooting) {
			ctx->dev_tof_distance = tof_get_mm();
		}
		else {
			tof_stop();
		}
	}
	else {
		ctx->dev_tof_distance = -1;
	}
#endif /* ENABLE_TOF */

	uint16_t adc_raw = adc_read();
	float	 conversion_factor = 3.3f / (1 << 12);
	float	 adc_volts = (float)adc_raw * conversion_factor;
	float	 cpu_temp = 27.0f - (adc_volts - 0.706f) / 0.001721f;
	ctx->dev_temperature = cpu_temp;

	return ctx->op_state;
}

static void reset_external_devices()
{
	gpio_init(I2C_BUS0_PIN_RESET);
	gpio_set_dir(I2C_BUS0_PIN_RESET, GPIO_OUT);
	gpio_put(I2C_BUS0_PIN_RESET, 0);
	sleep_ms(2);
	gpio_put(I2C_BUS0_PIN_RESET, 1);
	sleep_ms(10);
}

int init_bsp(void)
{
	__unused int res;

	dev_contexts_gptr = &dev_contexts;
	BSPContexts_t*ctx = dev_contexts_gptr;
	/* clear and setup defaults for BSP context state */
	memset(ctx, 0, sizeof(BSPContexts_t));

	ctx->op_state = op_no_comms;

	reset_external_devices();

	stdio_init_all();
	sleep_ms(1500);
	printf("\n\n\n%s" KNRM "\n", APP_TITLE_STR);
	printf("Build date: %s, %s\n", SW_BUILDDATE_STR, SW_BUILDTIME_STR);
	printf("PICO_BOARD = %s\n", PICO_BOARD);
	printf("gcc __version__ = %s\n", __VERSION__);
	printf("pico_sdk_version_string = %s\n", PICO_SDK_VERSION_STRING);
	// https://forums.raspberrypi.com/viewtopic.php?t=300003
	extern char __flash_binary_start;
	extern char __flash_binary_end;
	uintptr_t	flash_begin = (uintptr_t)&__flash_binary_start;
	uintptr_t	flash_end = (uintptr_t)&__flash_binary_end;
	printf("Application binary 0x%08x..0x%08x len = %d\n",
		   flash_begin, flash_end, flash_end - flash_begin);

	// Start the RTC
	datetime_t t = {
			.year = 2025,
			.month = 01,
			.day = 01,
			.dotw = 3, // 0 is Sunday, so 5 is Friday
			.hour = 00,
			.min = 00,
			.sec = 00
	};
	rtc_init();
	rtc_set_datetime(&t);
	if (rtc_running()) {
		PR_INFO("RTC initialised\n");
	}
	else {
		ctx->dev_badflags |= (1 << dvr_rtc);
		PR_ERROR("RTC_IS_NOT_RUNNING\n");
	}

#ifdef ENABLE_WATCHDOG
	// Start the Watchdog. By default the SDK assumes a 12MHz XOSC.
	// watchdog_start_tick(12);
	watchdog_enable(WATCHDOG_TIMEOUT, 1);
	PR_INFO("WATCHDOG initialised\n");
#endif /* ENABLE_WATCHDOG */

	adc_init();
	adc_set_temp_sensor_enabled(true);
	adc_select_input(4);
	PR_INFO("CPU Temp initialised\n");

	i2c_pins_init();
	PR_INFO("I2C Busses initialised\n");

	int id = 0;
	if (efm8_init() < 0) {
		PR_ERROR("EFM8 init fail\n");
		ctx->dev_badflags |= (1 << dvr_mio);
	}
	else {
		id = efm8_get_rotary();
		id = BOUND(0, id, 9);
		PR_INFO("EFM8 Rotary switch = %d\n", id);
	}
	ctx->rotarysw_id = (uint8_t)id;

#if ENABLE_ALS
	if (als_init()) {
		ctx->dev_badflags |= (1 << dvr_als);
		PR_ERROR("ALS init fail\n");
	}
	else {
		PR_INFO("ALS initialised\n");
	}
#endif /* ENABLE_ALS */

#if ENABLE_TOF
	if (tof_init()) {
		ctx->dev_badflags |= (1 << dvr_tof);
		PR_ERROR("TOF init fail\n");
	}
	else {
		PR_INFO("TOF initialised\n");
	}
#endif /* ENABLE_TOF */

#if ENABLE_LED
	if (led_init() < 0) {
		ctx->dev_badflags |= (1 << dvr_led);
		PR_ERROR("LED init fail\n");
	}
	else {
		led_strip_selection_both();
		led_set(ROYALBLUE1, LED_INTENSITY_MAX);
		PR_INFO("LED initialised\n");
	}
#endif /* ENABLE_LED */

	watchdog_update();

#ifdef ENABLE_SD
	if (!loadSdCard()) {
		ctx->dev_badflags |= (1 << dvr_fatfs);
		PR_WARN("FATFS mount fail\n");
	}
	else {
		PR_INFO("FATFS mounted\n");
	}
#endif /* ENABLE_SD */

#ifdef ENABLE_EVE
	if (!Gpu_Init())
	{
		ctx->dev_badflags |= (1 << dvr_eve);
		PR_WARN("EVE initial fail\n");
	}
	else {
		PR_INFO("EVE initialised\n");
	}

	watchdog_update();
#endif /* ENABLE_EVE */

	return ctx->op_state;
}
