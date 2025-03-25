/**
 * @file rgbled.c
 *
 * @brief RGB LED API
 * Adopted from PRM project
 *
 **/

#include "bsp_hwdefs.h"
#include "bsp_debug.h"
#include "rgbled.h"
#define RGB_LED_KTD2061_ADDRESS KTD2061_I2C_ADDR
#define RGB_LED_I2C_CHANNEL KTD2061_PICO_I2C_BUS

static uint8_t	g_arr_toggle;
static uint32_t g_last_color;
static bool		g_force_update;

static void bsp_set(uint32_t color, uint8_t intensity);

inline bool led_check_color_update(uint32_t color)
{
	return ((g_last_color != color) || g_force_update);
}

inline void led_set_last_color(uint32_t color)
{
	g_last_color = color;
}
void led_force_update(void)
{
	g_force_update = true;
}

void led_strip_selection_left(void)
{
	uint8_t value[7] = {KTD2061_REG_ISELA12, 0, 0, 0, 0x88, 0x88, 0x88};
	i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, sizeof(value), false);
}
void led_strip_selection_right(void)
{
	uint8_t value[7] = {KTD2061_REG_ISELA12, 0x88, 0x88, 0x88, 0, 0, 0};
	i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, sizeof(value), false);
}
void led_strip_selection_both(void)
{
	uint8_t value[7] = {KTD2061_REG_ISELA12, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88};
	i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, sizeof(value), false);
}

/**
 *
 *  @brief Intialize the RGB LED
 *  @return None
 *
 */
int led_init(void)
{
	// Initialize KTD2061 LED controller:
	// Normal Mode (b'10)
	// BrightExtend Enable (b'1)
	// CoolExtend Temperature Setting 90*C (b'11)
	// Fade Rate Setting 31ms (b'000)
	uint8_t value[7];
	value[0] = KTD2061_REG_CONTROL;
	value[1] = 0xB8; // 1011 1000

	int ret = i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, 2, false);
	return ret < 0 ? ret : 0;
}

/**
 *
 *  @brief Set the LED Color
 *  @param val color value to set
 *  @return None
 *
 */
void led_set(uint32_t color_name, uint8_t intensity)
{
	int ret = PICO_OK;
	if (led_check_color_update(color_name)) {
		led_set_last_color(color_name);
		bsp_set(color_name, intensity);
	}
	if (ret == PICO_OK) {
		g_force_update = false;
	}
	else {
		PR_ERROR("%s(): line = %d, error code = %d\n", __func__, __LINE__, ret);
	}
}

void led_toggle(uint32_t color_on, uint32_t color_off, uint8_t intensity)
{
	int ret = PICO_OK;
	if (g_arr_toggle == 0) {
		g_arr_toggle = 1;
		if (led_check_color_update(color_on)) {
			led_set_last_color(color_on);
			bsp_set(color_on, intensity);
		}
	}
	else {
		g_arr_toggle = 0;
		if (led_check_color_update(color_off)) {
			led_set_last_color(color_off);
			bsp_set(color_off, intensity);
		}
	}
	if (ret == PICO_OK) {
		g_force_update = false;
	}
	else {
		PR_ERROR("%s(): line = %d, error code = %d\n", __func__, __LINE__, ret);
	}
}

static void bsp_set(uint32_t color, uint8_t intensity)
{
	uint8_t value[4];

	value[0] = KTD2061_REG_IRED0;
	value[1] = R(color);
	value[2] = G(color);
	value[3] = B(color);
	i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, 4, false);
}
