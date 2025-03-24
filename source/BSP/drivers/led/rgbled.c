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

// clang-format off
static uint8_t cyan_gb_code[LED_CYAN_ARR_SIZE][2] = {
	{2, 1}, {3, 2}, {4, 3}, {6, 4},
	{9, 5}, {11, 6}, {14, 7}, {17, 8},
	{20, 9}, {22, 10}, {24, 11}, {26, 12},
	{28, 13}, {30, 14}, {32, 15}, {35, 16},
	{38, 17}, {40, 18}
};

static uint8_t orange_rg_code[LED_ORANGE_ARR_SIZE][2] = { {6, 1}, {12, 2}, {18, 3}, {26, 4},
									{32, 5}, {38, 6}, {44, 7}, {50, 8},
									{56, 9}, {62, 10} };
// clang-format on

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

void led_color_correction(uint32_t *color, uint8_t intensity)
{
	uint32_t adjust_value;
	switch (*color) {
	case RED_COLOR:
		adjust_value = (intensity * LED_RED_DIGITAL_MAX) / 100;
		if (intensity != 0 && adjust_value == 0) {
			adjust_value = 1;
		}
		*color = adjust_value << 16;
		break;
	case CYAN:
		if (intensity != 0) {
			adjust_value = (intensity * (LED_CYAN_ARR_SIZE - 1) / 100);
			*color = (cyan_gb_code[adjust_value][0] << 8) | cyan_gb_code[adjust_value][1];
		}
		else {
			*color = 0;
		}
		break;
	case ORANGE:
		if (intensity != 0) {
			adjust_value = (intensity * (LED_ORANGE_ARR_SIZE - 1) / 100);
			*color = (orange_rg_code[adjust_value][0] << 16) | (orange_rg_code[adjust_value][1] << 8);
		}
		else {
			*color = 0;
		}
		break;
	default:
		break;
	}
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
#ifndef MSVC_FT800EMU
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
#endif // MSVC_FT800EMU
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
	// led_color_correction(&color, intensity, false);

	value[0] = KTD2061_REG_IRED0;
	value[1] = R(color);
	value[2] = G(color);
	value[3] = B(color);
	i2c_write_blocking(RGB_LED_I2C_CHANNEL, RGB_LED_KTD2061_ADDRESS, value, 4, false);
}
