/**
 * @file rgbled.h
 *
 * @brief RGB LED API
 * Adopted from PRM project
 *
 **/

#ifndef INCLUDES_BSP_RGBLED_H_
#define INCLUDES_BSP_RGBLED_H_
#include <stdint.h>
#include <stdbool.h>

#define ROYALBLUE1             0x4876FFUL
#define RED_COLOR              0xFF0000UL
#define CYAN                   0x00FFFFUL
#define ORANGE                 0xFFA500UL

/** @name Definitions for brightness intensity of the RGB LEDs
 */
//@{
/** @brief Maximum intensity of brightness */
#define LED_INTENSITY_MAX		192u //100%
/** @brief High intensity of brightness */
#define LED_INTENSITY_HIGH		153 //80%
/** @brief Medium intensity of brightness */
#define LED_INTENSITY_MEDIUM	96u //50%
/** @brief Low intensity of brightness */
#define LED_INTENSITY_LOW		48u //25%
/** @brief Zero brightness or OFF condition */
#define LED_INTENSITY_OFF		0u

#define R(c)	(uint8_t)((c & 0xFF0000) >> 16)
#define G(c)	(uint8_t)((c & 0x00FF00) >> 8)
#define B(c)	(uint8_t)((c & 0x0000FF) >> 0)

#define KTD2061_REG_ID (0x00)
#define KTD2061_REG_STATUS (0x01)
#define KTD2061_REG_CONTROL (0x02)
#define KTD2061_REG_IRED0 (0x03)
#define KTD2061_REG_IGRN0 (0x04)
#define KTD2061_REG_IBLU0 (0x05)
#define KTD2061_REG_IRED1 (0x06)
#define KTD2061_REG_IGRN1 (0x07)
#define KTD2061_REG_IBLU1 (0x08)
#define KTD2061_REG_ISELA12 (0x09)
#define KTD2061_REG_ISELA34 (0x0A)
#define KTD2061_REG_ISELB12 (0x0B)
#define KTD2061_REG_ISELB34 (0x0C)
#define KTD2061_REG_ISELC12 (0x0D)
#define KTD2061_REG_ISELC34 (0x0E)

#define LED_RED_DIGITAL_MAX	(40)
#define LED_CYAN_ARR_SIZE		(18)
#define LED_ORANGE_ARR_SIZE	(10)

/**
 *  
 *  @brief Initialise the RGB LED
 *  @return 0 for good init, else negative for error
 *   
 */
int led_init(void);

/**
 *  
 *  @brief Set the LED colour with the given intensity
 *  @param color_name The colour value to set. Friendly names are defined in colornames.h
 *  @param intensity Varies from 0 - 31
 *  @return None
 *   
 */
void led_set(uint32_t color, uint8_t intensity);


/**
 * @param type of LEDs to control. Refer LED_TYPE
 * @param color_name The colour to toggle
 */
void led_toggle(uint32_t color_on, uint32_t color_off, uint8_t intensity);

void led_strip_selection_left(void);
void led_strip_selection_right(void);
void led_strip_selection_both(void);
bool led_check_color_update(uint32_t color);
void led_set_last_color(uint32_t color);
void led_force_update(void);
void led_color_correction(uint32_t *color, uint8_t intensity);

#endif
