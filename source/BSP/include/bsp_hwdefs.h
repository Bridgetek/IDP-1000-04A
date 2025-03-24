/*
 * pd40.h
 * File to describe the MCU pinouts, components and drivers used within the
 * PD40 Display Panel hardware
 *
 */
#ifndef _PD_HWDEFS_H_
#define _PD_HWDEFS_H_

#include <hardware/i2c.h>
#include <hardware/spi.h>
#include <hardware/uart.h>
#include <hardware/gpio.h>

#ifdef PATH_MAX
#undef PATH_MAX
#endif
#define PATH_MAX	  255
#define PD_PATH_MAX	  255
#define OPENFILES_MAX 8

// clang-format off

#define HWVSN_STR				"B.1" // For e.g. "B.1" where B is the major version and 1 is the minor version.
#define WATCHDOG_TIMEOUT    3000    // msecs

#define RP2040_FLASH_MX25L6433	1
#define PD_FLASH_SIZE			(8 * 1048576) // based on MX25L6433F
#define PD_LFS_SIZE				(4 * 1048576)

#define LCD_WIDTH				1280
#define LCD_HEIGHT				800
#define LCD_BRIGHTNESS_PWM_MIN	12		// out of 128
#define LCD_BRIGHTNESS_PWM_MAX	96		// out of 128
#define LCD_BRIGHTNESS_DEFAULT	80		// percent within above min-max
#define BT81X_ENABLE
#define BT817_ENABLE
#define BT_FLASH_SIZE			(64 * 1048576)
#define BT_FLASH_PAGE_SIZE		4096

#define I2C_BUS0_PIN_SDA		20
#define I2C_BUS0_PIN_SCL		21
#define I2C_BUS0_SPEED			(400*1000)  // EFM8, LTR308, Vl53L3CX
#define I2C_BUS0_PIN_RESET		23
#define I2C_BUS1_PIN_SDA		18
#define I2C_BUS1_PIN_SCL		19
#define I2C_BUS1_SPEED			(400*1000)  // KM8731-CODEC, KTD2061-LED 

#define EFM8_PICO_I2C_BUS		i2c0
#define EFM8_I2C_ADDR			0x58
#define EFM8_IRQN_PICO_PIN		17			// from U20 EFM8 GPIO P0.6

#define LTR308ALS_PICO_I2C_BUS	i2c0
#define LTR308ALS_I2C_ADDR		0x53

#define VL53L3CX_PICO_I2C_BUS	i2c0
#define VL53L3CX_I2C_ADDR		0x29
#define VL53L3CX_RST_PICO_PIN  	23	// TO DEPRECATE: see  I2C_BUS0_PIN_RESET
#define VL53L3CX_IRQ_PICO_PIN 	29

#define WM8731_PICO_I2C_BUS		i2c1
#define WM8731_I2C_ADDR			0x1A

#define KTD2061_PICO_I2C_BUS	i2c1
#define KTD2061_I2C_ADDR		0x68

#define SD_SPI					spi1    // PD100-ESD uses spi1 for sdcard
#define SD_SCK					10
#define SD_MOSI					11
#define SD_MISO					12
#define SD_CS					13
#define SD_DETECT				16
#define SD_SPI_DMA_IRQ          0

#define I2S_DIN_PIN             22
#define I2S_DOUT_PIN            26
#define I2S_BCLK_PIN            27
#define I2S_LRCLK_PIN           28
#define PICO_AUDIO_I2S_DATA_PIN         I2S_DOUT_PIN
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE   I2S_BCLK_PIN
#define PICO_AUDIO_DMA_IRQ      1   // grr! PD100-ESD-SDcard is hardcoded for 0

/* Additional EVE GPU GPIO constants */
#define EVE_FCC_AUDIOMODE_PWM	0x4
#define EVE_GPIO_SPKRBUFOE_N	2
#define EVE_GPIO_SPKRSW			3
/* Eve GPIO_SPKRSW path bit definition */
#define EVE_SPKRSW_TO_CODEC		0
#define EVE_SPKRSW_TO_BUZZER	1

/* Additional constants for LCD auto-brightness from ALS */
#define ALS_DEF_THRESHOLDED	   false // --> analog-style log-brightness
#define ALS_DEF_THRESHOLD_LUX  200
#define ALS_DEF_LCD_BRIGHT_LO  20	// percent within REG_PWM min-max
#define ALS_DEF_LCD_BRIGHT_HI  80	// percent within REG_PWM min-max
#define ALS_DEF_USE_LOG10	   false
#define ALS_DEF_LOG_MULTIPLIER 7.0	  // 9.9323
#define ALS_DEF_LOG_CONSTANT   5.0	  // was 27.059
#define ALS_DEF_LUX_BREAKPOINT 1254.0 // max lux when LCD at max brightness

#define HWVSN_STR				"B.1" // For e.g. "B.1" where B is the major version and 1 is the minor version.
#define WATCHDOG_TIMEOUT    3000    // msecs

#define RP2040_FLASH_MX25L6433	1
#define FLASH_SIZE			(8 * 1048576) // based on MX25L6433F
#define LFS_SIZE				(4 * 1048576)

#define LCD_WIDTH				1280
#define LCD_HEIGHT				800
#define LCD_BRIGHTNESS_PWM_MIN	12		// out of 128
#define LCD_BRIGHTNESS_PWM_MAX	96		// out of 128
#define LCD_BRIGHTNESS_DEFAULT	80		// percent within above min-max
#define BT81X_ENABLE
#define BT817_ENABLE
#define BT_FLASH_SIZE			(64 * 1048576)
#define BT_FLASH_PAGE_SIZE		4096

#define I2C_BUS0_PIN_SDA		20
#define I2C_BUS0_PIN_SCL		21
#define I2C_BUS0_SPEED			(400*1000)  // EFM8, LTR308, Vl53L3CX
#define I2C_BUS0_PIN_RESET		23
#define I2C_BUS1_PIN_SDA		18
#define I2C_BUS1_PIN_SCL		19
#define I2C_BUS1_SPEED			(400*1000)  // KM8731-CODEC, KTD2061-LED 

#define EFM8_PICO_I2C_BUS		i2c0
#define EFM8_I2C_ADDR			0x58
#define EFM8_IRQN_PICO_PIN		17			// from U20 EFM8 GPIO P0.6

#define LTR308ALS_PICO_I2C_BUS	i2c0
#define LTR308ALS_I2C_ADDR		0x53

#define VL53L3CX_PICO_I2C_BUS	i2c0
#define VL53L3CX_I2C_ADDR		0x29
#define VL53L3CX_RST_PICO_PIN  	23	// TO DEPRECATE: see  I2C_BUS0_PIN_RESET
#define VL53L3CX_IRQ_PICO_PIN 	29

#define WM8731_PICO_I2C_BUS		i2c1
#define WM8731_I2C_ADDR			0x1A

#define KTD2061_PICO_I2C_BUS	i2c1
#define KTD2061_I2C_ADDR		0x68

#define SD_SPI					spi1    // PD100-ESD uses spi1 for sdcard
#define SD_SCK					10
#define SD_MOSI					11
#define SD_MISO					12
#define SD_CS					13
#define SD_DETECT				16
#define SD_SPI_DMA_IRQ          0

#define I2S_DIN_PIN             22
#define I2S_DOUT_PIN            26
#define I2S_BCLK_PIN            27
#define I2S_LRCLK_PIN           28
#define PICO_AUDIO_I2S_DATA_PIN         I2S_DOUT_PIN
#define PICO_AUDIO_I2S_CLOCK_PIN_BASE   I2S_BCLK_PIN
#define PICO_AUDIO_DMA_IRQ      1   // grr! PD100-ESD-SDcard is hardcoded for 0

/* Additional EVE GPU GPIO constants */
#define EVE_FCC_AUDIOMODE_PWM	0x4
#define EVE_GPIO_SPKRBUFOE_N	2
#define EVE_GPIO_SPKRSW			3
/* Eve GPIO_SPKRSW path bit definition */
#define EVE_SPKRSW_TO_CODEC		0
#define EVE_SPKRSW_TO_BUZZER	1

/* Additional constants for LCD auto-brightness from ALS */
#define ALS_DEF_THRESHOLDED	   false // --> analog-style log-brightness
#define ALS_DEF_THRESHOLD_LUX  200
#define ALS_DEF_LCD_BRIGHT_LO  20	// percent within REG_PWM min-max
#define ALS_DEF_LCD_BRIGHT_HI  80	// percent within REG_PWM min-max
#define ALS_DEF_USE_LOG10	   false
#define ALS_DEF_LOG_MULTIPLIER 7.0	  // 9.9323
#define ALS_DEF_LOG_CONSTANT   5.0	  // was 27.059
#define ALS_DEF_LUX_BREAKPOINT 1254.0 // max lux when LCD at max brightness

/* Common PD-RP2040 GPIO pinouts to display GPU, LCD and RS845 */
#define EVE_SPI					spi0    // PD100-ESD uses spi0 for GPU
#define EVE_SCK					2
#define EVE_MOSI				3
#define EVE_MISO				4
#define EVE_IO2					5
#define EVE_IO3					6
#define EVE_PDN					7
#define EVE_CS					14

// clang-format on

typedef enum {
	dvr_rtc = 0,	  // 0x000001 hard err...
	dvr_mio,		  // 0x000002
	dvr_als,		  // 0x000004
	dvr_tof,		  // 0x000008
	dvr_led,		  // 0x000010
	dvr_max
} DRIVER_FAILFLAG_BITNUM;

#endif // _PD_HWDEFS_H_