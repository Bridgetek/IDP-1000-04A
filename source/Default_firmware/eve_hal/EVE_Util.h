/**
 * @file EVE_Util.h
 * @brief EVE HAL framework utilities
 *
 * @author Bridgetek
 *
 * @date 2018
 *
 * MIT License
 *
 * Copyright (c) [2019] [Bridgetek Pte Ltd (BRTChip)]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef EVE_UTIL__H
#define EVE_UTIL__H

/*********************
 *      INCLUDES
 *********************/
#include "EVE_HalDefs.h"

/*************
** TYPEDEFS **
*************/

typedef struct EVE_BootupParameters
{
	/** Clock PLL multiplier (ft81x: 5, 60MHz, bt81x: 6, 72MHz) */
	EVE_81X_PLL_FREQ_T SystemClock;

	/** External oscillator (default: false) */
	bool ExternalOsc;

	/** SPI */
	EVE_SPI_CHANNELS_T SpiChannels; /**< Variable to contain single/dual/quad channels */
	uint8_t SpiDummyBytes; /**< Number of dummy bytes as 1 or 2 for SPI read */

} EVE_BootupParameters;

typedef struct EVE_ConfigParameters
{
	/** @name Display */
	///@{
	int16_t Width; /**< Line buffer width (pixels) */
	int16_t Height; /**< Screen and render height (lines) */
	int16_t HCycle;
	int16_t HOffset;
	int16_t HSync0;
	int16_t HSync1;
	int16_t VCycle;
	int16_t VOffset;
	int16_t VSync0;
	int16_t VSync1;
	uint8_t PCLK;
	int8_t Swizzle;
	int8_t PCLKPol;
	int8_t CSpread;
	uint8_t OutBitsR;
	uint8_t OutBitsG;
	uint8_t OutBitsB;
	bool Dither;
	///@}

	bool AdaptiveFramerate;

	bool PClk2X;
	uint16_t PClkFreq;
	int16_t AhHCycleMax;

#ifdef EVE_SUPPORT_HSF
	/** Physical horizontal pixels. Set to 0 to disable HSF. */
	int16_t HsfWidth; /**< Screen width (columns) */
#endif

} EVE_ConfigParameters;

/** Display resolution presets.
@note Also update `s_DisplayResolutions` and `s_DisplayNames` in EVE_Util.c around ln 50,
as well as `EVE_Util_configDefaults` around ln 500, when adding display presets. */
typedef enum EVE_DISPLAY_T
{
	EVE_DISPLAY_DEFAULT = 0, /**< 0 */

	/** Landscape */
	EVE_DISPLAY_WXGA_1280x800_57Hz, /**< Landscape */

	EVE_DISPLAY_NB,

} EVE_DISPLAY_T;

/**********************
 *  GLOBAL PROTOTYPES
 **********************/

/** @name INIT AND SHUTDOWN */
///@{

/** Get the default bootup parameters. */
void EVE_Util_bootupDefaults(EVE_HalContext *phost, EVE_BootupParameters *bootup);

/** Boot up the device. Obtains the chip Id. Sets up clock and SPI speed. */
bool EVE_Util_bootup(EVE_HalContext *phost, EVE_BootupParameters *bootup);

/** Get the default configuration parameters for the specified display. */
void EVE_Util_configDefaults(EVE_HalContext *phost, EVE_ConfigParameters *config, EVE_DISPLAY_T display);

/** Get the default configuration parameters for the specified display parameters. */
bool EVE_Util_configDefaultsEx(EVE_HalContext *phost, EVE_ConfigParameters *config, uint32_t width, uint32_t height, uint32_t refreshRate, uint32_t hsfWidth);

/** Boot up the device. Configures the display, resets or initializes coprocessor state. */
bool EVE_Util_config(EVE_HalContext *phost, EVE_ConfigParameters *config);

/** Complementary of bootup. Does not close the HAL context. */
void EVE_Util_shutdown(EVE_HalContext *phost);

/** Sets the display list to a blank cleared screen. */
void EVE_Util_clearScreen(EVE_HalContext *phost);

/** Resets the coprocessor.
To be used after a coprocessor fault, or to exit CMD_LOGO.
After a reset, flash will be in attached state (not in full speed).
Coprocessor will be set to the latest API level. */
bool EVE_Util_resetCoprocessor(EVE_HalContext *phost);

/** Calls EVE_Util_bootup and EVE_Util_config using the default parameters */
bool EVE_Util_bootupConfig(EVE_HalContext *phost);
///@}

#endif /* #ifndef EVE_HAL_INCL__H */

/* end of file */
