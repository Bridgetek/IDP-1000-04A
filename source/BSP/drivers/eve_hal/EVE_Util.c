/**
 * @file EVE_Util.c
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

#include "EVE_Platform.h"

void EVE_CoDlImpl_resetCoState(EVE_HalContext *phost);

static eve_progmem_const uint8_t c_DlCodeBootup[3 * 4 + 16 * 3 * 4] = {
	0, 0, 0, 2, // GPU instruction CLEAR_COLOR_RGB
	7, 0, 0, 0x26, // GPU instruction CLEAR

	0, 0, 0, 5, // Bitmap_handle(0)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	1, 0, 0, 5, // Bitmap_handle(1)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	2, 0, 0, 5, // Bitmap_handle(2)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	3, 0, 0, 5, // Bitmap_handle(3)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	4, 0, 0, 5, // Bitmap_handle(4)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	5, 0, 0, 5, // Bitmap_handle(5)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	6, 0, 0, 5, // Bitmap_handle(6)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	7, 0, 0, 5, // Bitmap_handle(7)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	8, 0, 0, 5, // Bitmap_handle(8)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	9, 0, 0, 5, // Bitmap_handle(9)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	10, 0, 0, 5, // Bitmap_handle(10)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	11, 0, 0, 5, // Bitmap_handle(11)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	12, 0, 0, 5, // Bitmap_handle(12)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	13, 0, 0, 5, // Bitmap_handle(13)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	14, 0, 0, 5, // Bitmap_handle(14)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)
	15, 0, 0, 5, // Bitmap_handle(15)
	0, 0, 0, 0x28, // Bitmap_layout_H(0,0)
	0, 0, 0, 0x29, // Bitmap_Size_H(0,0)

	0, 0, 0, 0, // GPU instruction DISPLAY
};

/** @note Update `EVE_DISPLAY_T` in `EVE_Util.h` when adding display presets. */
static const uint16_t s_DisplayResolutions[EVE_DISPLAY_NB][4] = {
	/* width, height, refreshRate, hsfWidth */

	{ 480, 272, 60, 0 }, /* EVE_DISPLAY_DEFAULT (values ignored) */

	/* Landscape */
	{ 1280, 800, 57, 0 }, /* EVE_DISPLAY_WXGA_1280x800_57Hz */
};

/**
 * @brief Clear the screen
 *
 * @param phost Pointer to Hal context
 */
void EVE_Util_clearScreen(EVE_HalContext *phost)
{
	EVE_Hal_wrProgMem(phost, RAM_DL, c_DlCodeBootup, sizeof(c_DlCodeBootup));
	EVE_Hal_wr8(phost, REG_DLSWAP, DLSWAP_FRAME);
}

/**
 * @brief
 *
 * @param phost  Pointer to Hal context
 * @param bootup
 */
void EVE_Util_bootupDefaults(EVE_HalContext *phost, EVE_BootupParameters *bootup)
{
	int32_t chipId = EVE_CHIPID;
	(void)chipId;

	memset(bootup, 0, sizeof(EVE_BootupParameters));

#if !defined(EVE_USE_INTERNAL_OSC)
	/* Board without external oscillator will not work when ExternalOsc is enabled */
	bootup->ExternalOsc = true;
#endif

#ifdef EVE_SYSTEM_CLOCK
	bootup->SystemClock = EVE_SYSTEM_CLOCK;
#else
	bootup->SystemClock = EVE_SYSCLK_72M; /* 72Mhz is default for BT8xx */
#endif

#if defined(ENABLE_SPI_QUAD)
	bootup->SpiChannels = EVE_SPI_QUAD_CHANNEL;
	bootup->SpiDummyBytes = 2;
#elif defined(ENABLE_SPI_DUAL)
	bootup->SpiChannels = EVE_SPI_DUAL_CHANNEL;
	bootup->SpiDummyBytes = 2;
#else
	bootup->SpiChannels = EVE_SPI_SINGLE_CHANNEL;
	bootup->SpiDummyBytes = 1;
#endif
}

#ifndef EVE_HARDCODED_DISPLAY_TIMINGS
#define EVE_HARDCODED_DISPLAY_TIMINGS 1
#endif


/**
 * @brief
 *
 * @param freq
 * @return uint16_t
 */
static uint16_t getPClkFreq(uint32_t freq)
{
	// uint32_t resFreq;
	uint32_t res;
	// eve_printf_debug("Want frequency %i\n", (int)wantFreq);
	int32_t nearestFreq = 0x7FFFFFFFL;
	int32_t nearestDiv = 0;
	int32_t nearestMul = 0;
	for (uint32_t div = 1; div < 15; ++div)
	{
		uint32_t mul = (freq * div) / 6000000;
		if (mul > 19)
			break;
		// eve_printf_debug("Div %i Mul %i\n", (int)div, (int)mul);
		if (mul > 1)
		{
			int32_t resFreq = 6000000 * mul / div;
			// eve_printf_debug("Res frequency %i\n", (int)resFreq);
			int32_t diffFreq = resFreq - (int32_t)freq;
			if (diffFreq >= 0 && diffFreq < (nearestFreq - (int32_t)freq))
			{
				nearestFreq = resFreq;
				nearestDiv = div;
				nearestMul = mul;
			}
		}
		if (mul && mul <= 18)
		{
			int32_t resFreq = 6000000 * (mul + 1) / div;
			// eve_printf_debug("Res frequency (mul + 1) %i\n", (int)resFreq);
			int32_t diffFreq = resFreq - (int32_t)freq;
			if (diffFreq >= 0 && diffFreq < (nearestFreq - (int32_t)freq))
			{
				nearestFreq = resFreq;
				nearestDiv = div;
				nearestMul = mul + 1;
			}
		}
	}
	if (nearestFreq != 0x7FFFFFFFL)
	{
		// resFreq = nearestFreq;
		// maxRate = resFreq / minCycles;
		uint32_t pllFreq = 12 * nearestMul;
		uint32_t range = pllFreq >= 160 ? 3 : (pllFreq >= 80 ? 2 : (pllFreq >= 40 ? 1 : 0));
		res = (nearestDiv & 0xF) | ((nearestMul & 0x3F) << 4) | (range << 10);
		// eve_printf_debug("Set PCLK Freq to %X (freq %i, mul %i, div %i)\n", (int)config->PClkFreq, (int)nearestFreq, (int)nearestMul, (int)nearestDiv);
		return res;
	}
	return 0;
}

/**
 * @brief
 *
 * @param pclkFreq
 * @return uint32_t
 */
static inline uint32_t getFreqFromPClkFreq(uint16_t pclkFreq)
{
	if (!pclkFreq)
		return 0;
	uint32_t mul = (pclkFreq >> 4) & 0x3F;
	uint32_t div = pclkFreq & 0xF;
	uint32_t res = mul * (12 * 1000 * 1000) / div / 2;
	return res;
}

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 * @param config
 * @param width
 * @param height
 * @param refreshRate
 * @param hsfWidth
 * @param freq
 * @return true True if successful
 * @return false False if error
 */
static bool configDefaultsEx(EVE_HalContext *phost, EVE_ConfigParameters *config, uint32_t width, uint32_t height, uint32_t refreshRate, uint32_t hsfWidth, uint32_t freq)
{
	/* Only false if the output resolution will be wrong.
	Not affected by unsupported HSF or refreshRate */
	bool supportedResolution = true;

	uint32_t screenWidth;
	uint32_t pixels;
	uint32_t minCycles;
	uint32_t maxRate;
	uint32_t pclk;

	uint32_t cycles;
	uint32_t vcycle;
	uint32_t hcycle;
	uint32_t hoffset;
	uint32_t voffset;
	uint32_t hsync1;
	uint32_t vsync1;

#ifdef EVE_SUPPORT_HSF
	uint32_t maxFreq;
	uint32_t maxRatePass;
	uint32_t pclkFreq = 0;
#endif

	memset(config, 0, sizeof(EVE_ConfigParameters));

	/*
	Refresh rate is as follows:
	REG_VCYCLE * REG_HCYCLE * REG_PCLK is the number of clocks each frame takes,
	Divide REG_FREQUENCY by this number to get the frame refresh rate in Hz.
	REG_HCYCLE must be higher than REG_HSIZE, REG_VCYCLE must be higher than REG_VSIZE.

	By default, FT800 is at 48MHz, FT810 is at 60MHz, and BT815 is increased to 72MHz.
	User may configure this differently in `EVE_Util_bootup`.

	Clocks per frame by clock frequency at 60Hz:
	- 84MHz: 1400k
	- 72MHz: 1200k
	- 60MHz: 1000k
	- 48MHz: 800k
	- 36MHz: 600k
	- 24MHz: 400k

	Clocks required per frame by display resolution at 60Hz:
	Max PCLK at frequency: 24  36  48  60  72  84MHz
	- 320x240: 76.8k        5   7  10  13  15  18
	- 480x272: 130.56k      3   4   6   7   9  10
	*/

	/* Trim oversize displays */
#ifdef EVE_SUPPORT_HSF
	if (hsfWidth >= width)
	{
		eve_printf_debug("Screen HSF width %i is too large, disabled\n", (int)config->HsfWidth);
		hsfWidth = 0;
		width = hsfWidth;
		supportedResolution = false;
	}
#endif
	if (width > 2047)
	{
		eve_printf_debug("Line buffer width %i is too large for this configuration\n", (int)config->Width);
		width = 2047;
		supportedResolution = false;
	}
	if (height > 2047)
	{
		eve_printf_debug("Screen height %i is too large for this configuration\n", (int)config->Height);
		height = 2047;
		supportedResolution = false;
	}

#ifdef EVE_SUPPORT_HSF
	if (EVE_Hal_supportHsf(phost))
	{
		/* Render width */
		config->Width = (int16_t)width;

		/* Screen width */
		config->HsfWidth = (int16_t)hsfWidth;
	}
	else
#endif
	    if (hsfWidth)
	{
		/* Use the screen width if HSF is not supported */
		config->Width = (int16_t)hsfWidth;
		eve_printf_debug("Display config specifies HSF but not supported on this platform\n");
	}
	else
	{
		config->Width = (int16_t)width;
	}
	config->Height = (int16_t)height;

	screenWidth = hsfWidth ? hsfWidth : width; /* Use screen width for calculation */
	pixels = screenWidth * height;

	/* Calculate maximum refresh rate */
	minCycles = pixels + (pixels >> 2); /* pixels * 1.25 */
#ifdef EVE_SUPPORT_HSF
	maxRatePass = (freq - 1) / minCycles;
	maxFreq = (freq << 1) < 96000000 ? (freq << 1) : 96000000;
	if (hsfWidth)
		maxRate = maxRatePass; // No 2X
	else
		maxRate = maxFreq / minCycles; // Support 2X
#else
	maxRate = freq / minCycles;
#endif

	/* If the refresh rate is too low, try with tighter settings */
	if (refreshRate > maxRate)
	{
		minCycles = pixels + (pixels >> 3); /* pixels * 1.125 */
#ifdef EVE_SUPPORT_HSF
		maxRatePass = (freq - 1) / minCycles;
		if (hsfWidth)
			maxRate = maxRatePass; // No 2X
		else
			maxRate = maxFreq / minCycles; // Support 2X
#else
		maxRate = freq / minCycles;
#endif
	}

#ifdef EVE_SUPPORT_HSF
	if (EVE_Hal_supportHsf(phost) && (hsfWidth || refreshRate > maxRatePass))
	{
		/* Setup EXTSYNC with freq lower (not equal) than system frequency for HSF, up to 2X otherwise */
		/* LL2 frequency = 12MHz * REG_PCLK_FREQ[8:4] */
		/* PCLK frequency = PLL2 frequency / REG_PCLK_FREQ[3:0] / 2 */
		/* PCLK frequency = 6MHz * REG_PCLK_FREQ[8:4] / REG_PCLK_FREQ[3:0] */
		/* PCLK frequency = 6MHz * mul / div */
		int32_t wantFreq = minCycles * (refreshRate > maxRate ? maxRate : refreshRate); // 25.92m for 800x480
		config->PClkFreq = getPClkFreq(wantFreq);
		pclkFreq = getFreqFromPClkFreq(config->PClkFreq);
		if (pclkFreq)
			maxRate = pclkFreq / minCycles;
	}
#endif

#ifdef EVE_SUPPORT_HSF
	if (pclkFreq)
	{
		pclk = 1;
	}
	else
#endif
	    if (maxRate < refreshRate)
	{
		/* Trim unsupported framerate */
		eve_printf_debug("Frame rate limited to %d\n", (unsigned int)maxRate);
		refreshRate = maxRate;
		pclk = 1;
	}
	else
	{
#ifdef EVE_SUPPORT_HSF
		pclk = maxRatePass / refreshRate;
#else
		pclk = maxRate / refreshRate;
#endif
	}
	config->PCLK = (uint8_t)pclk;

	/* Change PCLKPol if the right edge of the display is noisy,
	or if the display is showing jagged colors. Values 0 or 1.
	This toggles the polarity of the PCLK. */
	config->PCLKPol = 1; /* non-default */

	/* Approximate an average good setting */
#ifdef EVE_SUPPORT_HSF
	cycles = (pclkFreq ? pclkFreq : freq) / (refreshRate * pclk);
#else
	cycles = freq / (refreshRate * pclk);
#endif
	vcycle = height * cycles / pixels; /* max vcycle without blanking, on average 1.125x height, or 1.25x */
	vcycle = (vcycle + ((vcycle + height) >> 1) + height + height) >> 2; /* vcycle is 1.5 max vcycle + 2.5 height / 4, so on average 1.046875x height, or 1.09375x */
	hcycle = cycles / vcycle; /* cycles div by vcycle, on average 1.07375x, or 1.1425x */
	hoffset = (hcycle - screenWidth) >> 1;
	voffset = (vcycle - height) >> 1;
	hsync1 = hoffset >> 1;
	vsync1 = voffset >> 1;
	hoffset += hsync1;
	voffset += vsync1;
	if (pclkFreq && pclkFreq >= freq)
	{
		hcycle &= ~1UL;
		hoffset &= ~1UL;
		hsync1 &= ~1UL;
	}
	config->HCycle = (int16_t)hcycle;
	config->HSync0 = 0;
	config->HSync1 = (int16_t)hsync1;
	config->HOffset = (int16_t)hoffset;
	config->VCycle = (int16_t)vcycle;
	config->VSync0 = 0;
	config->VSync1 = (int16_t)vsync1;
	config->VOffset = (int16_t)voffset;

	/* Verify */
	eve_assert(config->HSync1);
	eve_assert(config->HOffset > config->HSync1);
#ifdef EVE_SUPPORT_HSF
	eve_assert(config->HCycle > (config->HsfWidth ? config->HsfWidth : config->Width));
	eve_assert((config->HCycle - (config->HsfWidth ? config->HsfWidth : config->Width)) > config->HOffset);
#else
	eve_assert(config->HCycle > config->Width);
	eve_assert((config->HCycle - config->Width) > config->HOffset);
#endif
	eve_assert(config->VSync1);
	eve_assert(config->VOffset > config->VSync1);
	eve_assert(config->VCycle > config->Height);
	eve_assert((config->VCycle - config->Height) > config->VOffset);

#ifdef EVE_ADAPTIVE_FRAMERATE
	config->AdaptiveFramerate = 1;
#else
	config->AdaptiveFramerate = 0;
#endif

	config->AhHCycleMax = 0;

	/* Other options */
	/* Toggle CSpread if you see red and blue fringing on black and white edges */
	config->CSpread = 0; /* non-default */
	/* Change this if RGB colors are swapped */
	config->Swizzle = 0;
	config->Dither = 1;
	config->OutBitsR = 6;
	config->OutBitsG = 6;
	config->OutBitsB = 6;

	return supportedResolution;
}

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 * @param config
 * @param width
 * @param height
 * @param refreshRate
 * @param hsfWidth
 * @return true True if successful
 * @return false False if error
 */
bool EVE_Util_configDefaultsEx(EVE_HalContext *phost, EVE_ConfigParameters *config, uint32_t width, uint32_t height, uint32_t refreshRate, uint32_t hsfWidth)
{
	uint32_t freq = EVE_Hal_rd32(phost, REG_FREQUENCY);
	bool res = configDefaultsEx(phost, config, width, height, refreshRate, hsfWidth, freq);
#ifdef _DEBUG
	if (config->PClkFreq)
	{
		uint32_t pclkFrequency = getFreqFromPClkFreq(config->PClkFreq);
		eve_printf_debug("Display refresh rate set to %f Hz using pclk %X freq %f MHz\n", (float)((double)pclkFrequency / ((double)config->HCycle * (double)config->VCycle)), (int)config->PClkFreq, (float)((double)pclkFrequency * 0.000001));
	}
	else
		eve_printf_debug("Display refresh rate set to %f Hz\n", (float)((double)freq / ((double)config->HCycle * (double)config->VCycle * (double)config->PCLK)));
#endif
	return res;
}

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 * @param config
 * @param display
 */
void EVE_Util_configDefaults(EVE_HalContext *phost, EVE_ConfigParameters *config, EVE_DISPLAY_T display)
{
	bool supportedResolution;
	uint32_t freq = EVE_Hal_rd32(phost, REG_FREQUENCY);
	uint32_t width;
	uint32_t height;
	uint32_t refreshRate;
	uint32_t hsfWidth;

	/* NOTE: Update `EVE_DISPLAY_T` in `EVE_Util.h` when adding display presets. */
	if (!display)
	{
		/* Default displays if none was explicitly chosen */
		display = EVE_DISPLAY_WXGA_1280x800_57Hz;
	}

	width = s_DisplayResolutions[display][0];
	height = s_DisplayResolutions[display][1];
	refreshRate = s_DisplayResolutions[display][2];
	hsfWidth = s_DisplayResolutions[display][3];
	supportedResolution = configDefaultsEx(phost, config, width, height, refreshRate, hsfWidth, freq);

	if (display == EVE_DISPLAY_WXGA_1280x800_57Hz)
	{
		/*
		BT817 known values:
		Resolution: 1280x800
		Refresh rate: 57.393Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 1280;
			config->Height = 800;
			config->HCycle = 1411;
			config->HOffset = 120;
			config->HSync0 = 0;
			config->HSync1 = 100;
			config->VCycle = 815;
			config->VOffset = 14;
			config->VSync0 = 0;
			config->VSync1 = 10;
			config->PCLK = 1;
			config->PClkFreq = 0x8B1; // 66MHz
#ifdef EVE_SUPPORT_HSF
			config->HsfWidth = 0;
#endif
		}
#endif
		config->CSpread = 0;
		config->Dither = 0;
		config->PCLKPol = 0;
		config->OutBitsR = 0;
		config->OutBitsG = 0;
		config->OutBitsB = 0;
	}

	/* Disable max cycle if incompatible hardcoded timings are used */
	if (config->AhHCycleMax && config->HCycle > config->AhHCycleMax)
		config->AhHCycleMax = 0;

#ifdef _DEBUG
	if (config->PClkFreq)
	{
		uint32_t pclkFrequency = getFreqFromPClkFreq(config->PClkFreq);
		eve_printf_debug("Display refresh rate set to %f Hz using pclk %X freq %f MHz\n", (float)((double)pclkFrequency / ((double)config->HCycle * (double)config->VCycle)), (int)config->PClkFreq, (float)((double)pclkFrequency * 0.000001));
	}
	else
		eve_printf_debug("Display refresh rate set to %f Hz\n", (float)((double)freq / ((double)config->HCycle * (double)config->VCycle * (double)config->PCLK)));
#endif
}

#define EXTRACT_CHIPID(romChipId) EVE_extendedChipId((((romChipId) >> 8) & 0xFF) | (((romChipId) & (0xFF)) << 8))

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 * @param bootup
 * @return true True if successful
 * @return false False if error
 */
bool EVE_Util_bootup(EVE_HalContext *phost, EVE_BootupParameters *bootup)
{
	/* IMPORTANT: Do not use EVE_CoCmd functions here, as they can be overridden by hooks */

	const uint32_t expectedChipId = EVE_CHIPID;
	uint8_t engineStatus;
	uint32_t chipId;
	uint8_t id;
	uint32_t freq;

	do
	{
		int tries = 0;

		/* EVE will be in SPI Single channel after POR */
		if (!EVE_Hal_powerCycle(phost, true))
			return false;

		if (bootup->ExternalOsc)
		{
			/* Set the clk to external clock. Must disable it when no external clock source on the board */
			EVE_Hal_hostCommand(phost, EVE_EXTERNAL_OSC);
			EVE_sleep(10);
		}
		else
		{
			/* Send CLKINT explicitly to play safe for all EVE series chip*/
			EVE_Hal_hostCommand(phost, EVE_INTERNAL_OSC);
			EVE_sleep(10);
		}

		/* Update system clock as per user selected */
		if (bootup->SystemClock)
		{
			EVE_Host_selectSysClk(phost, bootup->SystemClock);
		}

		EVE_Host_coreReset(phost);
		/* Access address 0 to wake up the FT800 */
		EVE_Hal_hostCommand(phost, EVE_ACTIVE_M);
		EVE_sleep(300);

		/* Wait for valid chip ID */
		chipId = EVE_Hal_rd32(phost, ROM_CHIPID);
		while (EXTRACT_CHIPID(chipId) != EVE_CHIPID)
		{
			eve_printf_debug("EVE ROM_CHIPID after wake up %lx\n", (unsigned long)chipId);

			++tries;
			EVE_sleep(20);
			if (phost->CbCmdWait && !phost->CbCmdWait(phost))
				return false;

			chipId = EVE_Hal_rd32(phost, ROM_CHIPID);
		}
	} while (!chipId);

	/* Validate chip ID to ensure the correct HAL is used */
	/* ROM_CHIPID is valid across all EVE devices */
	if (expectedChipId && EXTRACT_CHIPID(chipId) != expectedChipId)
		eve_printf_debug("Mismatching EVE chip id %lx, expect model %lx\n", (unsigned long)((chipId >> 8) & 0xFF) | ((chipId & 0xFF) << 8), (unsigned long)expectedChipId);
	eve_printf_debug("EVE chip id %lx %lx.%lx (EVE gen %i)\n", (unsigned long)EVE_shortChipId(EXTRACT_CHIPID(chipId)), (unsigned long)((chipId >> 16) & 0xFF), (unsigned long)((chipId >> 24) & 0xFF), EVE_gen(EXTRACT_CHIPID(chipId)));

	/* Turn off previous audio playback (in case powerdown is not connected) */
	EVE_Hal_wr32(phost, REG_PLAYBACK_LENGTH, 0);

	/* Read Register ID to check if EVE is ready. */
	while ((id = EVE_Hal_rd8(phost, REG_ID)) != 0x7C)
	{
		eve_printf_debug("EVE register ID after wake up %x\n", (unsigned int)id);

		EVE_sleep(20);
		if (phost->CbCmdWait && !phost->CbCmdWait(phost))
			return false;
	}
	eve_printf_debug("EVE register ID after wake up %x\n", (unsigned int)id);
	eve_assert(chipId == EVE_Hal_rd32(phost, ROM_CHIPID));

	/* Read REG_CPURESET to check if engines are ready.
	    Bit 0 for coprocessor engine,
	    Bit 1 for touch engine,
	    Bit 2 for audio engine.
	*/
	while ((engineStatus = EVE_Hal_rd8(phost, REG_CPURESET)) != 0x00)
	{
		if (engineStatus & 0x01)
		{
			eve_printf_debug("Coprocessor engine is not ready\n");
		}
		if (engineStatus & 0x02)
		{
			eve_printf_debug("Touch engine is not ready\n");
		}
		if (engineStatus & 0x04)
		{
			eve_printf_debug("Audio engine is not ready\n");
		}

		EVE_sleep(20);
		if (phost->CbCmdWait && !phost->CbCmdWait(phost))
			return false;
	}
	eve_printf_debug("All engines are ready\n");

	/* Update REG_FREQUENCY as per user selected */
	if (bootup->SystemClock != EVE_SYSCLK_DEFAULT)
	{
		uint32_t clockMHz = bootup->SystemClock * 12;
		freq = clockMHz * 1000 * 1000;
		EVE_Hal_wr32(phost, REG_FREQUENCY, freq);
		EVE_Hal_flush(phost);
		eve_printf_debug("EVE clock frequency set to %d MHz\n", (unsigned int)clockMHz);
	}
	else
	{
		freq = EVE_Hal_rd32(phost, REG_FREQUENCY);
		eve_printf_debug("EVE default clock is %d MHz\n", (unsigned int)(freq / 1000000));
	}

	/* Switch to configured default SPI channel mode */
	EVE_Hal_setSPI(phost, bootup->SpiChannels, bootup->SpiDummyBytes);
#ifdef _DEBUG
	const char *spiChannels = (phost->SpiChannels == EVE_SPI_QUAD_CHANNEL)
	    ? "Quad"
	    : ((phost->SpiChannels == EVE_SPI_DUAL_CHANNEL) ? "Dual" : "Single");
	const char *requested = (bootup->SpiChannels == EVE_SPI_QUAD_CHANNEL)
	    ? "Quad"
	    : ((bootup->SpiChannels == EVE_SPI_DUAL_CHANNEL) ? "Dual" : "Single");
	if (bootup->SpiChannels == phost->SpiChannels)
		eve_printf_debug("%s channel SPI\n", spiChannels);
	else
		eve_printf_debug("%s channel SPI (%s unsupported)\n", spiChannels, requested);
#endif

	/* Sanity check after SPI change */
	if (EVE_Hal_rd32(phost, REG_FREQUENCY) != freq)
	{
		eve_printf_debug("SPI channel configuration change failed\n");
		if (bootup->SpiChannels > EVE_SPI_SINGLE_CHANNEL)
		{
			/* Try again with dual channel */
			--bootup->SpiChannels;
			eve_printf_debug("Retry with %s channel SPI\n", bootup->SpiChannels ? "Dual" : "Single");
			return EVE_Util_bootup(phost, bootup);
		}
		return false;
	}

	/* In some cases, for example, QSPI on MM817EV board with MM930LITE,
	   the drive strength is required to adjust for better signal integrity.
	   Setting to max current 20mA here is to play safe.  Users may adjust the
	   corresponding bits of REG_GPIOX according to their own situation. */
	EVE_Hal_wr16(phost, REG_GPIOX_DIR, 0xffff);
	EVE_Hal_wr16(phost, REG_GPIOX, 0xfffb);

	/* Update touch firmware */
	eve_printf_debug("Setup touch for Goodix\n");
	EVE_Hal_wr8(phost, REG_CPURESET, 2);
	EVE_Hal_wr16(phost, REG_TOUCH_CONFIG, 0x05d0);
	EVE_Hal_wr8(phost, REG_CPURESET, 0);

	return true;
}

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 * @param config
 * @return true True if successful
 * @return false False if error
 */
bool EVE_Util_config(EVE_HalContext *phost, EVE_ConfigParameters *config)
{
	/* IMPORTANT: Do not use EVE_CoCmd functions here, as they can be overridden by hooks */

	bool reconfigure = phost->PCLK;
	if (reconfigure)
	{
		/* If PCLK is already set, we're reconfiguring. Quiet flush first */
		eve_printf_debug("Reconfiguring\n");
		EVE_Cmd_waitFlush(phost);
	}

	const bool swapXY = EVE_Hal_rd8(phost, REG_ROTATE) & 0x2;
	uint16_t wp;
	uint16_t rp;

	eve_assert(config->Width < 2048);
	eve_assert(config->Height < 2048);

	/* Turn off display output clock */
	EVE_Hal_wr8(phost, REG_PCLK, 0);
	phost->PCLK = 0;

	EVE_Hal_wr16(phost, REG_HCYCLE, config->HCycle);
	EVE_Hal_wr16(phost, REG_HOFFSET, config->HOffset);
	EVE_Hal_wr16(phost, REG_HSYNC0, config->HSync0);
	EVE_Hal_wr16(phost, REG_HSYNC1, config->HSync1);
	EVE_Hal_wr16(phost, REG_VCYCLE, config->VCycle);
	EVE_Hal_wr16(phost, REG_VOFFSET, config->VOffset);
	EVE_Hal_wr16(phost, REG_VSYNC0, config->VSync0);
	EVE_Hal_wr16(phost, REG_VSYNC1, config->VSync1);
	EVE_Hal_wr8(phost, REG_SWIZZLE, config->Swizzle);
	EVE_Hal_wr8(phost, REG_PCLK_POL, config->PCLKPol);
	EVE_Hal_wr16(phost, REG_HSIZE, config->Width);
	EVE_Hal_wr16(phost, REG_VSIZE, config->Height);
	EVE_Hal_wr16(phost, REG_CSPREAD, config->CSpread);
	EVE_Hal_wr16(phost, REG_DITHER, config->Dither);
	EVE_Hal_wr16(phost, REG_OUTBITS,
	    (uint16_t)(((config->OutBitsR & 0x7) << 6)
	        | ((config->OutBitsG & 0x7) << 3)
	        | (config->OutBitsB & 0x7)));
	if (swapXY)
	{
		phost->Width = config->Height;
		phost->Height = config->Width;
	}
	else
	{
		phost->Width = config->Width;
		phost->Height = config->Height;
	}

	bool pclk2X = false;
	uint32_t freq = EVE_Hal_rd32(phost, REG_FREQUENCY);
	uint16_t pclkFreq = config->PClkFreq;
	if (!pclkFreq)
	{
		/* If not configured, match system frequency, in case a legacy configuration with PClk 1 is specified */
		/* PClk Freq is 60MHz by default, but we raise the system frequency to 72MHz during bootup */
		uint32_t refMul = freq / 6000000;
		uint32_t pllFreq = 12 * refMul;
		uint32_t range = pllFreq >= 160 ? 3 : (pllFreq >= 80 ? 2 : (pllFreq >= 40 ? 1 : 0));
		pclkFreq = 1 | ((refMul & 0x3F) << 4) | (range << 10);
	}
	EVE_Hal_wr32(phost, REG_PCLK_FREQ, pclkFreq);
	if (config->PClk2X)
	{
		/* User forces 2X */
		EVE_Hal_wr8(phost, REG_PCLK_2X, 1);
	}
	else if (config->PCLK == 1)
	{
		/* Send two pixels per system clock to the EXTSYNC block if PCLK_frequency > SystemClock */
		uint32_t pclkFrequency = getFreqFromPClkFreq(pclkFreq);
		pclk2X = pclkFrequency > freq;
		EVE_Hal_wr8(phost, REG_PCLK_2X, pclk2X);
	}
	else
	{
		EVE_Hal_wr8(phost, REG_PCLK_2X, 0);
	}

	EVE_Hal_wr8(phost, REG_ADAPTIVE_FRAMERATE, config->AdaptiveFramerate);

	EVE_Hal_wr16(phost, REG_AH_HCYCLE_MAX, config->AhHCycleMax);

	EVE_Util_clearScreen(phost);

	/* Refresh fifo */
	wp = EVE_Cmd_wp(phost);
	rp = EVE_Cmd_rp(phost);
	EVE_Cmd_space(phost);
	phost->MediaFifoAddress = 0;
	phost->MediaFifoSize = 0;

	/* Coprocessor needs a reset */
	if (reconfigure ? EVE_CMD_FAULT(rp) : (wp || rp))
	{
		/* If this occurs after powerdown and bootup, powerdown is not working */
		eve_printf_debug("Coprocessor fifo not empty\n");
		if (!EVE_Util_resetCoprocessor(phost))
			return false;
	}
	else
	{
		/* Setup coprocessor defaults */
		eve_printf_debug("Configure coprocessor defaults\n");
		EVE_Cmd_wr32(phost, CMD_DLSTART);
		EVE_Cmd_wr32(phost, CMD_COLDSTART);

		/* Set the latest API level. */
		EVE_Cmd_wr32(phost, CMD_APILEVEL);
		EVE_Cmd_wr32(phost, 2);

#ifdef EVE_FLASH_AVAILABLE
		/* Reattach flash to avoid inconsistent state */
		EVE_Cmd_wr32(phost, CMD_FLASHDETACH);
		EVE_Cmd_wr32(phost, CMD_FLASHATTACH);
#endif

		EVE_CoDlImpl_resetCoState(phost);
	}

#ifdef EVE_SUPPORT_HSF
	if (config->HsfWidth)
	{
		if (EVE_Hal_supportHsf(phost) && !pclk2X)
		{
			EVE_Cmd_startFunc(phost);
			EVE_Cmd_wr32(phost, CMD_HSF);
			EVE_Cmd_wr32(phost, config->HsfWidth);
			EVE_Cmd_endFunc(phost);
		}
		else
		{
			/* Set width to screen width if HSF is not supported here */
			eve_printf_debug("HSF specified but not supported on this device\n");
			EVE_Hal_wr16(phost, REG_HSIZE, config->HsfWidth);
			if (swapXY)
				phost->Height = config->HsfWidth;
			else
				phost->Width = config->HsfWidth;
		}
	}
#endif

	EVE_Hal_flush(phost);
	if (!EVE_Cmd_waitFlush(phost))
		return false;

	EVE_Hal_wr8(phost, REG_PCLK, config->PCLK); /* After this display is visible on the LCD */
	phost->PCLK = config->PCLK;

#ifdef EVE_SUPPORT_HSF
	if (config->HsfWidth)
	{
		if (EVE_Hal_supportHsf(phost))
		{
			eve_printf_debug("HSize: %i, HSF HSize: %i\n", EVE_Hal_rd32(phost, REG_HSIZE), EVE_Hal_rd32(phost, REG_HSF_HSIZE));
		}
	}
#endif

	eve_printf_debug("EVE configuration ready\n");
	return true;
}

/**
 * @brief
 *
 * @param phost Pointer to Hal context
 */
void EVE_Util_shutdown(EVE_HalContext *phost)
{
	EVE_Hal_wr16(phost, REG_GPIOX_DIR, 0xffff);
	EVE_Hal_wr16(phost, REG_GPIOX, 0);

	EVE_Hal_wr8(phost, REG_PCLK, 0);
	EVE_Hal_flush(phost);
	EVE_Hal_powerCycle(phost, false);
}

#if defined(_DEBUG)
/**
 * @brief Backup the last 128 bytes of RAM_G, which may be used for an error message
 *
 * @param phost Pointer to Hal context
 */
void debugBackupRamG(EVE_HalContext *phost)
{
	if (!phost->DebugMessageVisible)
	{
		EVE_Hal_rdMem(phost, phost->DebugBackup, RAM_G + RAM_G_SIZE - sizeof(phost->DebugBackup), sizeof(phost->DebugBackup));
		phost->DebugMessageVisible = true;
	}
}
/**
 * @brief
 *
 * @param phost Pointer to Hal context
 */
static void debugRestoreRamG(EVE_HalContext *phost)
{
	if (phost->DebugMessageVisible)
	{
		EVE_Hal_wrMem(phost, RAM_G + RAM_G_SIZE - sizeof(phost->DebugBackup), phost->DebugBackup, sizeof(phost->DebugBackup));
		phost->DebugMessageVisible = false;
	}
}
#endif

/**
 * @brief Reset Coprocessor
 *
 * @param phost Pointer to Hal context
 * @return true True if successful
 * @return false False if error
 */
bool EVE_Util_resetCoprocessor(EVE_HalContext *phost)
{
	/* IMPORTANT: Do not use EVE_CoCmd functions here, as they can be overridden by hooks */
	bool ready;
	uint16_t rd;
	uint16_t wr;
	(void)rd;
	(void)wr;

	eve_printf_debug("Reset coprocessor\n");

	/* Set REG_CPURESET to 1, to hold the coprocessor in the reset condition */
	EVE_Hal_wr8(phost, REG_CPURESET, 1);
	EVE_Hal_flush(phost);
	EVE_sleep(100);

	/* Set REG_CMD_READ and REG_CMD_WRITE to zero */
	EVE_Hal_wr16(phost, REG_CMD_READ, 0);
	EVE_Hal_wr16(phost, REG_CMD_WRITE, 0);
	EVE_Hal_wr16(phost, REG_CMD_DL, 0);
	EVE_Hal_wr8(phost, REG_PCLK, phost->PCLK); /* j1 will set the pclk to 0 for that error case */

	/* Stop playing audio in case video with audio was playing during reset */
	EVE_Hal_wr8(phost, REG_PLAYBACK_PLAY, 0);

	/* Default */
	phost->CmdFault = false;

	/* Set REG_CPURESET to 0, to restart the coprocessor */
	EVE_Hal_wr8(phost, REG_CPURESET, 0);
	EVE_Hal_flush(phost);
	EVE_sleep(100);
	eve_assert((wr = EVE_Hal_rd16(phost, REG_CMD_WRITE)) == 0);
	eve_assert((rd = EVE_Hal_rd16(phost, REG_CMD_READ)) == 0);

	/* Refresh fifo */
	EVE_Cmd_waitFlush(phost);
	phost->MediaFifoAddress = 0;
	phost->MediaFifoSize = 0;

	/* Cold start. Ensure that the coprocessor is ready. */
	EVE_Cmd_wr32(phost, CMD_DLSTART);
	EVE_Cmd_wr32(phost, CMD_COLDSTART);

	/* Set the latest API level.*/
	EVE_Cmd_wr32(phost, CMD_APILEVEL);
	EVE_Cmd_wr32(phost, 2);

#ifdef EVE_FLASH_AVAILABLE
	/* Reattach flash to avoid inconsistent state */
	EVE_Cmd_wr32(phost, CMD_FLASHATTACH);
#endif

	/* Wait for coprocessor to be ready */
	ready = EVE_Cmd_waitFlush(phost);
	if (phost->CbCoprocessorReset) /* Notify */
		phost->CbCoprocessorReset(phost, !ready);

#if defined(_DEBUG)
	debugRestoreRamG(phost);
#endif
	EVE_CoDlImpl_resetCoState(phost);
	return ready;
}

/**
 * @brief Bootup Coprocessor
 *
 * @param phost Pointer to Hal context
 * @return true True if successful
 * @return false False if error
 */
bool EVE_Util_bootupConfig(EVE_HalContext *phost)
{
	EVE_BootupParameters bootup;
	EVE_ConfigParameters config;

	EVE_Util_bootupDefaults(phost, &bootup);
	if (!EVE_Util_bootup(phost, &bootup))
	{
		return false;
	}

	EVE_Util_configDefaults(phost, &config, EVE_DISPLAY_DEFAULT);
	if (!EVE_Util_config(phost, &config))
	{
		EVE_Util_shutdown(phost);
		return false;
	}

	return true;
}

/* end of file */
