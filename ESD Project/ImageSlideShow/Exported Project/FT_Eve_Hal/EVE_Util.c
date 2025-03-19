/**
 * This source code ("the Software") is provided by Bridgetek Pte Ltd
 * ("Bridgetek") subject to the licence terms set out
 *   http://brtchip.com/BRTSourceCodeLicenseAgreement/ ("the Licence Terms").
 * You must read the Licence Terms before downloading or using the Software.
 * By installing or using the Software you agree to the Licence Terms. If you
 * do not agree to the Licence Terms then do not download or use the Software.
 *
 * Without prejudice to the Licence Terms, here is a summary of some of the key
 * terms of the Licence Terms (and in the event of any conflict between this
 * summary and the Licence Terms then the text of the Licence Terms will
 * prevail).
 *
 * The Software is provided "as is".
 * There are no warranties (or similar) in relation to the quality of the
 * Software. You use it at your own risk.
 * The Software should not be used in, or for, any medical device, system or
 * appliance. There are exclusions of Bridgetek liability for certain types of loss
 * such as: special loss or damage; incidental loss or damage; indirect or
 * consequential loss or damage; loss of income; loss of business; loss of
 * profits; loss of revenue; loss of contracts; business interruption; loss of
 * the use of money or anticipated savings; loss of information; loss of
 * opportunity; loss of goodwill or reputation; and/or loss of, damage to or
 * corruption of data.
 * There is a monetary cap on Bridgetek's liability.
 * The Software may have subsequently been amended by another user and then
 * distributed by that other user ("Adapted Software").  If so that user may
 * have additional licence terms that apply to those amendments. However, Bridgetek
 * has no liability in relation to those amendments.
 */

#include "EVE_Util.h"
#include "EVE_Platform.h"
#include "EVE_HalImpl.h"


EVE_HAL_EXPORT void EVE_CoDlImpl_resetCoState(EVE_HalContext *phost);
static eve_progmem_const uint8_t c_DlCodeBootup[3 * 4  + 16 * 3 * 4] = {
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

/* NOTE: Update `EVE_DISPLAY_T` in `EVE_Util.h` when adding display presets. */
static const uint16_t s_DisplayResolutions[EVE_DISPLAY_NB][4] = {
	/* width, height, refreshRate, hsfWidth */

	{ 480, 272, 60, 0 }, /* EVE_DISPLAY_DEFAULT (values ignored) */

	/* Landscape */
	{ 320, 240, 56, 0 }, /* EVE_DISPLAY_QVGA_320x240_56Hz */
	{ 480, 272, 60, 0 }, /* EVE_DISPLAY_WQVGA_480x272_60Hz */
	{ 800, 480, 74, 0 }, /* EVE_DISPLAY_WVGA_800x480_74Hz */
	{ 1024, 600, 59, 0 }, /* EVE_DISPLAY_WSVGA_1024x600_59Hz */
	{ 1280, 720, 58, 0 }, /* EVE_DISPLAY_HDTV_1280x720_58Hz */
	{ 1280, 800, 57, 0 }, /* EVE_DISPLAY_WXGA_1280x800_57Hz */

	/* Portrait */
	{ 320, 480, 60, 0 }, /* EVE_DISPLAY_HVGA_320x480_60Hz */

	/* IDM2040-7A, AT‐T070SWH‐51CP, HX8264-E, 60 to 90Hz capable */
	{ 860, 480, 60, 800 }, /* EVE_DISPLAY_WVGA_800x480_W860_60Hz */
	
	/* IDM2040-21R(FT800), */
	{ 480, 480, 48, 0 }, /* EVE_DISPLAY_WQVGA_480x480_48Hz */

	/* Riverdi */
	{ 320, 240, 62, 0 }, /* EVE_DISPLAY_RIVERDI_IPS35_62Hz */
	{ 480, 272, 58, 0 }, /* EVE_DISPLAY_RIVERDI_IPS43_58Hz */
	{ 800, 480, 63, 0 }, /* EVE_DISPLAY_RIVERDI_IPS50_63Hz */
	{ 1024, 600, 59, 0 }, /* EVE_DISPLAY_RIVERDI_IPS70_59Hz */
	{ 1280, 800, 59, 0 }, /* EVE_DISPLAY_RIVERDI_IPS101_59Hz */

};


#if (defined(_WIN32) || defined(__linux__))

/* Interactive platform selection */
static const char *s_HostDisplayNames[EVE_HOST_NB] = {
	"<Unknown>",

	"BT8XX Emulator",
	"FT4222",
	"MPSSE",
	"Embedded",
};

#define EVE_SELECT_CHIP_NB 14


#endif

/* VERIFY: Can the emulator handle this? */
static inline void uploadTouchFirmware(EVE_HalContext *phost)
{
	/* no-op */
}

/**
 * @brief Clear the screen
 *
 * @param phost  Pointer to Hal context
 */
EVE_HAL_EXPORT void EVE_Util_clearScreen(EVE_HalContext *phost)
{
	EVE_Hal_wrProgMem(phost, RAM_DL, c_DlCodeBootup, sizeof(c_DlCodeBootup));
	EVE_Hal_wr8(phost, REG_DLSWAP, DLSWAP_FRAME);
}

EVE_HAL_EXPORT void EVE_Util_bootupDefaults(EVE_HalContext *phost, EVE_BootupParameters *bootup)
{
	int32_t chipId = EVE_CHIPID;
	(void)chipId;

	memset(bootup, 0, sizeof(EVE_BootupParameters));

	/* Board without external oscillator will not work when ExternalOsc is enabled */
	bootup->ExternalOsc = true;

#ifdef EVE_SYSTEM_CLOCK
	bootup->SystemClock = EVE_SYSTEM_CLOCK;
#else
	if (chipId >= EVE_FT800 && chipId <= EVE_BT818)
	{
		if (chipId >= EVE_BT815)
		{
			bootup->SystemClock = EVE_SYSCLK_72M; /* 72Mhz is default for BT8xx */
		}
		else
		    if (chipId >= EVE_FT810)
		{
			bootup->SystemClock = EVE_SYSCLK_60M; /* 60Mhz is default for FT81x */
		}
		else
		{
			bootup->SystemClock = EVE_SYSCLK_48M; /* 48Mhz is default for FT80x */
		}
	}
	else
	{
		/* If the chipid is not known yet, this will reconfigure the system clock during `EVE_Util_bootup`.
		Additional default clock overrides need to be implemented there as well. */
		bootup->SystemClock = EVE_SYSCLK_DEFAULT;
	}
#endif



}

#ifndef EVE_HARDCODED_DISPLAY_TIMINGS
#define EVE_HARDCODED_DISPLAY_TIMINGS 1
#endif


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

static inline uint32_t getFreqFromPClkFreq(uint16_t pclkFreq)
{
	if (!pclkFreq)
		return 0;
	uint32_t mul = (pclkFreq >> 4) & 0x3F;
	uint32_t div = pclkFreq & 0xF;
	uint32_t res = mul * (12 * 1000 * 1000) / div / 2;
	return res;
}


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

	uint32_t maxFreq;
	uint32_t maxRatePass;
	uint32_t pclkFreq = 0;

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
	if (EVE_CHIPID >= EVE_FT810)
	{
		if (hsfWidth >= width)
		{
			eve_printf_debug("Screen HSF width %i is too large, disabled\n", (int)config->HsfWidth);
			hsfWidth = 0;
			width = hsfWidth;
			supportedResolution = false;
		}
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
	}
	else
	{
		if (width > 493)
		{
			/* 494 and up are not reliable */
			eve_printf_debug("Screen width %i is too large for this configuration\n", (int)config->Width);
			width = 493;
			supportedResolution = false;
		}
		if (height > 511)
		{
			eve_printf_debug("Screen height %i is too large for this configuration\n", (int)config->Height);
			height = 511;
			supportedResolution = false;
		}
	}

	if (EVE_Hal_supportHsf(phost))
	{
		/* Render width */
		config->Width = (int16_t)width;

		/* Screen width */
		config->HsfWidth = (int16_t)hsfWidth;

		if (hsfWidth)
		{
		}
	}
	else
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
	maxRatePass = (freq - 1) / minCycles;
	maxFreq = (freq << 1) < 96000000 ? (freq << 1) : 96000000;
	if (hsfWidth)
		maxRate = maxRatePass; // No 2X
	else
		maxRate = maxFreq / minCycles; // Support 2X

	/* If the refresh rate is too low, try with tighter settings */
	if (refreshRate > maxRate)
	{
		minCycles = pixels + (pixels >> 3); /* pixels * 1.125 */
		maxRatePass = (freq - 1) / minCycles;
		if (hsfWidth)
			maxRate = maxRatePass; // No 2X
		else
			maxRate = maxFreq / minCycles; // Support 2X
	}

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

	if (pclkFreq)
	{
		pclk = 1;
	}
	else
	    if (maxRate < refreshRate)
	{
		/* Trim unsupported framerate */
		eve_printf_debug("Frame rate limited to %d\n", (unsigned int)maxRate);
		refreshRate = maxRate;
		pclk = 1;
	}
	else
	{
		pclk = maxRatePass / refreshRate;
	}
	config->PCLK = (uint8_t)pclk;

	/* Change PCLKPol if the right edge of the display is noisy,
	or if the display is showing jagged colors. Values 0 or 1.
	This toggles the polarity of the PCLK. */
	config->PCLKPol = 1; /* non-default */

	/* Approximate an average good setting */
	cycles = (pclkFreq ? pclkFreq : freq) / (refreshRate * pclk);
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
	eve_assert(config->HCycle > (config->HsfWidth ? config->HsfWidth : config->Width));
	eve_assert((config->HCycle - (config->HsfWidth ? config->HsfWidth : config->Width)) > config->HOffset);
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

	if (EVE_CHIPID == EVE_FT812
	    || EVE_CHIPID == EVE_FT813
	    || EVE_CHIPID == EVE_BT882
	    || EVE_CHIPID == EVE_BT883
	    || EVE_CHIPID >= EVE_BT815)
	{
		config->Dither = 0;
		config->OutBitsR = 8;
		config->OutBitsG = 8;
		config->OutBitsB = 8;
	}
	else
	{
		config->Dither = 1;
		config->OutBitsR = 6;
		config->OutBitsG = 6;
		config->OutBitsB = 6;
	}

	return supportedResolution;
}

EVE_HAL_EXPORT bool EVE_Util_configDefaultsEx(EVE_HalContext *phost, EVE_ConfigParameters *config, uint32_t width, uint32_t height, uint32_t refreshRate, uint32_t hsfWidth)
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

EVE_HAL_EXPORT void EVE_Util_configDefaults(EVE_HalContext *phost, EVE_ConfigParameters *config, EVE_DISPLAY_T display)
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

	/* Known values for specific display models */

	if (display == EVE_DISPLAY_QVGA_320x240_56Hz && freq == 48000000)
	{
		/*
		FT800 known values:
		Resolution: 320x240
		Refresh rate: 55.916Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 320;
			config->Height = 240;
			config->HCycle = 408;
			config->HOffset = 70;
			config->HSync0 = 0;
			config->HSync1 = 10;
			config->VCycle = 263;
			config->VOffset = 13;
			config->VSync0 = 0;
			config->VSync1 = 2;
			config->PCLK = 8;
			config->PClkFreq = 0;
			config->HsfWidth = 0;
		}
#endif
		config->Swizzle = 2;
		config->PCLKPol = 0;
		config->CSpread = 1;
		config->Dither = 1;
	}
	else if (display == EVE_DISPLAY_WQVGA_480x272_60Hz && freq == 48000000)
	{
		/*
		FT800 known values:
		Display: LBL-T050BPH-01
		REG_FREQUENCY: 48MHz
		REG_PCLK: 5
		REG_HCYCLE: 548
		REG_VCYCLE: 292
		Resolution: 480x272
		Refresh rate: 59.994Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 480;
			config->Height = 272;
			config->HCycle = 548;
			config->HOffset = 43;
			config->HSync0 = 0;
			config->HSync1 = 41;
			config->VCycle = 292;
			config->VOffset = 12;
			config->VSync0 = 0;
			config->VSync1 = 10;
			config->PCLK = 5;
			config->PClkFreq = 0;
			config->HsfWidth = 0;
		}
#endif
	}
	else if (display == EVE_DISPLAY_WQVGA_480x480_48Hz)
	{
		/*
		FT800 known values:
		Display: LBL-T050BPH-01
		REG_FREQUENCY: 48MHz
		REG_PCLK: 7
		REG_HCYCLE: 548
		REG_VCYCLE: 292
		Resolution: 480x480
		Refresh rate: 59.994Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 480;
			config->Height = 480;
			config->HCycle = 548;
			config->HOffset = 43;
			config->HSync0 = 0;
			config->HSync1 = 41;
			config->VCycle = 512;
			config->VOffset = 12;
			config->VSync0 = 0;
			config->VSync1 = 10;
			config->PCLK = 7;
			config->PClkFreq = 0;
			config->HsfWidth = 0;
		}
#endif
	}
	else if (display == EVE_DISPLAY_WQVGA_480x272_60Hz && EVE_CHIPID >= EVE_BT817)
	{
		/* Same as above, but using PClkFreq */
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 480;
			config->Height = 272;
			config->HCycle = 548;
			config->HOffset = 43;
			config->HSync0 = 0;
			config->HSync1 = 41;
			config->VCycle = 292;
			config->VOffset = 12;
			config->VSync0 = 0;
			config->VSync1 = 10;
			config->PCLK = 1;
			config->PClkFreq = 0x885; // 9.6MHz
			config->HsfWidth = 0;
		}
#endif
	}
	else if (display == EVE_DISPLAY_WVGA_800x480_74Hz && freq == 72000000)
	{
		/*
		BT81X known values:
		Display: MTF070TN83-V1
		REG_FREQUENCY: 72MHz
		Resolution: 800x480
		Refresh rate: 73.892Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 800;
			config->Height = 480;
			config->HCycle = 928;
			config->HOffset = 88;
			config->HSync0 = 0;
			config->HSync1 = 48;
			config->VCycle = 525;
			config->VOffset = 32;
			config->VSync0 = 0;
			config->VSync1 = 3;
			config->PCLK = 2;
			config->PClkFreq = 0;
			config->HsfWidth = 0;
		}
#endif
	}
	else if (display == EVE_DISPLAY_HDTV_1280x720_58Hz && freq == 72000000)
	{
		/*
		BT815 known values:
		REG_FREQUENCY: 72MHz
		Resolution: 1280x720
		Refresh rate: 58.182Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (supportedResolution)
		{
			config->Width = 1280;
			config->Height = 720;
			config->HCycle = 1650;
			config->HOffset = 40 + 220;
			config->HSync0 = 0;
			config->HSync1 = 40;
			config->VCycle = 750;
			config->VOffset = 5 + 20;
			config->VSync0 = 0;
			config->VSync1 = 5;
			config->PCLK = 1;
			config->PClkFreq = 0;
			config->HsfWidth = 0;
		}
#endif
		config->CSpread = 0;
		config->Dither = 0;
		config->PCLKPol = 0;
		config->OutBitsR = 0;
		config->OutBitsG = 0;
		config->OutBitsB = 0;
	}
	else if (display == EVE_DISPLAY_WXGA_1280x800_57Hz)
	{
		/*
		BT817 known values:
		Resolution: 1280x800
		Refresh rate: 57.393Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS

			config->Width = 1280;
			config->Height = 800;
			config->HCycle = 1440;
			config->HOffset = 160;
			config->HSync0 = 0;
			config->HSync1 = 2;
			config->VCycle = 822;
			config->VOffset = 15;
			config->VSync0 = 0;
			config->VSync1 = 2;
			config->PCLK = 1;
			config->PClkFreq = 0x8C1; 
			config->Dither = 1;

#endif
		config->CSpread = 0;
		config->PCLKPol = 0;
		config->OutBitsR = 0;
		config->OutBitsG = 0;
		config->OutBitsB = 0;
	}
	else if (display == EVE_DISPLAY_WSVGA_1024x600_59Hz)
	{
		/*
		BT817 known values:
		Resolution: 1024x600
		Refresh rate: 59.758Hz
		*/
#if EVE_HARDCODED_DISPLAY_TIMINGS
		if (EVE_CHIPID >= EVE_BT817 && supportedResolution)
		{
			config->Width = 1024;
			config->Height = 600;
			config->HCycle = 1344;
			config->HOffset = 160;
			config->HSync0 = 0;
			config->HSync1 = 100;
			config->VCycle = 635;
			config->VOffset = 23;
			config->VSync0 = 0;
			config->VSync1 = 10;
			config->PCLK = 1;
			config->PClkFreq = 0xD12; // 51Mhz
			config->HsfWidth = 0;
		}
#endif
		config->CSpread = 0;
		config->Dither = 1;
		config->PCLKPol = 1;
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

#if 0
	eve_printf_debug("Width: %i\n", (int)config->Width);
	eve_printf_debug("Height: %i\n", (int)config->Height);
	eve_printf_debug("HCycle: %i\n", (int)config->HCycle);
	eve_printf_debug("HOffset: %i\n", (int)config->HOffset);
	eve_printf_debug("HSync0: %i\n", (int)config->HSync0);
	eve_printf_debug("HSync1: %i\n", (int)config->HSync1);
	eve_printf_debug("VCycle: %i\n", (int)config->VCycle);
	eve_printf_debug("VOffset: %i\n", (int)config->VOffset);
	eve_printf_debug("VSync0: %i\n", (int)config->VSync0);
	eve_printf_debug("VSync1: %i\n", (int)config->VSync1);
	eve_printf_debug("PCLK: %i\n", (int)config->PCLK);
	eve_printf_debug("Swizzle: %i\n", (int)config->Swizzle);
	eve_printf_debug("PCLKPol: %i\n", (int)config->PCLKPol);
	eve_printf_debug("CSpread: %i\n", (int)config->CSpread);
	eve_printf_debug("OutBitsR: %i\n", (int)config->OutBitsR);
	eve_printf_debug("OutBitsG: %i\n", (int)config->OutBitsG);
	eve_printf_debug("OutBitsB: %i\n", (int)config->OutBitsB);
	eve_printf_debug("Dither: %i\n", (int)config->Dither);
	eve_printf_debug("AdaptiveFramerate: %i\n", (int)config->AdaptiveFramerate);
	eve_printf_debug("PClkFreq: %X\n", (int)config->PClkFreq);
	eve_printf_debug("AhHCycleMax: %i\n", (int)config->AhHCycleMax);
	eve_printf_debug("HsfWidth: %i\n", (int)config->HsfWidth);
#endif
}

#define EXTRACT_CHIPID(romChipId) EVE_extendedChipId((((romChipId) >> 8) & 0xFF) | (((romChipId) & (0xFF)) << 8))

EVE_HAL_EXPORT bool EVE_Util_bootup(EVE_HalContext *phost, EVE_BootupParameters *bootup)
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
		while (EXTRACT_CHIPID(chipId) < EVE_FT800
		    || EXTRACT_CHIPID(chipId) > EVE_BT818)
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

	/* Switch to the proper chip ID if applicable */

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
			if (EVE_HOST == EVE_HOST_BT8XXEMU) // FIXME: Emulator REG_CPURESET bits behaviour on FT800 and FT801 doesn't match
			{
				EVE_Hal_wr8(phost, REG_CPURESET, 0);
			}
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
	if (EVE_CHIPID >= EVE_FT810)
	{
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
	}

	/* Sanity check after SPI change */
	if (EVE_Hal_rd32(phost, REG_FREQUENCY) != freq)
	{
		eve_printf_debug("SPI channel configuration change failed\n");
		return false;
	}

	/* In some cases, for example, QSPI on MM817EV board with MM930LITE,
	   the drive strength is required to adjust for better signal integrity.
	   Setting to max current 20mA here is to play safe.  Users may adjust the
	   corresponding bits of REG_GPIOX according to their own situation. */
	if (EVE_CHIPID >= EVE_FT810)
	{
		EVE_Hal_wr16(phost, REG_GPIOX_DIR, 0xffff);
		EVE_Hal_wr16(phost, REG_GPIOX, 0xffff);
	}
	else
	{
		EVE_Hal_wr8(phost, REG_GPIO_DIR, 0xff);
		EVE_Hal_wr8(phost, REG_GPIO, 0xff);
	}

	/* Update touch firmware */
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 6285)
#endif
	if ((EVE_CHIPID == EVE_FT811 || EVE_CHIPID == EVE_FT813) && (EVE_HOST != EVE_HOST_BT8XXEMU))
#ifdef _MSC_VER
#pragma warning(pop)
#endif
	{
		/* Download new firmware to fix pen up issue */
		/* It may cause resistive touch not working any more*/
		uploadTouchFirmware(phost);
		EVE_Hal_flush(phost);
		EVE_sleep(100);
	}

	if ((EVE_CHIPID == EVE_FT800) && (EVE_HOST != EVE_HOST_BT8XXEMU))
#ifdef _MSC_VER
#pragma warning(pop)
#endif
	{
		uploadTouchFirmware(phost);
		EVE_Hal_flush(phost);
		EVE_sleep(100);
	}

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 6285)
#endif
	if ((EVE_CHIPID == EVE_BT815 || EVE_CHIPID == EVE_BT817) && (EVE_HOST != EVE_HOST_BT8XXEMU))
#ifdef _MSC_VER
#pragma warning(pop)
#endif
	{
		// Assume WXGA is using Goodix touch IC
		// To be fixed: use macro for touch configuration
		eve_printf_debug("Setup touch for Goodix\n");
		EVE_Hal_wr8(phost, REG_CPURESET, 2);
		EVE_Hal_wr16(phost, REG_TOUCH_CONFIG, 0x05d0);
		EVE_Hal_wr8(phost, REG_CPURESET, 0);
	}


	return true;
}

EVE_HAL_EXPORT bool EVE_Util_config(EVE_HalContext *phost, EVE_ConfigParameters *config)
{
	/* IMPORTANT: Do not use EVE_CoCmd functions here, as they can be overridden by hooks */

	bool reconfigure = phost->PCLK;
	if (reconfigure)
	{
		/* If PCLK is already set, we're reconfiguring. Quiet flush first */
		eve_printf_debug("Reconfiguring\n");
		EVE_Cmd_waitFlush(phost);
	}

	const bool swapXY = EVE_CHIPID >= EVE_FT810 ? EVE_Hal_rd8(phost, REG_ROTATE) & 0x2 : false;
	uint16_t wp;
	uint16_t rp;

	if (EVE_CHIPID < EVE_FT810)
	{
		eve_assert(config->Width < 512);
		eve_assert(config->Height < 512);
	}
	else
	{
		eve_assert(config->Width < 2048);
		eve_assert(config->Height < 2048);
	}

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
	if (EVE_CHIPID >= EVE_BT817)
	{
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
	}

	if (EVE_CHIPID >= EVE_BT815)
	{
		EVE_Hal_wr8(phost, REG_ADAPTIVE_FRAMERATE, config->AdaptiveFramerate);
	}

	if (EVE_CHIPID >= EVE_BT817)
	{
		EVE_Hal_wr16(phost, REG_AH_HCYCLE_MAX, config->AhHCycleMax);
	}


	EVE_Util_clearScreen(phost);

	/* Refresh fifo */
	wp = EVE_Cmd_wp(phost);
	rp = EVE_Cmd_rp(phost);
	EVE_Cmd_space(phost);
	if (EVE_CHIPID >= EVE_FT810)
	{
		phost->MediaFifoAddress = 0;
		phost->MediaFifoSize = 0;
	}

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

		/* Set the latest API level.
		CMD_APILEVEL applicable since BT817. */
		if (EVE_CHIPID >= EVE_BT817)
		{
			EVE_Cmd_wr32(phost, CMD_APILEVEL);
			EVE_Cmd_wr32(phost, 2);
		}

		if (EVE_CHIPID >= EVE_BT815)
		{
			/* Reattach flash to avoid inconsistent state */
			EVE_Cmd_wr32(phost, CMD_FLASHDETACH);
			EVE_Cmd_wr32(phost, CMD_FLASHATTACH);
		}

		EVE_CoDlImpl_resetCoState(phost);
	}

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

	EVE_Hal_flush(phost);
	if (!EVE_Cmd_waitFlush(phost))
		return false;

	EVE_Hal_wr8(phost, REG_PCLK, config->PCLK); /* After this display is visible on the LCD */
	phost->PCLK = config->PCLK;

	if (config->HsfWidth)
	{
		if (EVE_Hal_supportHsf(phost))
		{
			eve_printf_debug("HSize: %i, HSF HSize: %i\n", EVE_Hal_rd32(phost, REG_HSIZE), EVE_Hal_rd32(phost, REG_HSF_HSIZE));
		}
	}






#if 0
	eve_printf_debug("REG_HSIZE: %i\n", (int)EVE_Hal_rd16(phost, REG_HSIZE));
	eve_printf_debug("REG_VSIZE: %i\n", (int)EVE_Hal_rd16(phost, REG_VSIZE));
	eve_printf_debug("REG_HCYCLE: %i\n", (int)EVE_Hal_rd16(phost, REG_HCYCLE));
	eve_printf_debug("REG_HOFFSET: %i\n", (int)EVE_Hal_rd16(phost, REG_HOFFSET));
	eve_printf_debug("REG_HSYNC0: %i\n", (int)EVE_Hal_rd16(phost, REG_HSYNC0));
	eve_printf_debug("REG_HSYNC1: %i\n", (int)EVE_Hal_rd16(phost, REG_HSYNC1));
	eve_printf_debug("REG_VCYCLE: %i\n", (int)EVE_Hal_rd16(phost, REG_VCYCLE));
	eve_printf_debug("REG_VOFFSET: %i\n", (int)EVE_Hal_rd16(phost, REG_VOFFSET));
	eve_printf_debug("REG_VSYNC0: %i\n", (int)EVE_Hal_rd16(phost, REG_VSYNC0));
	eve_printf_debug("REG_VSYNC1: %i\n", (int)EVE_Hal_rd16(phost, REG_VSYNC1));
	eve_printf_debug("REG_PCLK: %i\n", (int)EVE_Hal_rd16(phost, REG_PCLK));
	eve_printf_debug("REG_SWIZZLE: %i\n", (int)EVE_Hal_rd16(phost, REG_SWIZZLE));
	eve_printf_debug("REG_PCLK_POL: %i\n", (int)EVE_Hal_rd16(phost, REG_PCLK_POL));
	eve_printf_debug("REG_CSPREAD: %i\n", (int)EVE_Hal_rd16(phost, REG_CSPREAD));
	eve_printf_debug("REG_OUTBITS: %i\n", (int)EVE_Hal_rd16(phost, REG_OUTBITS));
	eve_printf_debug("REG_DITHER: %i\n", (int)EVE_Hal_rd16(phost, REG_DITHER));
	eve_printf_debug("REG_ADAPTIVE_FRAMERATE: %i\n", (int)EVE_Hal_rd16(phost, REG_ADAPTIVE_FRAMERATE));
	eve_printf_debug("REG_PCLK_FREQ: %X\n", (int)EVE_Hal_rd16(phost, REG_PCLK_FREQ));
	eve_printf_debug("REG_AH_HCYCLE_MAX: %i\n", (int)EVE_Hal_rd16(phost, REG_AH_HCYCLE_MAX));
	eve_printf_debug("REG_HSF_HSIZE: %i\n", (int)EVE_Hal_rd16(phost, REG_HSF_HSIZE));
#endif

	eve_printf_debug("EVE configuration ready\n");
	return true;
}

EVE_HAL_EXPORT void EVE_Util_shutdown(EVE_HalContext *phost)
{
	if (EVE_CHIPID >= EVE_FT810)
	{
		EVE_Hal_wr16(phost, REG_GPIOX_DIR, 0xffff);
		EVE_Hal_wr16(phost, REG_GPIOX, 0);
	}
	else
	{
		EVE_Hal_wr8(phost, REG_GPIO_DIR, 0xff);
		EVE_Hal_wr8(phost, REG_GPIO, 0);
	}
	EVE_Hal_wr8(phost, REG_PCLK, 0);
	EVE_Hal_flush(phost);
	EVE_Hal_powerCycle(phost, false);
}

/*
Patch: OTP needs to be reactivated when the coprocessor is reset during CMD_LOGO
Applicable to: FT81X-series
*/
#define EVE_SUBPATCH_PTR 0x7ffeU /* NOTE: This address is only valid for FT81X */
static inline bool EVE_Util_needsSubPatch(EVE_HalContext *phost)
{
	return (EVE_CHIPID >= EVE_FT810) && (EVE_CHIPID <= EVE_FT813);
}

/*
Patch: Video patch from OTP needs to be reapplied after coprocessor reset
Applicable to: BT81X-series
*/
#define EVE_VIDEOPATCH_ADDR 0x309162UL /* NOTE: This address is only valid for BT815 and BT816 */
static inline bool EVE_Util_needsVideoPatch(EVE_HalContext *phost)
{
	return (EVE_CHIPID >= EVE_BT815) && (EVE_CHIPID <= EVE_BT816);
}

#if defined(_DEBUG)
/* Backup the last 128 bytes of RAM_G, which may be used for an error message */
void debugBackupRamG(EVE_HalContext *phost)
{
	if (!phost->DebugMessageVisible)
	{
		EVE_Hal_rdMem(phost, phost->DebugBackup, RAM_G + RAM_G_SIZE - sizeof(phost->DebugBackup), sizeof(phost->DebugBackup));
		phost->DebugMessageVisible = true;
	}
}
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
 * @param phost  Pointer to Hal context
 * @return true True if successful
 * @return false False if error
 */
EVE_HAL_EXPORT bool EVE_Util_resetCoprocessor(EVE_HalContext *phost)
{
	/* IMPORTANT: Do not use EVE_CoCmd functions here, as they can be overridden by hooks */

	const bool needsVideoPatch = EVE_Util_needsVideoPatch(phost);
	uint16_t videoPatchVector;
	bool ready;
	uint16_t rd;
	uint16_t wr;
	(void)rd;
	(void)wr;

	eve_printf_debug("Reset coprocessor\n");

	if (needsVideoPatch)
	{
		/* BT81X video patch */
		videoPatchVector = EVE_Hal_rd16(phost, EVE_VIDEOPATCH_ADDR);
	}

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

	if (EVE_Util_needsSubPatch(phost))
	{
		/* Enable patched rom in case the reset is requested while CMD_LOGO is running.
		This is necessary as CMD_LOGO may swap to the other rom page */
		EVE_Hal_wr8(phost, REG_ROMSUB_SEL, 3);
	}

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
	if (EVE_CHIPID >= EVE_FT810)
	{
		phost->MediaFifoAddress = 0;
		phost->MediaFifoSize = 0;
	}

	if (EVE_Util_needsSubPatch(phost))
	{
		/* Clear cmd with CMD_STOP, exiting CMD_EXECUTE may loop over, depending on OTP */
		EVE_Hal_startTransfer(phost, EVE_TRANSFER_WRITE, RAM_CMD);
		for (int i = 0; i < EVE_CMD_FIFO_SIZE; i += 4)
			EVE_Hal_transfer32(phost, CMD_STOP);
		EVE_Hal_endTransfer(phost);

		/* Go back into the patched coprocessor main loop */
		EVE_Hal_wr8(phost, REG_ROMSUB_SEL, 3);
		EVE_Cmd_startFunc(phost);
		EVE_Cmd_wr32(phost, (4294967047UL));
		EVE_Cmd_wr32(phost, EVE_SUBPATCH_PTR);
		EVE_Cmd_wr32(phost, 0);
		EVE_Cmd_endFunc(phost);
		EVE_Hal_flush(phost);
		eve_assert((wr = EVE_Hal_rd16(phost, REG_CMD_WRITE)) == 12);
		eve_assert((rd = EVE_Hal_rd16(phost, REG_CMD_READ)) == 0);

		/* Difficult to check when CMD_EXECUTE is processed when there's an OTP,
		since the read pointer keeps looping back to 0. */
		EVE_sleep(100);
		eve_assert((wr = EVE_Hal_rd16(phost, REG_CMD_WRITE)) == 12);
		eve_assert((rd = EVE_Hal_rd16(phost, REG_CMD_READ)) == 0);

		/* Need to manually stop previous command from repeating infinitely,
		however, this may cause the coprocessor to overshoot the command fifo,
		hence it's been filled with harmless CMD_STOP commands. */
		EVE_Hal_wr16(phost, REG_CMD_WRITE, 0);
		EVE_Hal_flush(phost);
		EVE_sleep(100);

		/* Refresh fifo */
		EVE_Cmd_waitFlush(phost);
		eve_assert((wr = EVE_Hal_rd16(phost, REG_CMD_WRITE)) == 0);
		eve_assert((rd = EVE_Hal_rd16(phost, REG_CMD_READ)) == 0);
	}

	if (needsVideoPatch)
	{
		/* BT81X video patch */
		EVE_Hal_wr16(phost, EVE_VIDEOPATCH_ADDR, videoPatchVector);
	}

	/* Cold start. Ensure that the coprocessor is ready. */
	EVE_Cmd_wr32(phost, CMD_DLSTART);
	EVE_Cmd_wr32(phost, CMD_COLDSTART);

	/* Set the latest API level.
	CMD_APILEVEL applicable since BT817. */
	if (EVE_CHIPID >= EVE_BT817)
	{
		EVE_Cmd_wr32(phost, CMD_APILEVEL);
		EVE_Cmd_wr32(phost, 2);
	}

	if (EVE_CHIPID >= EVE_BT815)
	{
		/* Reattach flash to avoid inconsistent state */
		EVE_Cmd_wr32(phost, CMD_FLASHATTACH);
	}

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
 * @param phost  Pointer to Hal context
 * @return true True if successful
 * @return false False if error
 */
EVE_HAL_EXPORT bool EVE_Util_bootupConfig(EVE_HalContext *phost)
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

/**********************
** INTERACTIVE SETUP **
**********************/

#if (defined(_WIN32) || defined(__linux__))

void EVE_Util_selectDeviceInteractive(EVE_CHIPID_T *chipId, size_t *deviceIdx)
{
	char buf[1024];
	EVE_DeviceInfo info;
	int selectedDeviceIdx;
	size_t deviceCount;
	size_t i;

SELECTDEVICE:
	deviceCount = EVE_Hal_list();
	size_t realDeviceCount = 0;
	for (i = 0; i < deviceCount; ++i)
	{
		EVE_Hal_info(&info, i);
		if (info.Host)
		{
			++realDeviceCount;
			*deviceIdx = i;
		}
	}
	if (realDeviceCount > 1)
	{
		buf[0] = '\0';
		printf("Select a device:\n");
		for (i = 0; i < deviceCount; ++i)
		{
			EVE_Hal_info(&info, i);
			if (info.Host)
				printf("- [%d] %s (%s, %s)\n", (unsigned int)i, info.DisplayName, s_HostDisplayNames[info.Host], info.SerialNumber);
		}
		fgets(buf, sizeof(buf), stdin);
#ifdef _WIN32
		if (sscanf_s(buf, "%i", &selectedDeviceIdx) != 1)
#else
		if (sscanf(buf, "%i", &selectedDeviceIdx) != 1)
#endif
			goto SELECTDEVICE;
		*deviceIdx = selectedDeviceIdx;
		EVE_Hal_info(&info, *deviceIdx);
		if (!info.Host)
			goto SELECTDEVICE;
		printf("\n");
	}
	else if (realDeviceCount > 0)
	{
		EVE_Hal_info(&info, *deviceIdx);
		printf("%s (%s, %s)\n", info.DisplayName, s_HostDisplayNames[info.Host], info.SerialNumber);
	}
	else
	{
		*deviceIdx = -1;
		*chipId = EVE_SUPPORT_CHIPID;
		return;
	}

	*chipId = EVE_SUPPORT_CHIPID;
}


#endif

#if defined(_WIN32) || defined(__linux__)
void EVE_Util_selectFlashFileInteractive(eve_tchar_t *flashPath, size_t flashPathSize, bool *updateFlash, bool *updateFlashFirmware, const EVE_HalParameters *params, const eve_tchar_t *flashFile)
{
	size_t flashPathSz;
#ifdef _WIN32
	errno_t ferr;
#endif
	FILE *f = NULL;

SELECTFLASH:
	*updateFlash = false;
	*updateFlashFirmware = false;
	if (flashFile)
	{
		uint8_t buffer[4096];
		{
			/* Query user if they want to update the flash file on the device */
			printf("Upload flash image (y/n, or press ENTER to skip):\n");
			buffer[0] = '\0';
			fgets((char *)buffer, sizeof(buffer), stdin);
			/* Fast string to bool, reliably defined for strings starting
			with 0, 1, t, T, f, F, y, Y, n, N, anything else is undefined. */
			*updateFlash = (buffer[0] == '1' || (buffer[0] & 0xD2) == 0x50);
			printf("\n");
			if (*updateFlash)
			{
				printf("Upload flash firmware (y/n, or press ENTER to skip):\n");
				buffer[0] = '\0';
				fgets((char *)buffer, sizeof(buffer), stdin);
				*updateFlashFirmware = (buffer[0] == '1' || (buffer[0] & 0xD2) == 0x50);
				printf("\n");
			}
		}
		/* When uploading, or under emulator when the default flash is specified,
		offer to confirm which flash file will be used. */
		if (*updateFlash
		)
		{
			/* Query user for any changes to the flash file name */
#ifdef _WIN32
			printf("Select flash file %s(or press ENTER to use %s%ls%s):\n",
			    *updateFlashFirmware ? "with firmware " : "",
			    flashFile[0] ? "\"" : "", /* Quote */
			    flashFile[0] ? flashFile : L"no flash",
			    flashFile[0] ? "\"" : ""); /* Quote */
#else
			printf("Select flash file %s(or press ENTER to use %s%s%s):\n",
			    *updateFlashFirmware ? "with firmware " : "",
			    flashFile[0] ? "\"" : "", /* Quote */
			    flashFile[0] ? flashFile : "no flash",
			    flashFile[0] ? "\"" : ""); /* Quote */
#endif

#ifdef _WIN32
			fgetws(flashPath, MAX_PATH, stdin);
#else
			fgets(flashPath, MAX_PATH, stdin);
#endif
			if (flashPath[0] == '\r' || flashPath[0] == '\n')
				flashPath[0] = '\0';
#ifdef _WIN32
			flashPathSz = wcslen(flashPath);
#else
			flashPathSz = strlen(flashPath);
#endif
			while (flashPathSz && (flashPath[flashPathSz - 1] == '\r' || flashPath[flashPathSz - 1] == '\n'))
			{
				/* Trim flash path */
				flashPath[flashPathSz - 1] = '\0';
				--flashPathSz;
			}

			if (flashFile[0] == '\0' && flashPath[0] == '\0')
			{
				/* No flash */
				printf("\n");
				return;
			}

			/* Check if this file can be opened */
#ifdef _WIN32
			ferr = _wfopen_s(&f, flashPath[0] ? flashPath : flashFile, L"rb");
#else
			f = fopen(flashPath[0] ? flashPath : flashFile, "rb");
#endif
			if (
#ifdef _WIN32
			    ferr ||
#endif
			    !f)
			{
				printf("File \"%ls\" cannot be opened\n", flashPath[0] ? flashPath : flashFile);
				goto SELECTFLASH;
			}
			fseek(f, 0, SEEK_END);
			fclose(f);
			f = NULL;
			printf("\n");
		}
		if (*updateFlash
		)
		{
			if (!flashPath[0])
			{
#ifdef _WIN32
				wcscpy_s(flashPath, flashPathSize, flashFile);
#else
				strncpy(flashPath, flashFile, flashPathSize - 1);
				flashPath[flashPathSize - 1] = '\0';
#endif
			}
		}
	}
}
#endif


#if defined(_WIN32) || defined(__linux__)
#pragma warning(push)
#pragma warning(disable : 6262) // Large stack due to buffer
EVE_HAL_EXPORT void EVE_Util_uploadFlashFileInteractive(EVE_HalContext *phost, const eve_tchar_t *flashPath, bool updateFlashFirmware)
{
	ptrdiff_t flashSize;
	FILE *f = NULL;
	uint8_t buffer[64 * 4096];
	uint8_t rbuffer[64 * 4096];
#ifdef _WIN32
	errno_t err = 0;
#endif

	/* Upload flash */

	EVE_BootupParameters bootupParams;
	printf("Preparing to upload flash...\n");

	/* Open flash file and get size */
#ifdef _WIN32
	err = _wfopen_s(&f, flashPath, L"rb");
#else
	f = fopen(flashPath, "rb");
#endif

	if (
#ifdef _WIN32
	    err ||
#endif
	    !f)
	{
		printf("Flash file cannot be opened\n");
	}
	else
	{

		fseek(f, 0, SEEK_END);
		flashSize = ftell(f);
		fseek(f, 0, SEEK_SET);

		/* Get the default bootup parameters for the device */
		EVE_Util_bootupDefaults(phost, &bootupParams);

		if (EVE_Util_bootup(phost, &bootupParams))
		{
			/* Get the default bootup parameters for the device */
			EVE_ConfigParameters configParams;
			EVE_Util_configDefaults(phost, &configParams, EVE_DISPLAY_DEFAULT);

			/* No display */
			configParams.PCLK = 0;

			/* Boot up */
			if (EVE_Util_config(phost, &configParams))
			{
				do
				{
					uint32_t flashStatus;
					uint32_t flashDeviceSize;
					size_t remainingBytes;
					uint32_t flashAddr;

					if (!EVE_Hal_supportFlash(phost))
					{
						printf("This device doesn't support flash\n");
						break;
					}

					flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
					if (flashStatus == FLASH_STATUS_DETACHED)
					{
						EVE_Cmd_wr32(phost, CMD_FLASHATTACH);
						if (!EVE_Cmd_waitFlush(phost)) /* Wait for command completion */
						{
							printf("Coprocessor fault\n");
							break;
						}
					}

					flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
					if (flashStatus < FLASH_STATUS_BASIC)
					{
						printf("Flash could not be attached\n");
						break;
					}

					if (updateFlashFirmware)
					{
						printf("Upload flash firmware from image...\n");
						if (fread(buffer, 4096, 1, f) != 1)
						{
							printf("Could not read file\n");
							break;
						}
						EVE_Hal_wrMem(phost, 0, buffer, 4096);
						EVE_Cmd_startFunc(phost);
						EVE_Cmd_wr32(phost, CMD_FLASHUPDATE);
						EVE_Cmd_wr32(phost, 0);
						EVE_Cmd_wr32(phost, 0);
						EVE_Cmd_wr32(phost, 4096);
						EVE_Cmd_endFunc(phost);
						if (!EVE_Cmd_waitFlush(phost)) /* Wait for command completion */
						{
							printf("Coprocessor fault\n");
							break;
						}
					}
					else
					{
						if (fseek(f, 4096, SEEK_CUR))
						{
							printf("Could not seek file\n");
						}
					}

					if (flashStatus == FLASH_STATUS_BASIC)
					{
						uint32_t resAddr;
						printf("Enter fast flash mode\n");
						EVE_Cmd_startFunc(phost);
						EVE_Cmd_wr32(phost, CMD_FLASHFAST);
						resAddr = EVE_Cmd_moveWp(phost, 4); /* Get the address where the coprocessor will write the result */
						EVE_Cmd_endFunc(phost);
						if (!EVE_Cmd_waitFlush(phost)) /* Wait for command completion */
						{
							printf("Coprocessor fault\n");
							break;
						}
						EVE_Hal_rd32(phost, RAM_CMD + resAddr); /* Fetch result */
					}

					flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
					if (flashStatus < FLASH_STATUS_FULL)
					{
						printf("Flash could not enter fast mode. Has the correct firmware been uploaded?\n");
						break;
					}

					flashDeviceSize = EVE_Hal_rd32(phost, REG_FLASH_SIZE) * 1024 * 1024;
					if (flashDeviceSize < flashSize)
					{
						printf("Not enough space on flash, need %i bytes, have %i bytes\n", (int)flashSize, (int)flashDeviceSize);
						break;
					}

					remainingBytes = flashSize - 4096;
					flashAddr = 4096;

					printf("Upload flash from image...\n");
					eve_assert(sizeof(buffer) >= 4096);

					while (remainingBytes)
					{
						/* Read from file */
						size_t el;
						size_t sz;
						if (remainingBytes < 4096)
						{
							el = fread(buffer, 1, remainingBytes, f);
							sz = 4096;
							remainingBytes = 0;
						}
						else
						{
							el = fread(buffer, 4096, min(remainingBytes, sizeof(buffer)) / 4096, f);
							sz = el * 4096;
							remainingBytes -= sz;
						}
						if (!el)
						{
							printf("\nFailed to read file\n");
							break;
						}
						printf("B");

						/* Write to flash */
					REWRITE:
						EVE_Hal_wrMem(phost, 0, buffer, (uint32_t)sz);
						EVE_Cmd_startFunc(phost);
						EVE_Cmd_wr32(phost, CMD_FLASHUPDATE);
						EVE_Cmd_wr32(phost, flashAddr);
						EVE_Cmd_wr32(phost, 0);
						EVE_Cmd_wr32(phost, (uint32_t)sz);
						EVE_Cmd_endFunc(phost);
						if (!EVE_Cmd_waitFlush(phost)) /* Wait for command completion */
						{
							printf("\nCoprocessor fault\n");
							break;
						}
						printf("R");

						/* Verify using CMD_FLASHREAD */
						EVE_Hal_startTransfer(phost, EVE_TRANSFER_WRITE, 0);
						for (size_t i = 0; i < sz; ++i)
							EVE_Hal_transfer8(phost, 0x0F);
						EVE_Hal_endTransfer(phost);
						EVE_Cmd_startFunc(phost);
						EVE_Cmd_wr32(phost, CMD_FLASHREAD);
						EVE_Cmd_wr32(phost, 0);
						EVE_Cmd_wr32(phost, flashAddr);
						EVE_Cmd_wr32(phost, (uint32_t)sz);
						EVE_Cmd_endFunc(phost);
						if (!EVE_Cmd_waitFlush(phost)) /* Wait for command completion */
						{
							printf("\nCoprocessor fault\n");
							break;
						}
						EVE_Hal_rdMem(phost, rbuffer, 0, (uint32_t)sz);
						for (size_t i = 0; i < sz; ++i)
						{
							if (buffer[i] != rbuffer[i])
							{
								printf("\nVerification failed\n");
								goto REWRITE;
							}
						}
						printf("T");

						flashAddr += (uint32_t)sz;
					}

					if (!remainingBytes)
					{
						printf("\nFlash upload is ready\n");
					}
					printf("\n");
				} while (false); /* breakable scope */
			}
			else
			{
				printf("Failed to bootup the device\n");
			}

			/* Shutdown */
			EVE_Util_shutdown(phost);
		}
		else
		{
			printf("Failed to bootup the device\n");
		}
	}
	printf("\n");

	if (f)
	{
		fclose(f);
	}
}
#pragma warning(pop)
#endif

bool EVE_Util_openDeviceInteractive(EVE_HalContext *phost, const eve_tchar_t *flashFile)
{
	EVE_CHIPID_T chipId;
	size_t deviceIdx;
	bool opened;

	EVE_HalParameters params = { 0 };
#if defined(_WIN32) || defined(__linux__)
	bool updateFlash = false;
	bool updateFlashFirmware = false;
	eve_tchar_t flashPath[MAX_PATH];
	flashPath[0] = '\0';
#endif

	/* Interactive device selection */
	EVE_Util_selectDeviceInteractive(&chipId, &deviceIdx);

	/* Fetch the default parameters for a device. Set the expected chip id.
	Pass the device index, or -1 to select the first device */
	EVE_Hal_defaultsEx(&params, deviceIdx);

#if defined(_WIN32) || defined(__linux__)
	if (chipId >= EVE_BT815 || (chipId <= 0 && flashFile && flashFile[0]))
		EVE_Util_selectFlashFileInteractive(flashPath, sizeof(flashPath) / sizeof(flashPath[0]), &updateFlash, &updateFlashFirmware, &params, flashFile);
#endif


	opened = EVE_Hal_open(phost, &params);

	if (!opened)
		return false;

#if defined(_WIN32) || defined(__linux__)
	if (updateFlash && flashPath[0])
		EVE_Util_uploadFlashFileInteractive(phost, flashPath, updateFlashFirmware);
#endif

	return true;
}

/* Calls EVE_Util_bootup and EVE_Util_config using the default parameters.
Falls back to no interactivity on FT9XX platform */
EVE_HAL_EXPORT bool EVE_Util_bootupConfigInteractive(EVE_HalContext *phost, EVE_DISPLAY_T display)
{
	EVE_DISPLAY_T selectedDisplay;
	EVE_BootupParameters bootup;
	EVE_ConfigParameters config;

	EVE_Util_bootupDefaults(phost, &bootup);
	if (!EVE_Util_bootup(phost, &bootup))
	{
		return false;
	}

	EVE_Util_selectDisplayInteractive(&selectedDisplay);
	if (!selectedDisplay)
		selectedDisplay = display;

	EVE_Util_configDefaults(phost, &config, selectedDisplay);
	if (!EVE_Util_config(phost, &config))
	{
		EVE_Util_shutdown(phost);
		return false;
	}

	
	return true;
}

void EVE_Util_forceFault(EVE_HalContext *phost, const char *err)
{
	/* Check if we're already in fault state */
	if (!EVE_Cmd_waitSpace(phost, 0))
		return;
#if 1
	/* Go into reset state and mimic a fault */
	EVE_Hal_wr8(phost, REG_CPURESET, 1);
	EVE_Hal_flush(phost);
	EVE_sleep(100);
	EVE_Hal_wr16(phost, REG_CMD_READ, 0xFFF);
	EVE_Hal_wr16(phost, REG_CMD_WRITE, 0xFFF);
#else
	/* TODO: Clean this up a bit */
	if (!EVE_Util_resetCoprocessor(phost))
		return;
	EVE_CoCmd_dlStart(phost);
	for (int i = 0; i < EVE_DL_COUNT; ++i)
		EVE_Cmd_wr32(phost, DISPLAY());
	EVE_Cmd_waitFlush(phost);
	EVE_Cmd_wr32(phost, DISPLAY());
	EVE_Hal_flush(phost);
	EVE_sleep(100);
#endif
	if (EVE_CHIPID >= EVE_BT815 && (EVE_Cmd_rp(phost) & 0x3))
	{
		/* Write our own error to the error report area */
		uint32_t errLen = (uint32_t)strlen(err);
		EVE_Hal_wrMem(phost, RAM_ERR_REPORT, (const uint8_t *)err, errLen);
		if (errLen < RAM_ERR_REPORT_MAX)
		{
			EVE_Hal_startTransfer(phost, EVE_TRANSFER_WRITE, RAM_ERR_REPORT + errLen);
			for (int i = (int)errLen; i < RAM_ERR_REPORT_MAX; ++i)
				EVE_Hal_transfer8(phost, 0);
			EVE_Hal_endTransfer(phost);
		}
	}
	/* Fault state now */
	EVE_Cmd_waitFlush(phost);
#if 0
#if defined(_DEBUG)
	if (EVE_CHIPID >= EVE_BT815
	    && phost->DebugMessageVisible)
	{
		uint32_t errLen = (uint32_t)strlen(err);
		EVE_Hal_wrMem(phost, RAM_G + RAM_G_SIZE - sizeof(phost->DebugBackup), (const uint8_t *)err, errLen);
		if (errLen < sizeof(phost->DebugBackup))
		{
			EVE_Hal_startTransfer(phost, EVE_TRANSFER_WRITE, RAM_G + RAM_G_SIZE - sizeof(phost->DebugBackup) + errLen);
			for (int i = (int)errLen; i < sizeof(phost->DebugBackup); ++i)
				EVE_Hal_transfer8(phost, 0);
			EVE_Hal_endTransfer(phost);
		}
	}
#endif
#endif
}

/* end of file */
