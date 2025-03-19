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

#ifndef EVE_CONFIG__H
#define EVE_CONFIG__H

#include "EVE_IntTypes.h"

/*

This file processes the input definitions,
the available list of which is specified further below for ESD using these macros.
- ESD_TARGET_GRAPHICS(definition, ...)
- ESD_TARGET_DISPLAY(definition, ...)
- ESD_TARGET_PLATFORM(definition, ...)
- ESD_TARGET_FLASH(definition, ...)

*/

#define ESD_TARGET_GRAPHICS(name, ...)
#define ESD_TARGET_DISPLAY(name, ...)
#define ESD_TARGET_PLATFORM(name, ...)
#define ESD_TARGET_FLASH(name, ...)
#define ESD_TARGET_TOUCH(name, ...)
#define ESD_TARGET_SPICHANNEL(name, ...)

// Chip IDs with EVE generation
#define EVE_FT800 0x10800
#define EVE_FT801 0x10801
#define EVE_FT810 0x20810
#define EVE_FT811 0x20811
#define EVE_FT812 0x20812
#define EVE_FT813 0x20813
#define EVE_BT880 0x20880
#define EVE_BT881 0x20881
#define EVE_BT882 0x20882
#define EVE_BT883 0x20883
#define EVE_BT815 0x30815
#define EVE_BT816 0x30816
#define EVE_BT817 0x40817
#define EVE_BT818 0x40818

// EVE generations
#define EVE1 1 /* FT800 and FT801 */
#define EVE2 2 /* FT810 to FT813, and BT880 to BT883 */
#define EVE3 3 /* BT815 and BT816 */
#define EVE4 4 /* BT817 and BT818 */

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/*

Uses regex matching for supported targets.
See https://regexr.com/ for help on regex.

To match only PANL70:
\bPANL70\b
To match platforms starting with MM9
\bMM9
\bMM9\w*\b
Match anything (default):
\b\w+\b
Match nothing:
(?=a)b

IntegratedPlatform specifies that the platform chip is part of the target, the specified string is a display name
IntegratedFlash specifies that the flash chip is part of the target, the specified string is a display name
OverrideSupport specifies that the supported flash of the target overrides the Supported option
of any other target, overriding the Integrated options as well

Example:
SupportedPlatforms = "8XXEMU|\bMM9|FT4222|MPSSE"
SupportedFlash = "\bW25Q\w*\b|\bMX25L\w*\b

*/

ESD_TARGET_GRAPHICS(ME810A_HV35R, DisplayName = "ME810A-HV35R", IntegratedDisplay = "ILI9488 (320x480)", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets = "\b(FT81X|FT810)\b")
ESD_TARGET_GRAPHICS(ME812A_WH50R, DisplayName = "ME812A-WH50R", SupportedDisplays = "\b\w+WVGA\w*\b", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets = "\b(FT81X|FT812)\b")
ESD_TARGET_GRAPHICS(ME813A_WH50C, DisplayName = "ME813A-WH50C", SupportedDisplays = "\b\w+WVGA\w*\b", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets = "\b(FT81X|FT813)\b")
// ESD_TARGET_GRAPHICS(ME810AU_HV35R, DisplayName = "ME810AU-HV35R", IntegratedDisplay = "ILI9488 (320x480)", IntegratedPlatform = "FT4222", SupportedArchitectures = "\bFT4222\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT810)\b")
// ESD_TARGET_GRAPHICS(ME812AU_WH50R, DisplayName = "ME812AU-WH50R", SupportedDisplays = "\b\w+WVGA\w*\b", IntegratedPlatform = "FT4222", SupportedArchitectures = "\bFT4222\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT812)\b")
// ESD_TARGET_GRAPHICS(ME813AU_WH50C, DisplayName = "ME813AU-WH50C", SupportedDisplays = "\b\w+WVGA\w*\b", IntegratedPlatform = "FT4222", SupportedArchitectures = "\bFT4222\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT813)\b")
// ESD_TARGET_GRAPHICS(PANL35, DisplayName = "PanL35", IntegratedDisplay = "KD2401 (320x480)", IntegratedPlatform = "FT903", SupportedArchitectures = "\bFT32\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT811)\b")
// ESD_TARGET_GRAPHICS(PANL50, DisplayName = "PanL50", SupportedDisplays = "\b\w+WVGA\w*\b", IntegratedPlatform = "MM900EV1B", SupportedArchitectures = "\bFT32\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT813)\b")
// ESD_TARGET_GRAPHICS(PANL70, DisplayName = "PanL70", IntegratedDisplay = "WVGA (800x480)", IntegratedPlatform = "FT930", SupportedArchitectures = "\bFT32\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT811)\b")
// ESD_TARGET_GRAPHICS(PANL70PLUS, DisplayName = "PanL70 Plus", IntegratedDisplay = "WVGA (800x480)", IntegratedPlatform = "FT930", SupportedArchitectures = "\bFT32\b", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT81X|FT811)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_VM800C, DisplayName = "VM800C", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT80X|FT800)\b")
ESD_TARGET_GRAPHICS(VM800B35A_BK, DisplayName = "VM800B35A-BK", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT800)\b")
ESD_TARGET_GRAPHICS(VM800B43A_BK, DisplayName = "VM800B43A-BK", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT800)\b")
ESD_TARGET_GRAPHICS(VM800B50A_BK, DisplayName = "VM800B50A-BK", SupportedFlash = "(?=a)b", LibraryTargets = "\b(FT800)\b")
ESD_TARGET_GRAPHICS(EVE_GRAPHICS_VM810C, DisplayName = "VM810C", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets = "\b(FT81X|FT810)\b")
ESD_TARGET_GRAPHICS(EVE_GRAPHICS_VM816C, DisplayName = "VM816C", IntegratedFlash = "W25Q128", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets = "\b(BT81X|BT816)\b", FirmwareFolder = "BT815")
ESD_TARGET_GRAPHICS(EVE_GRAPHICS_ME817EV, DisplayName = "ME817EV", IntegratedFlash = "W25Q128", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", LibraryTargets = "\b(BT81X|BT818)\b", FirmwareFolder = "BT817")

ESD_TARGET_GRAPHICS(EVE_GRAPHICS_GD3X_DAZZLER, DisplayName = "GD3X Dazzler", IntegratedDisplay = "HDMI 720p (1280x720)", IntegratedFlash = "W25Q64", SupportedTouch = "\b\w+DISABLED\w*\b", LibraryTargets = "\b(BT81X|BT815)\b", FirmwareFolder = "BT815")
ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDM2040, DisplayName = "IDM2040-7A (BT817)", IntegratedFlash = "W25Q128", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", IntegratedPlatform = "RP2040", LibraryTargets = "\b(BT81X|BT818)\b", FirmwareFolder = "BT817")
ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDP_1000_04A, DisplayName = "IDP-1000-04A (BT817)", IntegratedFlash = "W25Q128", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", IntegratedPlatform = "RP2040", LibraryTargets = "\b(BT81X|BT818)\b", FirmwareFolder = "BT817")

ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDP_4000_04A, DisplayName = "IDP-4000-04A (BT815)", IntegratedFlash = "W25Q128", SupportedTouch = "\b\w+GOODIX\w*\b", IntegratedPlatform = "RP2040", LibraryTargets = "\b(BT81X|BT815)\b", FirmwareFolder = "BT815")

ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDP_3500_04A, DisplayName = "IDP-3500-04A (FT903)", IntegratedDisplay = "HVGA (320x480)", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets = "\b(FT81X|FT811)\b")

ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDP_5000_04A, DisplayName = "IDP-5000-04A (FT903)", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets = "\b(FT81X|FT813)\b")


ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDM2040_43A, DisplayName = "IDM2040-43A (BT883)", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", IntegratedPlatform = "RP2040", LibraryTargets = "\b(FT81X|FT813)\b", FirmwareFolder = "-")

ESD_TARGET_GRAPHICS(EVE_GRAPHICS_IDM2040_21R, DisplayName = "IDM2040-21R (FT800)", SupportedFlash = "(?=a)b", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", IntegratedPlatform = "RP2040", LibraryTargets = "\b(FT80X|FT800)\b", FirmwareFolder = "-")

// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT800, DisplayName = "FT800 (Generic)", SupportedDisplays = "\b\w+(QVGA|HVGA|AT043B35)\w*\b", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets="\b(FT80X|FT800)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT801, DisplayName = "FT801 (Generic)", SupportedDisplays = "\b\w+(QVGA|HVGA|AT043B35)\w*\b", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets="\b(FT80X|FT801)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT810, DisplayName = "FT810 (Generic)", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets="\b(FT81X|FT810)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT811, DisplayName = "FT811 (Generic)", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets="\b(FT81X|FT811)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT812, DisplayName = "FT812 (Generic)", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets="\b(FT81X|FT812)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_FT813, DisplayName = "FT813 (Generic)", SupportedTouch = "\b\w+FOCAL\w*\b", LibraryTargets="\b(FT81X|FT813)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_BT815, DisplayName = "BT815 (Generic)", SupportedTouch = "\b\w+FOCAL\w*\b|\b\w+GOODIX\w*\b", LibraryTargets="\b(BT81X|BT815)\b")
// ESD_TARGET_GRAPHICS(EVE_GRAPHICS_BT816, DisplayName = "BT816 (Generic)", SupportedTouch = "\b\w+RESISTIVE\w*\b", LibraryTargets="\b(BT81X|BT816)\b")

/* Landscape */
ESD_TARGET_DISPLAY(EVE_DISPLAY_QVGA, DisplayName = "QVGA (320x240)")

ESD_TARGET_DISPLAY(EVE_DISPLAY_WQVGA, DisplayName = "WQVGA (480x272)")
ESD_TARGET_DISPLAY(EVE_DISPLAY_WVGA, DisplayName = "WVGA (800x480)")
ESD_TARGET_DISPLAY(EVE_DISPLAY_WVGA_NS, DisplayName = "WVGA (800x480) 1.792:1") // pixel aspect ratio 1.792:1 non square
ESD_TARGET_DISPLAY(EVE_DISPLAY_WSVGA, DisplayName = "WSVGA (1024x600)")
ESD_TARGET_DISPLAY(EVE_DISPLAY_WXGA, DisplayName = "WXGA (1280x800)")
ESD_TARGET_DISPLAY(EVE_DISPLAY_WQVGA_SQ, DisplayName = "WQVGA (480x480)")
// ESD_TARGET_DISPLAY(EVE_DISPLAY_AT043B35, DisplayName = "AT043B35 (480x272)")

/* Portrait */
// ESD_TARGET_DISPLAY(EVE_DISPLAY_ILI9488_HVGA_PORTRAIT, DisplayName = "ILI9488 (320x480)")
// ESD_TARGET_DISPLAY(EVE_DISPLAY_KD2401_HVGA_PORTRAIT, DisplayName = "KD2401 (320x480)")

ESD_TARGET_PLATFORM(EVE_PLATFORM_BT8XXEMU, DisplayName = "Emulator", Icon = ":/icons/game-monitor.png", OverrideSupport, SupportedFlash = "\b\w*_W25Q\w*\b|\b\w*_MX25L\w*\b", SupportedArchitectures = "\bBT8XXEMU\b")
ESD_TARGET_PLATFORM(MM900EV1A, SupportedArchitectures = "\bFT32\b")
ESD_TARGET_PLATFORM(MM900EV1B, SupportedArchitectures = "\bFT32\b")
ESD_TARGET_PLATFORM(MM900EV2A, SupportedArchitectures = "\bFT32\b")
ESD_TARGET_PLATFORM(MM900EV3A, SupportedArchitectures = "\bFT32\b")
ESD_TARGET_PLATFORM(MM900EV_LITE, DisplayName = "MM900EV-Lite", SupportedArchitectures = "\bFT32\b")
// ESD_TARGET_PLATFORM(MM930MINI, DisplayName = "MM930Mini", SupportedArchitectures = "\bFT32\b")
// ESD_TARGET_PLATFORM(MM930LITE, DisplayName = "MM930Lite", SupportedArchitectures = "\bFT32\b")
// ESD_TARGET_PLATFORM(MM932LC, DisplayName = "MM932LC", SupportedArchitectures = "\bFT32\b")
ESD_TARGET_PLATFORM(EVE_PLATFORM_FT4222, DisplayName = "FT4222", Icon = ":/icons/terminal.png", SupportedArchitectures = "\bFT4222\b")
ESD_TARGET_PLATFORM(EVE_PLATFORM_MPSSE, DisplayName = "MPSSE", Icon = ":/icons/terminal.png", SupportedArchitectures = "\bMPSSE\b")
ESD_TARGET_PLATFORM(EVE_PLATFORM_RP2040, DisplayName = "Raspberry Pi Pico", SupportedArchitectures = "\bPICO\b")
ESD_TARGET_PLATFORM(EVE_PLATFORM_MM2040EV, DisplayName = "MM2040EV", SupportedArchitectures = "\bPICO\b")
ESD_TARGET_PLATFORM(EVE_PLATFORM_STM32, DisplayName = "STM32F401CCU", SupportedArchitectures = "\bSTM32\b")

ESD_TARGET_FLASH(EVE_FLASH_W25Q16, DisplayName = "W25Q16")
ESD_TARGET_FLASH(EVE_FLASH_W25Q32, DisplayName = "W25Q32")
ESD_TARGET_FLASH(EVE_FLASH_W25Q64, DisplayName = "W25Q64")
ESD_TARGET_FLASH(EVE_FLASH_W25Q128, DisplayName = "W25Q128")

ESD_TARGET_FLASH(EVE_FLASH_MX25L16, DisplayName = "MX25L16")
ESD_TARGET_FLASH(EVE_FLASH_MX25L32, DisplayName = "MX25L32")
ESD_TARGET_FLASH(EVE_FLASH_MX25L64, DisplayName = "MX25L64")
ESD_TARGET_FLASH(EVE_FLASH_MX25L128, DisplayName = "MX25L128")
ESD_TARGET_FLASH(EVE_FLASH_MX25L256, DisplayName = "MX25L256", FlashFirmware = "unified.blob")
ESD_TARGET_FLASH(EVE_FLASH_MX25L512, DisplayName = "MX25L512", FlashFirmware = "unified.blob")
ESD_TARGET_FLASH(EVE_FLASH_MX25L1024, DisplayName = "MX25L1024", FlashFirmware = "unified.blob")
ESD_TARGET_FLASH(EVE_FLASH_MX25L2048, DisplayName = "MX25L2048", FlashFirmware = "unified.blob")

ESD_TARGET_TOUCH(EVE_TOUCH_FOCAL, DisplayName = "FocalTech")
ESD_TARGET_TOUCH(EVE_TOUCH_GOODIX, DisplayName = "Goodix")
ESD_TARGET_TOUCH(EVE_TOUCH_RESISTIVE, DisplayName = "Resistive")
ESD_TARGET_TOUCH(EVE_TOUCH_DISABLED, DisplayName = "Disabled")

ESD_TARGET_SPICHANNEL(EVE_SPICHANNEL_SINGLE, DisplayName = "SPI Single Channel")
ESD_TARGET_SPICHANNEL(EVE_SPICHANNEL_QUAD, DisplayName = "SPI Quad Channels")

/*

The following values are set based on the input definitions.
Do not set these values manually; instead, use the specific user definitions.

Graphics target:
- EVE_SUPPORT_CHIPID=EVE_FT800
- EVE_SUPPORT_CHIPID=EVE_FT801
- EVE_SUPPORT_CHIPID=EVE_FT810
- EVE_SUPPORT_CHIPID=EVE_FT811
- EVE_SUPPORT_CHIPID=EVE_FT812
- EVE_SUPPORT_CHIPID=EVE_FT813
- EVE_SUPPORT_CHIPID=EVE_BT880
- EVE_SUPPORT_CHIPID=EVE_BT881
- EVE_SUPPORT_CHIPID=EVE_BT882
- EVE_SUPPORT_CHIPID=EVE_BT883
- EVE_SUPPORT_CHIPID=EVE_BT815
- EVE_SUPPORT_CHIPID=EVE_BT816
- EVE_SUPPORT_CHIPID=EVE_BT817
- EVE_SUPPORT_CHIPID=EVE_BT818

Platform target:
- BT8XXEMU_PLATFORM (set EVE_PLATFORM_BT8XXEMU)
- FT9XX_PLATFORM (set MM900EV1A, etc.)
- FT4222_PLATFORM (set EVE_PLATFORM_FT4222)
- MPSSE_PLATFORM (set EVE_PLATFORM_MPSSE)
- RP2040_PLATFORM (set EVE_PLATFORM_RP2040)
- STM32_PLATFORM (set EVE_PLATFORM_STM32)										 

Display resolution:
- DISPLAY_RESOLUTION_QVGA
- DISPLAY_RESOLUTION_WQVGA
- DISPLAY_RESOLUTION_WVGA
- DISPLAY_RESOLUTION_WSVGA
- DISPLAY_RESOLUTION_WXGA
- DISPLAY_RESOLUTION_HVGA_PORTRAIT

Flash, with size in megabytes:
- EVE_FLASH_AVAILABLE
- EVE_FLASH_SIZE=16

Additionally, the following support flags are set:
- EVE_SUPPORT_FLASH (Set even if EVE_FLASH_AVAILABLE is not set)
- EVE_SUPPORT_UNICODE
- EVE_SUPPORT_ASTC
- EVE_SUPPORT_PNG
- EVE_SUPPORT_VIDEO
- EVE_SUPPORT_CMDB
- EVE_SUPPORT_MEDIAFIFO
- EVE_SUPPORT_CAPACITIVE
- EVE_SUPPORT_RESISTIVE

*/

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/*

Remap options.

*/


/*

Remap manual use of internal macros, internal macros are mapped at the end of this file (due to ESD_SIMULATION overrides)
(Conditional code should check the internal macros MPSSE_PLATFORM, etc. and not the user-specified macros.)

*/

/*

Validate the configured options.

*/
#define EVE_GRAPHICS_AVAILABLE

#define EVE_DISPLAY_AVAILABLE

#define EVE_PLATFORM_AVAILABLE

#if defined(EVE_FLASH_W25Q128)
#define EVE_FLASH_AVAILABLE
#endif

#define EVE_TOUCH_AVAILABLE


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/*

The selected graphics module below must set one of the following options.
(Users must select the graphics module definition, *not* one of these enable options.)
- FT800_ENABLE
- FT801_ENABLE
- FT810_ENABLE
- FT811_ENABLE
- FT812_ENABLE
- FT813_ENABLE
- BT880_ENABLE
- BT881_ENABLE
- BT882_ENABLE
- BT883_ENABLE
- BT815_ENABLE
- BT816_ENABLE
- BT817_ENABLE
- BT818_ENABLE

It may also set platform, display, and flash values if none are configured.

*/

#define BT817_ENABLE
// error : inserted by coan: "#define ENABLE_SPI_QUAD" contradicts -U or --implicit at C:\Users\shahul.hameed\Documents\Git\IDP-1000-04A\ESD Project\ImageSlideShow\Exported Project\FT_Eve_Hal\EVE_Config.h(914)



#ifndef EVE_FLASH_AVAILABLE
#define EVE_FLASH_AVAILABLE
#define EVE_FLASH_W25Q128
#endif



/// Re-Mapping FT800 Series to FT80X

/// Re-Mapping FT810 Series to FT81X

/// Re-Mapping BT880 Series to BT88X

/// Re-Mapping BT815 Series to BT81X

/// Re-Mapping BT817 Series to BT81XA
#define BT81XA_ENABLE

/// Model numbered macro for versioning convenience.
#define EVE_SUPPORT_CHIPID EVE_BT817
#define EVE_SUPPORT_GEN EVE4
#define BT_817_ENABLE

#define BT_81XA_ENABLE

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#define DISPLAY_RESOLUTION_WXGA

#ifndef EVE_DISPLAY_REFRESH
#define EVE_DISPLAY_REFRESH 0
#endif

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/* The following are only used in ESD to configure the most appropriate flash emulation */

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/*

Ultimately, the platform selection must set one of the following internal platform flags.
- BT8XXEMU_PLATFORM
- FT9XX_PLATFORM
- FT4222_PLATFORM
- MPSSE_PLATFORM
- RP2040_PLATFORM
- STM32_PLATFORM				
These may only be set by one of the platform target definitions, and should not be set manually by the user.

*/

#if defined(__FT900__)
// error : inserted by coan: "#define FT900_PLATFORM" contradicts -U or --implicit at C:\Users\shahul.hameed\Documents\Git\IDP-1000-04A\ESD Project\ImageSlideShow\Exported Project\FT_Eve_Hal\EVE_Config.h(1562)

#elif defined(__FT930__)
// error : inserted by coan: "#define FT93X_PLATFORM" contradicts -U or --implicit at C:\Users\shahul.hameed\Documents\Git\IDP-1000-04A\ESD Project\ImageSlideShow\Exported Project\FT_Eve_Hal\EVE_Config.h(1567)

#else
#define RP2040_PLATFORM

#endif




#define EMBEDDED_PLATFORM
#define EVE_HOST EVE_HOST_EMBEDDED

#define EVE_CONFIG__STR(x) #x
#define EVE_CONFIG_STR(x) EVE_CONFIG__STR(x)

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

#define EVE_PLATFORM_AVAILABLE




///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

// Under multi target, the build should create a dynamic library
// Multi target is used by EVE Screen Editor for uploading to a device
#ifndef EVE_HAL_EXPORT
#define EVE_HAL_EXPORT
#endif


// Under multi target, enable all desktop functionality.
// None should have been defined previously.
#define EVE_CHIPID EVE_SUPPORT_CHIPID
#define EVE_GEN EVE_SUPPORT_GEN

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/// Feature support.
/// Avoid hardcoding specific EVE models throughout the libraries.
/// Allows disabling specific features for debugging purposes.
#if (EVE_SUPPORT_CHIPID >= EVE_BT817)
#define EVE_SUPPORT_HSF
#endif
#if (EVE_SUPPORT_CHIPID >= EVE_BT815)
#define EVE_SUPPORT_FLASH
#define EVE_SUPPORT_UNICODE
#define EVE_SUPPORT_ASTC
#endif
#if (EVE_SUPPORT_CHIPID >= EVE_FT810)
#define EVE_SUPPORT_PNG
#define EVE_SUPPORT_VIDEO
#define EVE_SUPPORT_CMDB
#define EVE_SUPPORT_MEDIAFIFO
#if !(EVE_SUPPORT_CHIPID >= EVE_BT880 && EVE_SUPPORT_CHIPID <= EVE_BT883)
// error : inserted by coan: "#define EVE_SUPPORT_LARGEFONT" contradicts -U or --implicit at C:\Users\shahul.hameed\Documents\Git\IDP-1000-04A\ESD Project\ImageSlideShow\Exported Project\FT_Eve_Hal\EVE_Config.h(1727)
#endif
#endif
#if ((EVE_SUPPORT_CHIPID & 0x01) == 0x01)
#define EVE_SUPPORT_CAPACITIVE
#else
// error : inserted by coan: "#define EVE_SUPPORT_RESISTIVE" contradicts -U or --implicit at C:\Users\shahul.hameed\Documents\Git\IDP-1000-04A\ESD Project\ImageSlideShow\Exported Project\FT_Eve_Hal\EVE_Config.h(1734)
#endif

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/// Other options

#define EVE_DL_OPTIMIZE 1 /* Keep cache of displaylist values that don't often change but are generally set by every widget to reduce display list size */
#define EVE_DL_CACHE_SCISSOR 1 /* Keep cache of current scissor */
#define EVE_DL_END_PRIMITIVE 0 /* Whether the END command is sent */
#define EVE_DL_STATE_STACK_SIZE 4
#define EVE_DL_STATE_STACK_MASK 3

#define EVE_CMD_HOOKS 0 /* Allow adding a callback hook into EVE_CoCmd calls using CoCmdHook in EVE_HalContext */


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////


#define eve_progmem
#define eve_progmem_const const

typedef eve_progmem int8_t eve_prog_int8_t;
typedef eve_progmem uint8_t eve_prog_uint8_t;
typedef eve_progmem uint16_t eve_prog_uint16_t;

#ifndef EVE_TCHAR_DEFINED
#define EVE_TCHAR_DEFINED
#ifdef _WIN32
typedef wchar_t eve_tchar_t;
#else
typedef char eve_tchar_t;
#endif
#endif

#ifndef _MSC_VER
/* strcpy_s is not available in GCC */
#define strcpy_s(dst, sz, src) strcpy(dst, src)
#endif

#ifdef _MSC_VER
#define inline __inline
#endif

#if !defined(NDEBUG) && !defined(_DEBUG)
#define _DEBUG 1
#endif

#if defined(__GNUC__)
#define DO_PRAGMA_(x) _Pragma(#x)
#define eve_pragma_warning(msg) DO_PRAGMA_("GCC warning \"" msg "\"")
#elif defined(_MSC_VER)
#define eve_pragma_warning(msg) __pragma(message(__FILE__ "(" EVE_CONFIG_STR(__LINE__) "): warning EVE_Hal: " msg))
#else
#define eve_pragma_warning(msg)
#endif

#if defined(__GNUC__)
#define eve_pragma_error(msg) _Pragma("GCC error \"" msg "\"")
#elif defined(_MSC_VER)
#define eve_pragma_error(msg) __pragma(message(__FILE__ "(" EVE_CONFIG_STR(__LINE__) "): error EVE_Hal: " msg))
#else
#define eve_pragma_error(msg)
#endif



///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/* Set implementation options */

/* Disable unsupported options */

/* Enable FatFS by default on supported platforms */
#ifndef EVE_ENABLE_FATFS
#define EVE_ENABLE_FATFS 1
#endif


#ifndef EVE_ENABLE_FATFS
#define EVE_ENABLE_FATFS 0
#endif

/* Remove unsupported options */

///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////

/// Configuration sanity checks
#if !defined(EVE_SUPPORT_FLASH) && defined(EVE_FLASH_AVAILABLE)
#pragma message(__FILE__ "(" EVE_CONFIG_STR(__LINE__) "): warning EVE_FLASH_AVAILABLE: " \
                                                      "Cannot enable flash on EVE model which doesn't support flash")
#undef EVE_FLASH_AVAILABLE
#endif

#endif /* EVE_CONFIG__H */

/* end of file */
