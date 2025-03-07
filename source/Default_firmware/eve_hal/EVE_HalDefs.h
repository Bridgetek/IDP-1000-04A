/**
 * @file EVE_HalDefs.h
 * @brief Eve_Hal framework APIs
 *
 *  This file defines the generic APIs of phost access layer for the FT800 or EVE compatible silicon.
 *  Application shall access FT800 or EVE resources over these APIs,regardless of I2C or SPI protocol.
 *  In addition, there are some helper functions defined for FT800 coprocessor engine as well as phost commands.
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

#ifndef EVE_HAL_INCL__H
#define EVE_HAL_INCL__H

/*************
** INCLUDES **
*************/
#include "EVE_GpuTypes.h"

#if EVE_ENABLE_FATFS
#include "ff.h"
#endif

/***********
** MARCOS **
***********/
#define RAM_ERR_REPORT_MAX 128

/*************
** TYPEDEFS **
*************/

typedef enum EVE_MODE_T
{
	EVE_MODE_UNKNOWN = 0, /**< 0 */
	EVE_MODE_I2C,
	EVE_MODE_SPI,
} EVE_MODE_T;

typedef enum EVE_STATUS_T
{
	EVE_STATUS_CLOSED = 0, /**< 0 */
	EVE_STATUS_OPENED,
	EVE_STATUS_READING,
	EVE_STATUS_WRITING,
	EVE_STATUS_ERROR,
} EVE_STATUS_T;

typedef enum EVE_TRANSFER_T
{
	EVE_TRANSFER_NONE = 0, /**< 0 */
	EVE_TRANSFER_READ,
	EVE_TRANSFER_WRITE,
} EVE_TRANSFER_T;

typedef struct EVE_HalContext EVE_HalContext;
typedef bool (*EVE_Callback)(EVE_HalContext *phost);

/** Hook into coprocessor commands. Return 1 to abort the command. Useful for an optimization routine */
typedef int (*EVE_CoCmdHook)(EVE_HalContext *phost, uint32_t cmd, uint32_t state);
/** Hook into coprocessor reset */
typedef void (*EVE_ResetCallback)(EVE_HalContext *phost, bool fault);

typedef struct EVE_DeviceInfo
{
	char DisplayName[256];
	char SerialNumber[16];
	bool Opened;

} EVE_DeviceInfo;

/** Hal parameters */
typedef struct EVE_HalParameters
{
	void *UserContext;

	/** Called anytime the code is waiting during CMD write. Return false to abort wait */
	EVE_Callback CbCmdWait;

	uint32_t DeviceIdx;
	uint8_t SpiCsPin; /**< SPI chip select number of FT8XX chip */
	uint8_t SpiSckPin;
	uint8_t SpiMosiPin;
	uint8_t SpiMisoPin;
	uint8_t PowerDownPin; /**< FT8XX power down pin number */

} EVE_HalParameters;

#if (EVE_DL_OPTIMIZE) || (EVE_DL_CACHE_SCISSOR)
#define EVE_DL_STATE phost->DlState[phost->DlStateIndex]
typedef struct EVE_HalDlState
{
	// Keep to a minimum
#if (EVE_DL_OPTIMIZE)
	uint32_t PaletteSource;
	uint32_t ColorRGB;
	int16_t LineWidth;
	int16_t PointSize;
#endif
#if (EVE_DL_CACHE_SCISSOR)
	uint16_t ScissorX;
	uint16_t ScissorY;
	uint16_t ScissorWidth;
	uint16_t ScissorHeight;
#endif
#if (EVE_DL_OPTIMIZE)
	uint8_t ColorA;
	uint8_t Handle; /**< Current handle */
	uint8_t Cell; /**< Current cell */
#endif
	uint8_t VertexFormat; /**< Current vertex format */
#if (EVE_DL_OPTIMIZE)
	bool BitmapTransform; /**< BitmapTransform other than default is set */
#endif
} EVE_HalDlState;
#endif

typedef struct EVE_HalContext
{
	/** Pointer to user context */
	void *UserContext;

	/** Pointer to a support library context */
	void *LibraryContext;

	/** Called anytime the code is waiting during CMD write. Return false to abort wait */
	EVE_Callback CbCmdWait;
	/** Callback hook called anytime the coprocessor is reset through the EVE_Util interface */
	EVE_ResetCallback CbCoprocessorReset;

#if EVE_CMD_HOOKS
	/* Hook into coprocessor commands. Called when EVE_CoCmd interface is used. Return 1 to abort the command. Useful for an optimization routine */
	EVE_CoCmdHook CoCmdHook;
#endif

	EVE_STATUS_T Status;

	uint8_t PCLK;

	/** @name User space width and height, based on REG_HSIZE, REG_VSIZE and REG_ROTATE */
	///@{
	uint32_t Width;
	uint32_t Height;
	///@}

	/** @name Handles to external context */
	///@{
	void *SpiPort; /* SPI port */
	EVE_SPI_CHANNELS_T SpiChannels; /**< Variable to contain single/dual/quad channels */
	uint8_t SpiDummyBytes; /**< Number of dummy bytes as 1 or 2 for SPI read */
	uint8_t SpiCsPin; /**< SPI chip select number of FT8XX chip */
	uint8_t SpiSckPin;
	uint8_t SpiMosiPin;
	uint8_t SpiMisoPin;
	uint8_t PowerDownPin; /**< FT8XX power down pin number */
	///@}

	/** @name Buffer cmd smaller than a full cmd command */
	///@{
	uint8_t CmdBuffer[4];
	uint8_t CmdBufferIndex;

	uint16_t CmdSpace; /**< Free space, cached value */
#if !defined(EVE_SUPPORT_CMDB)
	uint16_t CmdWp; /**< Write pointer, only valid when CMDB is not used */
#endif
	///@}

	/** @name Media FIFO state */
	///@{
#if defined(EVE_SUPPORT_MEDIAFIFO)
	uint32_t MediaFifoAddress;
	uint32_t MediaFifoSize;
#if EVE_ENABLE_FATFS
	FIL LoadFileObj;
#else
	void *LoadFileHandle;
#endif
	ptrdiff_t LoadFileRemaining;
#endif
	///@}

	/** @name Display list optimization and compatibility caches */
	///@{
#if (EVE_DL_OPTIMIZE) || (EVE_DL_CACHE_SCISSOR)
	EVE_HalDlState DlState[EVE_DL_STATE_STACK_SIZE];
	uint8_t DlStateIndex;
#endif
#if (EVE_DL_OPTIMIZE)
	uint8_t DlPrimitive;
	uint32_t CoFgColor;
	uint32_t CoBgColor;
	bool CoBitmapTransform; /**< BitmapTransform other than identity is set on the coprocessor */
#endif
	uint8_t CoScratchHandle;
	///@}

#if defined(_DEBUG)
	bool DebugMessageVisible;
	uint8_t DebugBackup[RAM_ERR_REPORT_MAX];
#endif

	/** @name Status flags */
	///@{
	bool CmdFunc; /**< Flagged while transfer to cmd is kept open */
	bool CmdFault; /**< Flagged when coprocessor is in fault mode and needs to be reset */
	bool CmdWaiting; /**< Flagged while waiting for CMD write (to check during any function that may be called by CbCmdWait) */
	///@}

} EVE_HalContext;

typedef struct EVE_HalPlatform
{
	uint32_t OpenedDevices;

} EVE_HalPlatform;

/**********************
 *   INLINE FUNCTIONS
 **********************/

/** @name CAPS */
///@{
static inline bool EVE_Hal_supportHsf(EVE_HalContext *phost)
{
#ifdef EVE_SUPPORT_HSF
	return true;
#else
	return false;
#endif
}

static inline bool EVE_Hal_supportFlash(EVE_HalContext *phost)
{
#ifdef EVE_SUPPORT_FLASH
	return true;
#else
	return false;
#endif
}

static inline bool EVE_Hal_supportMediaFifo(EVE_HalContext *phost)
{
#ifdef EVE_SUPPORT_MEDIAFIFO
	return true;
#else
	return false;
#endif
}

static inline bool EVE_Hal_supportVideo(EVE_HalContext *phost)
{
#ifdef EVE_SUPPORT_VIDEO
	return true;
#else
	return false;
#endif
}

static inline bool EVE_Hal_supportLargeFont(EVE_HalContext *phost)
{
#ifdef EVE_SUPPORT_LARGEFONT
	return true;
#else
	return false;
#endif
}

/** Include the EVE generation in the chip ID value to simplify feature set comparisons(BT880 support) */
static inline uint32_t EVE_extendedChipId(int chipId)
{
	return (uint32_t)((chipId & 0xFFFF) | 0x40000);
}

/** Remove EVE generation from the chip ID */
static inline int EVE_shortChipId(uint32_t chipId)
{
	return chipId & 0xFFFF;
}

static inline int EVE_gen(uint32_t chipId)
{
	return EVE4;
}
///@}

#endif /* #ifndef EVE_HAL_INCL__H */

/* end of file */
