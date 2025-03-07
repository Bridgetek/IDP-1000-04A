/**
 * @file EVE_CoCmd_IO.c
 * @brief EVE's co-processor IO commmands
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

bool EVE_CoCmd_memCrc(EVE_HalContext *phost, uint32_t ptr, uint32_t num, uint32_t *result)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_MEMCRC, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_MEMCRC);
	EVE_Cmd_wr32(phost, ptr);
	EVE_Cmd_wr32(phost, num);
	resAddr = EVE_Cmd_moveWp(phost, 4);
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (result)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		*result = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
	}
	return true;
}

bool EVE_CoCmd_regRead(EVE_HalContext *phost, uint32_t ptr, uint32_t *result)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_REGREAD, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_REGREAD);
	EVE_Cmd_wr32(phost, ptr);
	resAddr = EVE_Cmd_moveWp(phost, 4);
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (result)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		*result = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
	}
	return true;
}

void EVE_CoCmd_memWrite(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_MEMWRITE, ptr, num);
}

void EVE_CoCmd_memWrite32(EVE_HalContext *phost, uint32_t ptr, uint32_t value)
{
	EVE_CoCmd_dddd(phost, CMD_MEMWRITE, ptr, 4, value);
}

void EVE_CoCmd_memSet(EVE_HalContext *phost, uint32_t ptr, uint32_t value, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_MEMSET, ptr, value, num);
}

void EVE_CoCmd_memZero(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_MEMZERO, ptr, num);
}

void EVE_CoCmd_memCpy(EVE_HalContext *phost, uint32_t dest, uint32_t src, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_MEMCPY, dest, src, num);
}

void EVE_CoCmd_append(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_APPEND, ptr, num);
}

void EVE_CoCmd_snapshot(EVE_HalContext *phost, uint32_t ptr)
{
	EVE_CoCmd_dd(phost, CMD_SNAPSHOT, ptr);
}

void EVE_CoCmd_inflate(EVE_HalContext *phost, uint32_t ptr)
{
	EVE_CoCmd_dd(phost, CMD_INFLATE, ptr);
}


bool EVE_CoCmd_inflate_progMem(EVE_HalContext *phost, uint32_t dst, eve_progmem_const uint8_t *src, uint32_t size)
{
	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready
	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_INFLATE);
	EVE_Cmd_wr32(phost, dst);
	EVE_Cmd_wrProgMem(phost, src, (size + 3) & ~0x3UL);
	EVE_Cmd_endFunc(phost);
	return EVE_Cmd_waitFlush(phost); // Resource failed to load
}

/** @brief Get the end memory address of data inflated by CMD_INFLATE and CMD_INFLATE2
 * At API level 2, the allocation pointer is also advanced by:
 *  - cmd_loadimage
 *  - cmd_playvideo
 *  - cmd_videoframe
 *  - cmd_endlist
 */
bool EVE_CoCmd_getPtr(EVE_HalContext *phost, uint32_t *result)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_GETPTR, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_GETPTR);
	resAddr = EVE_Cmd_moveWp(phost, 4); // move write pointer to result location
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (result)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		*result = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
	}
	return true;
}

void EVE_CoCmd_loadImage(EVE_HalContext *phost, uint32_t ptr, uint32_t options)
{
	EVE_CoCmd_ddd(phost, CMD_LOADIMAGE, ptr, options);
}

bool EVE_CoCmd_loadImage_progMem(EVE_HalContext *phost, uint32_t dst, eve_progmem_const uint8_t *src, uint32_t size, uint32_t *format)
{
#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_LOADIMAGE, dst))
		return false;
#endif

	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready
	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_LOADIMAGE);
	EVE_Cmd_wr32(phost, dst);
	EVE_Cmd_wr32(phost, OPT_NODL);
	EVE_Cmd_wrProgMem(phost, src, (size + 3) & ~0x3UL);
	EVE_Cmd_endFunc(phost);
	if (!EVE_Cmd_waitFlush(phost))
		return false; // Image failed to load
	if (format)
		*format = EVE_Hal_rd32(phost, 0x3097e8);
	return true;
}

bool EVE_CoCmd_getProps(EVE_HalContext *phost, uint32_t *ptr, uint32_t *w, uint32_t *h)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_GETPROPS, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_GETPROPS);
	resAddr = EVE_Cmd_moveWp(phost, 12);
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (ptr || w || h)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		if (ptr)
			*ptr = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
		if (w)
			*w = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 4);
		if (h)
			*h = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 8);
	}
	return true;
}

void EVE_CoCmd_snapshot2(EVE_HalContext *phost, uint32_t fmt, uint32_t ptr, int16_t x, int16_t y, int16_t w, int16_t h)
{
	EVE_CoCmd_dddwwww(phost, CMD_SNAPSHOT2, fmt, ptr, x, y, w, h);
}

void EVE_CoCmd_mediaFifo(EVE_HalContext *phost, uint32_t ptr, uint32_t size)
{
	EVE_CoCmd_ddd(phost, CMD_MEDIAFIFO, ptr, size);
}

void EVE_CoCmd_videoStart(EVE_HalContext *phost)
{
	EVE_CoCmd_d(phost, CMD_VIDEOSTART);
}

void EVE_CoCmd_videoFrame(EVE_HalContext *phost, uint32_t dst, uint32_t ptr)
{
	EVE_CoCmd_dddd(phost, CMD_MEMWRITE, 3182920, 1, OPT_NODL); // WORKAROUND CMD_VIDEOFRAME
	EVE_CoCmd_ddd(phost, CMD_VIDEOFRAME, dst, ptr);
}

void EVE_CoCmd_flashErase(EVE_HalContext *phost)
{
	EVE_CoCmd_d(phost, CMD_FLASHERASE);
}

bool EVE_CoCmd_flashErase_flush(EVE_HalContext *phost)
{
	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready

	EVE_CoCmd_d(phost, CMD_FLASHERASE);

	return EVE_Cmd_waitFlush(phost);
}

void EVE_CoCmd_flashWrite(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_FLASHWRITE, ptr, num);
}

void EVE_CoCmd_flashRead(EVE_HalContext *phost, uint32_t dest, uint32_t src, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_FLASHREAD, dest, src, num);
}

bool EVE_CoCmd_flashRead_flush(EVE_HalContext *phost, uint32_t dest, uint32_t src, uint32_t num)
{
	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready

	EVE_CoCmd_dddd(phost, CMD_FLASHREAD, dest, src, num);

	return EVE_Cmd_waitFlush(phost);
}

void EVE_CoCmd_flashUpdate(EVE_HalContext *phost, uint32_t dest, uint32_t src, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_FLASHUPDATE, dest, src, num);
}

void EVE_CoCmd_flashDetach(EVE_HalContext *phost)
{
	EVE_CoCmd_d(phost, CMD_FLASHDETACH);
}

uint32_t EVE_CoCmd_flashAttach(EVE_HalContext *phost)
{
	uint32_t flashStatus;
	if (!EVE_Cmd_waitFlush(phost))
		return EVE_Hal_rd32(phost, REG_FLASH_STATUS); // Coprocessor must be ready
	flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
	if (flashStatus != FLASH_STATUS_DETACHED)
		return flashStatus; // Only attach when detached
	EVE_CoCmd_d(phost, CMD_FLASHATTACH);
	EVE_Cmd_waitFlush(phost); // Wait for command completion
	return EVE_Hal_rd32(phost, REG_FLASH_STATUS); // Return current status
}

/**
 * @brief Enter fast flash state.
 * @param phost Pointer to Hal context
 * @param result
 * @return new FLASH_STATUS
 */
uint32_t EVE_CoCmd_flashFast(EVE_HalContext *phost, uint32_t *result)
{
	uint16_t resAddr;
	uint32_t flashStatus;

	if (!EVE_Cmd_waitFlush(phost))
	{
		if (result)
			*result = 0xE000;
		return EVE_Hal_rd32(phost, REG_FLASH_STATUS);
	} // Coprocessor must be ready
	flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
	if (flashStatus < FLASH_STATUS_BASIC)
	{
		if (result)
			*result = 0xE001;
		return flashStatus;
	}
	if (flashStatus > FLASH_STATUS_BASIC)
	{
		if (result)
			*result = 0;
		return flashStatus;
	} // Only enter fast mode when attached

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_FLASHFAST, 0))
		return flashStatus;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_FLASHFAST);
	resAddr = EVE_Cmd_moveWp(phost, 4); // Get the address where the coprocessor will write the result
	EVE_Cmd_endFunc(phost);
	EVE_Cmd_waitFlush(phost); // Wait for command completion
	if (result)
		*result = EVE_Hal_rd32(phost, RAM_CMD + resAddr); // Fetch result
	return EVE_Hal_rd32(phost, REG_FLASH_STATUS); // Return current status
}

void EVE_CoCmd_flashSpiDesel(EVE_HalContext *phost)
{
	EVE_CoCmd_d(phost, CMD_FLASHSPIDESEL);
}

void EVE_CoCmd_flashSpiTx(EVE_HalContext *phost, uint32_t num)
{
	EVE_CoCmd_dd(phost, CMD_FLASHSPITX, num);
}

void EVE_CoCmd_flashSpiRx(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_FLASHSPIRX, ptr, num);
}

void EVE_CoCmd_flashSource(EVE_HalContext *phost, uint32_t ptr)
{
	EVE_CoCmd_dd(phost, CMD_FLASHSOURCE, ptr);
}

void EVE_CoCmd_appendF(EVE_HalContext *phost, uint32_t ptr, uint32_t num)
{
	EVE_CoCmd_ddd(phost, CMD_APPENDF, ptr, num);
}

void EVE_CoCmd_videoStartF(EVE_HalContext *phost)
{
	EVE_CoCmd_d(phost, CMD_VIDEOSTARTF);
}

bool EVE_CoCmd_loadImage_flash(EVE_HalContext *phost, uint32_t dst, uint32_t src, uint32_t *format)
{
#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_FLASHSOURCE, src))
		return false;
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_LOADIMAGE, dst))
		return false;
#endif

	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready
	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_FLASHSOURCE);
	EVE_Cmd_wr32(phost, src);
	EVE_Cmd_wr32(phost, CMD_LOADIMAGE);
	EVE_Cmd_wr32(phost, dst);
	EVE_Cmd_wr32(phost, OPT_FLASH | OPT_NODL);
	EVE_Cmd_endFunc(phost);
	if (!EVE_Cmd_waitFlush(phost))
		return false; // Image failed to load
	if (format)
		*format = EVE_Hal_rd32(phost, 0x3097e8);
	return true;
}

bool EVE_CoCmd_inflate_flash(EVE_HalContext *phost, uint32_t dst, uint32_t src)
{
#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_FLASHSOURCE, src))
		return false;
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_INFLATE2, dst))
		return false;
#endif

	if (!EVE_Cmd_waitFlush(phost))
		return false; // Coprocessor must be ready
	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_FLASHSOURCE);
	EVE_Cmd_wr32(phost, src);
	EVE_Cmd_wr32(phost, CMD_INFLATE2);
	EVE_Cmd_wr32(phost, dst);
	EVE_Cmd_wr32(phost, OPT_FLASH);
	EVE_Cmd_endFunc(phost);
	return EVE_Cmd_waitFlush(phost); // Image failed to load
}

void EVE_CoCmd_inflate2(EVE_HalContext *phost, uint32_t ptr, uint32_t options)
{
	EVE_CoCmd_ddd(phost, CMD_INFLATE2, ptr, options);
}

bool EVE_CoCmd_getImage(EVE_HalContext *phost, uint32_t *source, uint32_t *fmt, uint32_t *w, uint32_t *h, uint32_t *palette)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_GETIMAGE, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_GETIMAGE);
	resAddr = EVE_Cmd_moveWp(phost, 4 * 5);
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (source || fmt || w || h || palette)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		if (source)
			*source = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
		if (fmt)
			*fmt = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 4);
		if (w)
			*w = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 8);
		if (h)
			*h = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 12);
		if (palette)
			*palette = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 16);
	}
	return true;
}

void EVE_CoCmd_fontCache(EVE_HalContext *phost, uint32_t font, int32_t ptr, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_FONTCACHE, font, ptr, num);
}

bool EVE_CoCmd_fontCacheQuery(EVE_HalContext *phost, uint32_t *total, int32_t *used)
{
	uint16_t resAddr;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_FONTCACHEQUERY, 0))
		return false;
#endif

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_FONTCACHEQUERY);
	resAddr = EVE_Cmd_moveWp(phost, 8);
	EVE_Cmd_endFunc(phost);

	/* Read result */
	if (total || used)
	{
		if (!EVE_Cmd_waitFlush(phost))
			return false;
		if (total)
			*total = EVE_Hal_rd32(phost, RAM_CMD + resAddr);
		if (used)
			*used = EVE_Hal_rd32(phost, RAM_CMD + resAddr + 4);
	}
	return true;
}

void EVE_CoCmd_playVideo(EVE_HalContext *phost, uint32_t options)
{
	EVE_CoCmd_dd(phost, CMD_PLAYVIDEO, options);

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif

	if (options & (OPT_FLASH | OPT_MEDIAFIFO))
	{
		/* WORKAROUND: CMD_PLAYVIDEO completes immediately,
		but the command following it does not.
		Write a CMD_NOP command to behave as documented */
		EVE_CoCmd_nop(phost);
	}
}

bool EVE_CoCmd_animStart(EVE_HalContext *phost, int32_t ch, uint32_t aoptr, uint32_t loop)
{
	uint32_t flashStatus;

#if EVE_CMD_HOOKS
	if (phost->CoCmdHook && phost->CoCmdHook(phost, CMD_ANIMSTART, ch))
		return false;
#endif

	if (!EVE_Cmd_waitFlush(phost))
		return false;
	flashStatus = EVE_Hal_rd32(phost, REG_FLASH_STATUS);
	if (flashStatus < FLASH_STATUS_FULL)
	{
		// Don't attempt to start animation without full flash speed, it hangs the coprocessor
		eve_printf_debug("Attempt to send CMD_ANIMSTART without FLASH_STATUS_FULL (REG_FLASH_STATUS: %i)\n", (int)flashStatus);
		return false;
	}

	EVE_Cmd_startFunc(phost);
	EVE_Cmd_wr32(phost, CMD_ANIMSTART);
	EVE_Cmd_wr32(phost, ch);
	EVE_Cmd_wr32(phost, aoptr);
	EVE_Cmd_wr32(phost, loop);
	EVE_Cmd_endFunc(phost);
	return EVE_Cmd_waitFlush(phost);
}

void EVE_CoCmd_animStop(EVE_HalContext *phost, int32_t ch)
{
	EVE_CoCmd_dd(phost, CMD_ANIMSTOP, ch);
}

void EVE_CoCmd_animXY(EVE_HalContext *phost, int32_t ch, int16_t x, int16_t y)
{
	EVE_CoCmd_ddww(phost, CMD_ANIMXY, ch, x, y);
}

void EVE_CoCmd_animDraw(EVE_HalContext *phost, int32_t ch)
{
	EVE_CoCmd_dd(phost, CMD_ANIMDRAW, ch);

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
}

void EVE_CoCmd_animFrame(EVE_HalContext *phost, int16_t x, int16_t y, uint32_t aoptr, uint32_t frame)
{
	EVE_CoCmd_dwwdd(phost, CMD_ANIMFRAME, x, y, aoptr, frame);

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
}

void EVE_CoCmd_animFrameRam(EVE_HalContext *phost, int16_t x, int16_t y, uint32_t aoptr, uint32_t frame)
{
	EVE_CoCmd_dwwdd(phost, CMD_ANIMFRAMERAM, x, y, aoptr, frame);

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
}

void EVE_CoCmd_animStartRam(EVE_HalContext *phost, int32_t ch, uint32_t aoptr, uint32_t loop)
{
	EVE_CoCmd_dddd(phost, CMD_ANIMSTARTRAM, ch, aoptr, loop);
}

void EVE_CoCmd_runAnim(EVE_HalContext *phost, uint32_t waitmask, uint32_t play)
{
	EVE_CoCmd_ddd(phost, CMD_RUNANIM, waitmask, play);

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
}

void EVE_CoCmd_flashProgram(EVE_HalContext *phost, uint32_t dst, uint32_t src, uint32_t num)
{
	EVE_CoCmd_dddd(phost, CMD_FLASHPROGRAM, dst, src, num);
}

/* end of file */
