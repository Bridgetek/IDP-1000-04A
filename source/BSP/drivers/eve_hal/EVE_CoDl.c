/**
 * @file EVE_CoDl.c
 * @brief EVE's co-processor command
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

#include "EVE_CoDl.h"

#if defined(_MSC_VER) && (_MSC_VER < 1800)
#include <stdio.h> /* memset */
#endif

void EVE_CoDlImpl_resetDlState(EVE_HalContext *phost)
{
#if (EVE_DL_OPTIMIZE) || (EVE_DL_CACHE_SCISSOR)
	phost->DlStateIndex = 0;
	phost->DlState[0] = (EVE_HalDlState)
	{
#if (EVE_DL_OPTIMIZE)
		.LineWidth = 16,
		.PointSize = 16,
		.ColorRGB = 0xFFFFFF,
#endif
#if (EVE_DL_CACHE_SCISSOR)
		.ScissorWidth = (uint16_t)phost->Width,
		.ScissorHeight = (uint16_t)phost->Height,
#endif
#if (EVE_DL_OPTIMIZE)
		.ColorA = 0xFF,
		.Handle = 0x3F, /* Invalid value */
#endif
#if (EVE_DL_OPTIMIZE)
		    .VertexFormat = 4,
#elif (EVE_DL_CACHE_SCISSOR)
		0
#endif
	};
#endif

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
}

void EVE_CoDlImpl_resetCoState(EVE_HalContext *phost)
{
#if (EVE_DL_OPTIMIZE)
	phost->CoFgColor = 0x003870;
	phost->CoBgColor = 0x002040;
	phost->CoBitmapTransform = false;
#endif

	phost->CoScratchHandle = 15;
}


/* Nothing beyond this */
