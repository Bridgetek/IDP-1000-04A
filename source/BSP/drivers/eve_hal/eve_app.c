/**
 * @file eve_app.h
 * @brief Sample usage of EVE
 *
 * @author Bridgetek
 *
 * @date 2025
 * 
 * MIT License
 *
 * Copyright (c) [2025] [Bridgetek Pte Ltd (BRTChip)]
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

static EVE_HalContext s_halContext;
static EVE_HalContext* s_pHalContext;

static uint32_t a;
static uint32_t b;
static uint32_t c;
static uint32_t d;
static uint32_t e;
static uint32_t f;

bool cbCmdWait(struct EVE_HalContext* phost) {
    (void)(phost); // suspress warning on unused
    return true;
}

bool Gpu_Init(void) {
    size_t deviceIdx = -1;
    EVE_HalParameters params;
    s_pHalContext = &s_halContext;

    EVE_Hal_initialize();

    EVE_Hal_defaultsEx(&params, deviceIdx);
    params.CbCmdWait = cbCmdWait;

    if (!EVE_Hal_open(s_pHalContext, &params))
    {
        return false;
    }
    else
    {
        if (!EVE_Util_bootupConfig(s_pHalContext))
        {
            return false;
        }
    }
    return true;

}

bool Eve_Calibrate(void)
{
    uint32_t result;
    uint32_t transMatrix[6];
    EVE_Hal_wr8(s_pHalContext, REG_CTOUCH_EXTENDED, CTOUCH_MODE_COMPATIBILITY);

    eve_printf_debug("App_CoPro_Widget_Calibrate: Start\n");

    EVE_CoCmd_dlStart(s_pHalContext);
    EVE_CoDl_clearColorRgb(s_pHalContext, 64, 64, 64);
    EVE_CoDl_clear(s_pHalContext, 1, 1, 1);
    EVE_CoDl_colorRgb(s_pHalContext, 0xff, 0xff, 0xff);

    EVE_CoCmd_text(s_pHalContext, (uint16_t)(s_pHalContext->Width / 2), (uint16_t)(s_pHalContext->Height / 2), 27, OPT_CENTER, "Please Tap on the dot");

    result = EVE_CoCmd_calibrate(s_pHalContext);
    EVE_Cmd_waitFlush(s_pHalContext);

    eve_printf_debug("App_CoPro_Widget_Calibrate: End\n");

    // Print the configured values
    EVE_Hal_rdMem(s_pHalContext, (uint8_t*)transMatrix, REG_TOUCH_TRANSFORM_A, 4 * 6); //read all the 6 coefficients
    eve_printf_debug("Touch screen transform values are A 0x%lx,B 0x%lx,C 0x%lx,D 0x%lx,E 0x%lx, F 0x%lx\n",
        transMatrix[0], transMatrix[1], transMatrix[2], transMatrix[3], transMatrix[4], transMatrix[5]);

    return result != 0;
}

void Calibration_Restore(void)
{
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_A, a);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_B, b);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_C, c);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_D, d);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_E, e);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_F, f);
}

void Calibration_Save(void)
{
    a = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_A);
    b = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_B);
    c = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_C);
    d = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_D);
    e = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_E);
    f = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_F);
}

void EVE_logo(void)
{
    EVE_CoCmd_dlStart(s_pHalContext);
    EVE_CoDl_clearColorRgb(s_pHalContext, 0xFF, 0xFF, 0xFF);
    EVE_CoDl_clear(s_pHalContext, 1, 1, 1);
    EVE_CoCmd_logo(s_pHalContext);
    EVE_CoDl_display(s_pHalContext);
    EVE_Cmd_waitFlush(s_pHalContext);
}

void EVE_buzzer(void)
{
    uint16_t val = (uint16_t)((0xC0 << 8) | 0x50);
    EVE_Hal_wr8(s_pHalContext, REG_VOL_SOUND, 255);
    EVE_Hal_wr16(s_pHalContext, REG_SOUND, val);
    for (uint8_t i = 0; i < 3; i++)
    {
        EVE_Hal_wr8(s_pHalContext, REG_PLAY, 1);
        EVE_sleep(500);
    }

    EVE_Hal_wr16(s_pHalContext, REG_SOUND, 0);
    EVE_Hal_wr8(s_pHalContext, REG_PLAY, 1);
}

