/**
 * @file app.c
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

#include "app.h"

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

void Gpu_Init() {
    size_t deviceIdx = -1;
    EVE_HalParameters params;

    EVE_Hal_initialize();

    EVE_Hal_defaultsEx(&params, deviceIdx);
    params.CbCmdWait = cbCmdWait;

    EVE_Hal_open(s_pHalContext, &params);

    EVE_Util_bootupConfig(s_pHalContext);

    EVE_Util_loadSdCard(s_pHalContext);
}

bool Eve_Calibrate()
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

void Calibration_Restore()
{
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_A, a);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_B, b);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_C, c);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_D, d);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_E, e);
    EVE_Hal_wr32(s_pHalContext, REG_TOUCH_TRANSFORM_F, f);
}

void Calibration_Save() 
{
    a = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_A);
    b = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_B);
    c = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_C);
    d = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_D);
    e = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_E);
    f = EVE_Hal_rd32(s_pHalContext, REG_TOUCH_TRANSFORM_F);
}

void default_fw()
{
    EVE_CoCmd_dlStart(s_pHalContext);
    EVE_CoDl_clearColorRgb(s_pHalContext, 0xFF, 0xFF, 0xFF);
    EVE_CoDl_clear(s_pHalContext, 1, 1, 1);
    EVE_CoCmd_logo(s_pHalContext);
    EVE_CoDl_display(s_pHalContext);
    EVE_Cmd_waitFlush(s_pHalContext);
    while (1)
    {
        eve_printf_debug("Logo\n");
        EVE_sleep(1000);
    }
}

int main(int argc, char* argv[])
{
    s_pHalContext = &s_halContext;
    Gpu_Init();

    // read and store calibration setting
#if GET_CALIBRATION == 1
    Eve_Calibrate();
    Calibration_Save();
#endif

    EVE_Util_clearScreen(s_pHalContext);

    default_fw();

    EVE_Util_clearScreen(s_pHalContext);

    EVE_Hal_close(s_pHalContext);
    EVE_Hal_release();

    /* Init HW Hal for next loop*/
    Gpu_Init(s_pHalContext);
#if GET_CALIBRATION == 1
    Calibration_Restore();
#endif

    return 0;
}


