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

#ifndef EVE_KD2401_H
#define EVE_KD2401_H


#include <stdint.h>
/* Macros for KD driver */

#define KD2401_CS_LOW (gpio_write(33, 0))
#define KD2401_CS_HIGH (gpio_write(33, 1))
#define KD2401_CLK_LOW (gpio_write(27, 0))
#define KD2401_CLK_HIGH (gpio_write(27, 1))
#define KD2401_MOSI_LOW (gpio_write(29, 0))
#define KD2401_MOSI_HIGH (gpio_write(29, 1))
#define KD2401_DCX_LOW (gpio_write(41, 0))
#define KD2401_DCX_HIGH (gpio_write(41, 1))

#define KD2401_MISO_LOWHIGH (gpio_read(30))

//KD2401 commands
#define KD2401_CMD_SOFTWARE_RESET				(0x01)//next new command only after 5ms and sleep commands only after 120ms
#define KD2401_CMD_READ_DISPLAY_IDENTIFICATION	(0x04)
#define KD2401_CMD_READ_DISPLAY_POWERMODE		(0x0A)
#define KD2401_CMD_READ_MADCTRL				(0x0B)
#define KD2401_CMD_READ_PIXEL_FORMAT			(0x0C)

#define KD2401_READ_DISPLAY_SIGNALMODE			(0x0E)

#define KD2401_CMD_SLEEP_IN					(0x10)
#define KD2401_CMD_SLEEP_OUT					(0x11)
#define KD2401_CMD_NORMAL_DISPLAY_ON			(0x13)
#define KD2401_CMD_ALLPIXELOFF					(0x22)
#define KD2401_CMD_ALLPIXELON					(0x23)
#define KD2401_CMD_DISPLAYOFF					(0x28)
#define KD2401_CMD_DISPLAYON					(0x29)//need to do this
#define KD2401_CMD_COLOMNADDR					(0x2A)
#define KD2401_CMD_ROWADDR						(0x2B)
#define KD2401_CMD_TEARINGEFFECT_LINEOFF		(0x34)
#define KD2401_CMD_TEARINGEFFECT_LINEON		(0x35)

#define KD2401_CMD_MADCTRL						(0x36)
#define KD2401_CMD_IDLE_OFF					(0x38)
#define KD2401_CMD_IDLE_ON						(0x39)

#define KD2401_CMD_INTERFACE_PIXEL_FORMAT		(0x3A)//default is 06 and DBI 18 bits/pixel
#define KD2401_INTERFACE_PIXEL_FORMAT_18BIT_DPI	(0x66)
#define KD2401_INTERFACE_PIXEL_FORMAT_24BIT_DPI	(0x70)

#define KD2401_CMD_READ_ID1					(0xDA)
#define KD2401_CMD_READ_ID2					(0xDB)
#define KD2401_CMD_READ_ID3					(0xDC)

#define KD2401_ID1								(0x54)
#define KD2401_ID2								(0x80)
#define KD2401_ID3								(0x66)

#define KD2401_CMD_WRITE_CONTROL_DISPLAY		(0x53)
#define KD2401_CMD_WRITE_BRIGHTNESSVALUE		(0x51)
#define KD2401_CMD_READ_BRIGHTNESSVALUE		(0x52)
#define KD2401_CMD_WRITE_CTRLDISPLAYVALUE		(0x53)
#define KD2401_CMD_READ_CTRLDISPLAYVALUE		(0x54)

//extended functions
#define KD2401_CMD_INTERFACE_MODE_CONTROL		(0xB0)
#define KD2401_CMD_FRAME_RATE_CONTROL			(0xB1)
#define KD2401_CMD_BLANCKING_PORCH_CONTROL		(0xB5)
#define KD2401_CMD_DISPLAY_FUNCTION_CONTROL 	(0xB6)
#define KD2401_CMD_READ_ID4					(0xD3)
#define KD2401_CMD_IMAGEFUNCTION				(0xE9)

#define KD2401_ID4								(0x00009488L)

#define KD2401_CMD_SPI_READCMDSETTINGS			(0xFB)

void KD2401_Bootup();

#endif  /* EVE_KD2401 */
