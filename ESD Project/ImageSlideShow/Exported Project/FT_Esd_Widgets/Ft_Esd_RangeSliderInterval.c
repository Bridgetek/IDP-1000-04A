#include "Ft_Esd.h"
#include "Ft_Esd_RangeSliderInterval.h"
#include "Ft_Esd_Theme_DarkOrange.h"
#include "Ft_Esd_TouchTag.h"
#include "Ft_Esd_Math.h"
#include "Ft_Esd_Elements.h"
#include "Ft_CoPro_Cmds.h"
#include "Ft_Esd_CoCmd.h"
#include <stdio.h>

int stepQuantize(Ft_Esd_RangeSliderInterval *context, int Min, int Max, int Step, int Value);

ESD_METHOD(updateLimits, Context = Ft_Esd_RangeSliderInterval, Type = void)
ESD_PARAMETER(min, Type = int)
ESD_PARAMETER(max, Type = int)
void updateLimits(Ft_Esd_RangeSliderInterval *context, int min, int max)
{
	if (context->inputLeft < min || context->inputLeft > max)
		context->inputLeft = min;

	if (context->inputRight < min || context->inputRight > max)
		context->inputRight = max;
}

ESD_METHOD(updateFunction, Context = Ft_Esd_RangeSliderInterval, Type = void)
ESD_PARAMETER(GlobalWidgetX, Type = int)
ESD_PARAMETER(GlobalWidgetY, Type = int)
ESD_PARAMETER(WidgetWidth, Type = int)
ESD_PARAMETER(WidgetHeight, Type = int)
ESD_PARAMETER(TouchX, Type = int)
ESD_PARAMETER(TouchY, Type = int)
ESD_PARAMETER(min, Type = int)
ESD_PARAMETER(max, Type = int)
ESD_PARAMETER(step, Type = int)

void updateFunction(Ft_Esd_RangeSliderInterval *context, int GlobalWidgetX, int GlobalWidgetY, int WidgetWidth, int WidgetHeight, int TouchX, int TouchY, int min, int max, int step)
{
	if ((TouchX >= GlobalWidgetX) && (TouchX < (GlobalWidgetX + WidgetWidth + 20)) && (TouchY >= GlobalWidgetY) && (TouchY <= (GlobalWidgetY + WidgetHeight)))
	{
		int positionValue = min + (((TouchX - GlobalWidgetX) * (max - min)) / WidgetWidth);
		int middle = (context->inputLeft + context->inputRight) / 2;
		if (positionValue <= middle)
		{
			context->inputLeft = stepQuantize(context, min, max, step, positionValue);
		}
		else
		{
			context->inputRight = stepQuantize(context, min, max, step, positionValue);
		}
	}
}

ESD_METHOD(stepQuantize, Context = Ft_Esd_RangeSliderInterval, Type = int)
ESD_PARAMETER(Min, Type = int)
ESD_PARAMETER(Max, Type = int)
ESD_PARAMETER(Step, Type = int)
ESD_PARAMETER(Value, Type = int)
int stepQuantize(Ft_Esd_RangeSliderInterval *context, int Min, int Max, int Step, int Value)
{
	Value -= Min;
	int div = Value / Step;
	Value = div * Step + Min;
	return Value;
}

ESD_METHOD(RenderFunction, Context = Ft_Esd_RangeSliderInterval, Type = void)
ESD_PARAMETER(GlobalWidgetX, Type = int)
ESD_PARAMETER(GlobalWidgetY, Type = int)
ESD_PARAMETER(WidgetWidth, Type = int)
ESD_PARAMETER(WidgetHeight, Type = int)
ESD_PARAMETER(TouchX, Type = int)
ESD_PARAMETER(TouchY, Type = int)
ESD_PARAMETER(LeftValue, Type = int)
ESD_PARAMETER(RightValue, Type = int)
ESD_PARAMETER(min, Type = int)
ESD_PARAMETER(max, Type = int)
ESD_PARAMETER(theme, Type = Ft_Esd_Theme *)
ESD_PARAMETER(step, Type = int)
ESD_PARAMETER(markerlines, Type = ft_bool_t)
void RenderFunction(Ft_Esd_RangeSliderInterval *context, int GlobalWidgetX, int GlobalWidgetY, int WidgetWidth, int WidgetHeight, int TouchX, int TouchY, int LeftValue, int RightValue, int min, int max, Ft_Esd_Theme *theme, int step, ft_bool_t markerlines)
{
	EVE_HalContext *phost = Esd_GetHost();

	int markercount = ((max - min) / step) + 1;
	int markerheight = WidgetHeight / 4;
	int LeftHandleX = GlobalWidgetX + (((LeftValue - min) * (WidgetWidth)) / (max - min));
	int RightHandleX = GlobalWidgetX + (((RightValue - min) * (WidgetWidth)) / (max - min));

	EVE_CoDl_vertexFormat(phost, 3);

	// Line
	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetDefaultColor(theme));
	EVE_CoDl_colorA(phost, 255);
	EVE_CoDl_begin(phost, LINES);
	EVE_CoDl_lineWidth(phost, WidgetHeight);
	EVE_CoDl_vertex2f(phost, 8 * (GlobalWidgetX), 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_vertex2f(phost, 8 * (GlobalWidgetX + WidgetWidth), 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_end(phost);

	// The Interval between start point and end point
	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
	EVE_CoDl_colorA(phost, 255);
	EVE_CoDl_begin(phost, LINES);
	EVE_CoDl_lineWidth(phost, WidgetHeight);
	EVE_CoDl_vertex2f(phost, 8 * LeftHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_vertex2f(phost, 8 * RightHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_end(phost);

	// Left Circle
	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
	EVE_CoDl_begin(phost, POINTS);
	EVE_CoDl_pointSize(phost, WidgetHeight * 2);
	EVE_CoDl_vertex2f(phost, 8 * LeftHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_colorRgb(phost, 255, 255, 255);
	EVE_CoDl_pointSize(phost, WidgetHeight + (WidgetHeight / 4));
	EVE_CoDl_vertex2f(phost, 8 * LeftHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_end(phost);

	// Right Circle
	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
	EVE_CoDl_begin(phost, POINTS);
	EVE_CoDl_pointSize(phost, WidgetHeight * 2);
	EVE_CoDl_vertex2f(phost, 8 * RightHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_colorRgb(phost, 255, 255, 255);
	EVE_CoDl_pointSize(phost, WidgetHeight + (WidgetHeight / 4));
	EVE_CoDl_vertex2f(phost, 8 * RightHandleX, 8 * (GlobalWidgetY + (WidgetHeight / 2)));
	EVE_CoDl_end(phost);

	// Marker Lines
	if (markerlines)
	{
		EVE_CoDl_begin(phost, LINES);
		EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
		EVE_CoDl_colorA(phost, 255);
		EVE_CoDl_lineWidth(phost, WidgetWidth / 16);
		for (int iter = 0; iter < markercount; iter++)
		{
			EVE_CoDl_vertex2f(phost, 8 * (GlobalWidgetX + ((iter * step * WidgetWidth) / (max - min))), 8 * (GlobalWidgetY + WidgetHeight - markerheight));
			EVE_CoDl_vertex2f(phost, 8 * (GlobalWidgetX + ((iter * step * WidgetWidth) / (max - min))), 8 * (GlobalWidgetY + WidgetHeight));
		}
		EVE_CoDl_end(phost);
	}
}
