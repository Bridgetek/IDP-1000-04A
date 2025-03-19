#include "Ft_Esd.h"
#include "Ft_Esd_RangeSlider.h"

#include "Ft_Esd_Theme_DarkOrange.h"

#include "Ft_Esd_TouchTag.h"

#include "Ft_Esd_Math.h"
#include "Ft_Esd_Elements.h"
#include "Ft_CoPro_Cmds.h"
#include "Ft_Esd_CoCmd.h"

#include <stdio.h>

ESD_METHOD(Ft_Esd_RangeSlider_ValueX, Context = Ft_Esd_RangeSlider, Type = int)
ESD_PARAMETER(GlobalWidgetX, Type = int)
ESD_PARAMETER(WidgetWidth, Type = int)
ESD_PARAMETER(TouchX, Type = int)
ESD_PARAMETER(Min, Type = int)
ESD_PARAMETER(Max, Type = int)
int Ft_Esd_RangeSlider_ValueX(Ft_Esd_RangeSlider *context, int GlobalWidgetX, int WidgetWidth, int TouchX, int Min, int Max)
{
	int touchposition = 0;
	int unitwidth = (WidgetWidth) / (Max - Min);

	if (TouchX > GlobalWidgetX + WidgetWidth)
		touchposition = WidgetWidth;
	else if (TouchX < GlobalWidgetX)
		touchposition = 0;
	else
		touchposition = TouchX - GlobalWidgetX + unitwidth / 2;

	int value = (touchposition * (Max - Min)) / WidgetWidth + Min;
	return value;
}

ESD_METHOD(Ft_Esd_RangeSlider_StepQuatizedX, Context = Ft_Esd_RangeSlider, Type = int)
ESD_PARAMETER(Min, Type = int)
ESD_PARAMETER(Max, Type = int)
ESD_PARAMETER(Step, Type = int)
ESD_PARAMETER(Value, Type = int)
int Ft_Esd_RangeSlider_StepQuatizedX(Ft_Esd_RangeSlider *context, int Min, int Max, int Step, int Value)
{

	Value -= Min;
	int div = Value / Step;
	Value = div * Step + Min;
	return Value;
}

ESD_METHOD(Ft_Esd_RangeSlider_Render_Func, Context = Ft_Esd_RangeSlider)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(min, Type = int)
ESD_PARAMETER(max, Type = int)
ESD_PARAMETER(step, Type = int)
ESD_PARAMETER(value, Type = int)
ESD_PARAMETER(theme, Type = Ft_Esd_Theme *)
ESD_PARAMETER(markerlines, Type = ft_bool_t)
ESD_PARAMETER(Alpha, Type = ft_uint8_t)
void Ft_Esd_RangeSlider_Render_Func(Ft_Esd_RangeSlider *context, int x, int y, int width, int height, int min, int max, int step, int value, Ft_Esd_Theme *theme, ft_bool_t markerlines, ft_uint8_t Alpha);

void Ft_Esd_RangeSlider_Render_Func(Ft_Esd_RangeSlider *context, int x, int y, int width, int height, int min, int max, int step, int value, Ft_Esd_Theme *theme, ft_bool_t markerlines, ft_uint8_t Alpha)
{
	EVE_HalContext *phost = Esd_GetHost();

	int basex = x;
	int basey = y;
	int height1 = height / 2;
	int width1 = width;

	int value2 = value;
	value2 -= min;
	int value3 = value2 / step;

	int value1 = (((value - min) / step) * step * width) / (max - min);

	int markerheight = height / 4;

	int min1 = min;
	int max1 = max;
	int markercount = (max1 - min1) / step + 1;
	int markerstep = (step * width) / (max1 - min1);

	int unitwidth = (width) / (max - min);
	int iter;

	int linewidth1 = height / 2;
	int linewidth2 = width / 8;

	if (height < 0 || width < 0)
		return;

	EVE_CoDl_vertexFormat(phost, 3);

	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetClearColor(theme));
	EVE_CoDl_colorA(phost, 0);
	EVE_CoDl_begin(phost, RECTS);
	EVE_CoDl_vertex2f(phost, 8 * basex, 8 * (basey));
	EVE_CoDl_vertex2f(phost, 8 * (basex + width), 8 * (basey + height));
	EVE_CoDl_end(phost);

	EVE_CoDl_colorRgb_ex(phost, (*theme).DefaultColor);
	EVE_CoDl_colorA(phost, Alpha);
	EVE_CoDl_begin(phost, LINES);
	EVE_CoDl_lineWidth(phost, linewidth1);
	EVE_CoDl_vertex2f(phost, 8 * basex, 8 * (basey + height1));
	EVE_CoDl_vertex2f(phost, 8 * (basex + width), 8 * (basey + height1));
	EVE_CoDl_end(phost);

	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
	EVE_CoDl_colorA(phost, Alpha);
	EVE_CoDl_begin(phost, LINES);
	EVE_CoDl_lineWidth(phost, linewidth1);
	EVE_CoDl_vertex2f(phost, 8 * basex, 8 * (basey + height1));
	EVE_CoDl_vertex2f(phost, 8 * (basex + value1), 8 * (basey + height1));
	EVE_CoDl_end(phost);

	if (markerlines)
	{
		EVE_CoDl_begin(phost, LINES);
		EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
		EVE_CoDl_colorA(phost, Alpha);
		EVE_CoDl_lineWidth(phost, linewidth2);
		for (iter = 0; iter < markercount; iter++)
		{
			EVE_CoDl_vertex2f(phost, 8 * (basex + ((iter * step * width) / (max - min))), 8 * (basey + height - markerheight));
			EVE_CoDl_vertex2f(phost, 8 * (basex + ((iter * step * width) / (max - min))), 8 * (basey + height));
		}
		EVE_CoDl_end(phost);
	}

	EVE_CoDl_colorRgb_ex(phost, Ft_Esd_Theme_GetPrimaryColor(theme));
	EVE_CoDl_begin(phost, POINTS);
	EVE_CoDl_pointSize(phost, (height * 4));
	EVE_CoDl_vertex2f(phost, 8 * (basex + value1), 8 * (basey + height1));
	EVE_CoDl_colorRgb(phost, 255, 255, 255);
	EVE_CoDl_pointSize(phost, (height * 3));
	EVE_CoDl_vertex2f(phost, 8 * (basex + value1), 8 * (basey + height1));
	EVE_CoDl_end(phost);
}
