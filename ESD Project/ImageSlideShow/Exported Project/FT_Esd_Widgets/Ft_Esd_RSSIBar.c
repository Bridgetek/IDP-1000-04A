#include "Ft_Esd.h"
#include "Ft_Esd_RSSIBar.h"
#include "Ft_Esd_Theme.h"
#include "Ft_Esd_TouchTag.h"
#include "Ft_Esd_Math.h"
#include "Ft_Esd_Elements.h"
#include "Ft_CoPro_Cmds.h"
#include "Ft_Esd_CoCmd.h"
#include <stdio.h>
#include <time.h>
#include <assert.h>

int getFilledBar(int min, int max, int bar, int current)
{
	assert(min < max);
	assert(min < current);
	assert(current < max);
	assert(bar >= 2);
	if (min == 0 && current == 0)
		return 0;
	if (current == min)
		return 1;
	float strength_per_bar = (float)(max - min + 1) / bar; // TODO: Can we do this without float?
	int filled_bar = (int)ceil((current - min + 1) / strength_per_bar);
	return filled_bar;
}

ESD_METHOD(UpdateBarGraph, Context = Ft_Esd_RSSIBar, Type = void)
ESD_PARAMETER(GlobalX, Type = int)
ESD_PARAMETER(Width, Type = int)
ESD_PARAMETER(NumberOfBar, Type = int)
ESD_PARAMETER(MinStrength, Type = int)
ESD_PARAMETER(MaxStrength, Type = int)
void UpdateBarGraph(Ft_Esd_RSSIBar *context, int GlobalX, int Width, int NumberOfBar, int MinStrength, int MaxStrength)
{
	static int BaseX = 0; // FIXME: static is global scope! does not work with multiple bars.
	static int Repeat = 1; // FIXME: static is global scope! does not work with multiple bars.
	BaseX = GlobalX;
	static uint32_t Time = 0; // FIXME: static is global scope! does not work with multiple bars.
	Time = Time + Esd_GetDeltaMs();
	if (Time > 200)
	{
		Time = 0;
		if (Repeat < NumberOfBar)
			Repeat++;
		else
			Repeat = 1;
	}
	context->BaseValue = BaseX;
	context->RepeatValue = Repeat;
}

ESD_METHOD(RenderBarGraph, Context = Ft_Esd_RSSIBar, Type = void)
ESD_PARAMETER(X, Type = int)
ESD_PARAMETER(Y, Type = int)
ESD_PARAMETER(Width, Type = int)
ESD_PARAMETER(Height, Type = int)
ESD_PARAMETER(DefaultColor, Type = ft_argb32_t)
ESD_PARAMETER(FillingColor, Type = ft_argb32_t)
ESD_PARAMETER(MinStrength, Type = int)
ESD_PARAMETER(MaxStrength, Type = int)
ESD_PARAMETER(NumberOfBars, Type = int)
ESD_PARAMETER(CurrentStrength, Type = int)
ESD_PARAMETER(SignalSearchMode, Type = ft_bool_t)
ESD_PARAMETER(theme, Type = Ft_Esd_Theme *)
ESD_PARAMETER(BaseX, Type = int)
ESD_PARAMETER(Loop, Type = int)
void RenderBarGraph(Ft_Esd_RSSIBar *context, int X, int Y, int Width, int Height, ft_argb32_t DefaultColor, ft_argb32_t FillingColor, int MinStrength, int MaxStrength, int NumberOfBars, int CurrentStrength, ft_bool_t SignalSearchMode, Ft_Esd_Theme *theme, int BaseX, int Loop)
{
	EVE_HalContext *phost = Esd_GetHost();

	int WidthOneBar = Width / NumberOfBars;
	int HeightOneBar = Height / NumberOfBars;
	if (BaseX > X)
		BaseX = BaseX - WidthOneBar;
	if (CurrentStrength >= MaxStrength)
		CurrentStrength = MaxStrength;
	int fillBar = getFilledBar(MinStrength, MaxStrength, NumberOfBars, CurrentStrength);
	if (SignalSearchMode == true)
	{
		EVE_CoDl_vertexFormat(phost, 3);
		EVE_CoDl_begin(phost, RECTS);
		EVE_CoDl_colorRgb_ex(phost, DefaultColor);
		for (int iter = 1; iter <= NumberOfBars; iter++)
		{
			EVE_CoDl_vertex2f(phost, 8 * X, 8 * (Y + Height));
			EVE_CoDl_vertex2f(phost, 8 * (X + WidthOneBar - 2), 8 * (Y + Height - (iter * HeightOneBar)));
			X = X + WidthOneBar;
		}
		EVE_CoDl_colorRgb_ex(phost, FillingColor);
		for (int iter = 1; iter <= Loop; iter++)
		{
			EVE_CoDl_vertex2f(phost, 8 * (BaseX + ((iter - 1) * WidthOneBar)), 8 * (Y + Height));
			EVE_CoDl_vertex2f(phost, 8 * (BaseX + (iter * WidthOneBar - 2)), 8 * (Y + Height - (iter * HeightOneBar)));
		}
		EVE_CoDl_end(phost);
	}
	else
	{
		EVE_CoDl_vertexFormat(phost, 3);
		EVE_CoDl_begin(phost, RECTS);
		for (int iter = 1; iter <= NumberOfBars; iter++)
		{
			EVE_CoDl_colorRgb_ex(phost, DefaultColor);
			if (fillBar > 0)
			{
				EVE_CoDl_colorRgb_ex(phost, FillingColor);
				fillBar--;
			}
			EVE_CoDl_vertex2f(phost, 8 * X, 8 * (Y + Height));
			EVE_CoDl_vertex2f(phost, 8 * (X + WidthOneBar - 2), 8 * (Y + Height - (iter * HeightOneBar)));
			X = X + WidthOneBar;
		}
		EVE_CoDl_end(phost);
	}
}