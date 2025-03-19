/*
Ft_Esd_LinearRoller
C Source
*/
#include "Ft_Esd_LinearRoller.h"
#include "Ft_Esd_Theme.h"
#include "Ft_Esd_Dl.h"
#include "Ft_Esd_Primitives.h"
extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

void Ft_Esd_LinearRoller_DrawHorizontalRoller(Ft_Esd_LinearRoller *context);
void Ft_Esd_LinearRoller_DrawVerticalRoller(Ft_Esd_LinearRoller *context);

ESD_METHOD(Ft_Esd_LinearRoller_TickWidth, Context = Ft_Esd_LinearRoller, Type = int)
int Ft_Esd_LinearRoller_TickWidth(Ft_Esd_LinearRoller *context)
{
	Ft_Esd_Widget *owner = context->Owner;

	int width = context->Widget.GlobalWidth;
	if (width < 1)
		return 0;

	int height = context->Widget.GlobalHeight;
	if (height < 1)
		return 0;

	int length = context->Length(owner);
	length -= (length % 2);

	return (context->Horizontal(context) ? width : height) / length;
}

ESD_METHOD(Ft_Esd_LinearRoller_InitializeCenterValueWithMinMax, Context = Ft_Esd_LinearRoller, Type = int, Buffered = true)
ESD_PARAMETER(MinInput, Type = int)
ESD_PARAMETER(MinVariable, Type = int)
ESD_PARAMETER(MaxInput, Type = int)
ESD_PARAMETER(MaxVariable, Type = int)
int Ft_Esd_LinearRoller_InitializeCenterValueWithMinMax(Ft_Esd_LinearRoller *context, int MinInput, int MinVariable, int MaxInput, int MaxVariable)
{
	Ft_Esd_Widget *owner = context->Owner;

	bool check = false;
	if (MinInput != MinVariable)
	{
		context->MinVariable = MinInput;
		check = true;
	}

	if (MaxInput != MaxVariable)
	{
		context->MaxVariable = MaxInput;
		check = true;
	}

	if (check)
		context->centerValue = MinInput;

	if (context->centerValue < MinInput)
	{
		context->centerValue = MinInput;
	}

	if (context->centerValue > MaxInput)
	{
		context->centerValue = MaxInput;
	}

	if (context->valueVariable != context->Value(owner))
	{
		context->valueVariable = context->Value(owner);
		context->centerValue = context->valueVariable;
	}
	/*
	if(context->valueVariable != context->centerValue)
	    context->centerValue = context->valueVariable;
	*/
	return context->centerValue;
}

ESD_METHOD(Ft_Esd_LinearRoller_UpdateCenterValue, Context = Ft_Esd_LinearRoller, Type = int, Buffered = true)
ESD_PARAMETER(isTouched, Type = ft_bool_t)
ESD_PARAMETER(ScrollX, Type = int)
ESD_PARAMETER(ScrollY, Type = int)
ESD_PARAMETER(tickWidth, Type = ft_double_t)
int Ft_Esd_LinearRoller_UpdateCenterValue(Ft_Esd_LinearRoller *context, ft_bool_t isTouched, int ScrollX, int ScrollY, ft_double_t tickWidth)
{
	static ft_bool_t touchexitbool = false;
	static ft_bool_t touchentrybool = true;
	int scroll = context->Horizontal(context) ? ScrollX : ScrollY;
	static int prev = 0;
	static int diff = 0;
	static int cv = 0;
	if (isTouched)
	{
		if (touchentrybool)
		{
			touchexitbool = true;
			prev = scroll;
			touchentrybool = false;
			cv = context->centerValue;
		}
		else
		{
			diff = prev - scroll;
			context->centerValue = cv + (diff / tickWidth);
		}
	}
	else
	{
		if (touchexitbool)
		{
			touchexitbool = false;
			touchentrybool = true;
			diff = 0;
		}
	}
	return context->centerValue; // no need to connect the value to the center value as the center Value already updated in the function
}

ft_argb32_t Ft_Esd_LinearRoller_GetGradientColor(ft_argb32_t colorA, ft_argb32_t colorB, float scale)
{
	if (colorA == colorB)
		return colorA;
	if (scale < 0)
		return colorA;
	else if (scale > 1)
		return colorB;

	int a0 = ((colorA & 0xFF000000)) >> 24;
	int r0 = ((colorA & 0x00FF0000)) >> 16;
	int g0 = ((colorA & 0x0000FF00)) >> 8;
	int b0 = ((colorA & 0x000000FF));

	int a1 = ((colorB & 0xFF000000)) >> 24;
	int r1 = ((colorB & 0x00FF0000)) >> 16;
	int g1 = ((colorB & 0x0000FF00)) >> 8;
	int b1 = ((colorB & 0x000000FF));

	float oppScale = 1.0f - scale;
	int a = (int)(a0 * oppScale) + (int)(a1 * scale);
	int r = (int)(r0 * oppScale) + (int)(r1 * scale);
	int g = (int)(g0 * oppScale) + (int)(g1 * scale);
	int b = (int)(b0 * oppScale) + (int)(b1 * scale);
	if (a > 255)
		a = 255;
	if (r > 255)
		r = 255;
	if (g > 255)
		g = 255;
	if (b > 255)
		b = 255;

	ft_argb32_t color = (((((a << 8) + r) << 8) + g) << 8) + b;

	return color;
}

ESD_METHOD(Ft_Esd_LinearRoller_Draw, Context = Ft_Esd_LinearRoller)
void Ft_Esd_LinearRoller_Draw(Ft_Esd_LinearRoller *context)
{
	Esd_Dl_SAVE_CONTEXT();
	Ft_Esd_Rect16 scissor = Esd_Dl_Scissor_Set(context->Widget.GlobalRect);
	if (context->Horizontal(context))
	{
		Ft_Esd_LinearRoller_DrawHorizontalRoller(context);
	}
	else
	{
		Ft_Esd_LinearRoller_DrawVerticalRoller(context);
	}
	Esd_Dl_Scissor_Reset(scissor);
	Esd_Dl_RESTORE_CONTEXT();
}

void Ft_Esd_LinearRoller_DrawHorizontalRoller(Ft_Esd_LinearRoller *context)
{
	Ft_Esd_Widget *owner = context->Owner;
	int width = context->Widget.GlobalWidth;
	if (width < 1)
		return;

	int height = context->Widget.GlobalHeight;
	if (height < 1)
		return;

	int gx = context->Widget.GlobalX;
	int gy = context->Widget.GlobalY;
	int gyc = gy + (height / 2);

	int length = context->Length(owner);
	length -= (length % 2);
	int major = context->Major;
	int minor = context->Minor;

	float tick = ((float)context->Widget.GlobalWidth / (float)length) * 16.0f;

	int max = length;
	int half = max >> 1;
	int minTick = context->MinValue(owner); // min
	int maxTick = context->MaxValue(owner); // max
	bool isTrim = context->IsTrimToRange(owner);
	int minDisplayTick = (isTrim) ? minTick : minTick - half;
	int maxDisplayTick = (isTrim) ? maxTick : maxTick + half;
	int range = maxDisplayTick - minDisplayTick;

	int tickStart = context->centerValue - half;
	int tickEnd = context->centerValue + half;

	bool displayTick = context->IsDisplayTick(owner);
	bool displayMinor = context->IsDisplayMinor(owner);

	if (context->IsDisplayAxis(owner))
	{
		int x0 = (gx * 16);
		int x1 = x0 + (context->Widget.GlobalWidth * 16);
		int y0 = gyc * 16;
		int y1 = y0;
		Ft_Esd_Render_LineF(x0, y0, min(x1, 16383), y1, 4, context->ColorAxis);
	}

	for (int tickValue = tickStart; tickValue <= tickEnd; tickValue++)
	{
		int lineLength = height;
		int lineWidth = 0;
		bool isMajor = 0;
		bool isNeedTrim = context->IsTrimToRange(owner);

		if (tickValue % major == 0) // Draw major
		{
			lineLength = (int)(lineLength * context->MajorLength(owner));
			lineWidth = context->MajorWidth(owner);
			isMajor = FT_TRUE;
		}
		else if (displayMinor && (tickValue % minor == 0)) // Draw minor
		{
			lineLength = (int)(lineLength * context->MinorLength(owner));
			lineWidth = context->MinorWidth(owner);
		}
		else if (displayTick) // Draw tick
		{
			lineLength = (int)(lineLength * context->TickLength(owner));
			lineWidth = context->TickWidth(owner);
		}

		if (lineWidth > 0)
		{
			isNeedTrim = isTrim && (tickValue < minDisplayTick || tickValue > maxDisplayTick);
			bool showTrim = context->ShowTrimRange(owner);
			ft_argb32_t color = Ft_Esd_LinearRoller_GetGradientColor(context->ColorMin, context->ColorMax, ((float)tickValue - minTick) / range);
			if (isNeedTrim && !showTrim)
			{
				continue;
			}
			else if (isNeedTrim && showTrim)
			{
				color = Ft_Esd_LinearRoller_GetGradientColor(color, 0xFF000000, 0.85f);
			}

			int x = (int)((tickValue - tickStart) * tick);
			int x0 = (gx * 16) + (x);
			int x1 = x0;
			int halfLineLength = lineLength >> 1;
			int y0 = (gyc + halfLineLength) * 16;
			int y1 = (gyc - halfLineLength) * 16;
			Ft_Esd_Render_LineF(x0, y0, x1, y1, lineWidth << 3, color);
			if (isMajor && !isNeedTrim)
			{
				Ft_Esd_Dl_COLOR_A((color & 0xFF000000) >> 24);
				Ft_Esd_Dl_COLOR_RGB(color);
				ft_uint8_t font = Ft_Esd_Dl_RomFont_Setup(context->Font);
				int padding = context->LabelPadding(owner);
				Ft_Esd_Cmd_Number(x0 / 16, (y0 / 16) + padding, font, OPT_SIGNED | OPT_CENTERX, tickValue);
			}
		}
	}

	// Draw center line
	{
		int x0 = (gx * 16) + ((width * 16) / 2);
		int x1 = x0;
		int height2 = (int)(height * 16 * context->MajorLength(owner) + 48);
		int y0 = gyc * 16 - height2 / 2;
		int y1 = y0 + height2;
		int width = context->MajorWidth(owner) + 2;
		int tickValue = context->centerValue;
		context->centerColor = Ft_Esd_LinearRoller_GetGradientColor(context->ColorMin, context->ColorMax, ((float)tickValue - minTick) / range);
		Ft_Esd_Render_LineF(x0, y0, x1, y1, width << 3, context->centerColor);
	}
}

void Ft_Esd_LinearRoller_DrawVerticalRoller(Ft_Esd_LinearRoller *context)
{
	Ft_Esd_Widget *owner = context->Owner;
	int width = context->Widget.GlobalWidth;
	if (width < 1)
		return;

	int height = context->Widget.GlobalHeight;
	if (height < 1)
		return;

	int gx = context->Widget.GlobalX;
	int gy = context->Widget.GlobalY;
	int gxc = gx + (width / 2);

	int length = context->Length(owner);
	length -= (length % 2);
	int major = context->Major;
	int minor = context->Minor;

	float tick = ((float)context->Widget.GlobalWidth / (float)length) * 16.0f;

	int max = length;
	int half = max >> 1;
	int minTick = context->MinValue(owner); // min
	int maxTick = context->MaxValue(owner); // max
	bool isTrim = context->IsTrimToRange(owner);
	int minDisplayTick = (isTrim) ? minTick : minTick - half;
	int maxDisplayTick = (isTrim) ? maxTick : maxTick + half;
	int range = maxDisplayTick - minDisplayTick;

	int tickStart = context->centerValue - half;
	int tickEnd = context->centerValue + half;

	bool displayTick = context->IsDisplayTick(owner);
	bool displayMinor = context->IsDisplayMinor(owner);

	if (context->IsDisplayAxis(owner))
	{
		int y0 = (gy * 16);
		int y1 = y0 + (context->Widget.GlobalHeight * 16);
		int x0 = gxc * 16;
		int x1 = gxc * 16;
		Ft_Esd_Render_LineF(x0, y0, x1, min(y1, 16383), 4, context->ColorAxis);
	}

	for (int tickValue = tickStart; tickValue <= tickEnd; tickValue++)
	{
		int lineLength = width;
		int lineWidth = 0;
		bool isMajor = 0;
		bool isNeedTrim = context->IsTrimToRange(owner);

		if (tickValue % major == 0) // Draw major
		{
			lineLength = (int)(lineLength * context->MajorLength(owner));
			lineWidth = context->MajorWidth(owner);
			isMajor = FT_TRUE;
		}
		else if (displayMinor && (tickValue % minor == 0)) // Draw minor
		{
			lineLength = (int)(lineLength * context->MinorLength(owner));
			lineWidth = context->MinorWidth(owner);
		}
		else if (displayTick) // Draw tick
		{
			lineLength = (int)(lineLength * context->TickLength(owner));
			lineWidth = context->TickWidth(owner);
		}

		if (lineWidth > 0)
		{
			isNeedTrim = isTrim && (tickValue < minDisplayTick || tickValue > maxDisplayTick);
			bool showTrim = context->ShowTrimRange(owner);
			ft_argb32_t color = Ft_Esd_LinearRoller_GetGradientColor(context->ColorMin, context->ColorMax, ((float)tickValue - minTick) / range);
			if (isNeedTrim && !showTrim)
			{
				continue;
			}
			else if (isNeedTrim && showTrim)
			{
				color = Ft_Esd_LinearRoller_GetGradientColor(color, 0xFF000000, 0.85f);
			}

			int y = (int)((tickValue - tickStart) * tick);
			int y0 = (gy * 16) + (y);
			int y1 = y0;
			int halfLineLength = lineLength >> 1;
			int x0 = (gxc + halfLineLength) * 16;
			int x1 = (gxc - halfLineLength) * 16;
			Ft_Esd_Render_LineF(x0, y0, x1, y1, lineWidth << 3, color);
			if (isMajor && !isNeedTrim)
			{
				Ft_Esd_Dl_COLOR_A((color & 0xFF000000) >> 24);
				Ft_Esd_Dl_COLOR_RGB(color);
				ft_uint8_t font = Ft_Esd_Dl_RomFont_Setup(context->Font);
				int padding = context->LabelPadding(owner);
				Ft_Esd_Cmd_Number(x1 / 16 - padding, (y1 / 16), font, OPT_SIGNED | OPT_CENTERY, tickValue);
			}
		}
	}

	// Draw center line
	{
		int y0 = (gy * 16) + ((height * 16) / 2);
		int y1 = y0;
		int width2 = (int)(width * 16 * context->MajorLength(owner) + 48);
		int x0 = gxc * 16 - width2 / 2;
		int x1 = x0 + width2;
		int width = context->MajorWidth(owner) + 2;
		int tickValue = context->centerValue;
		context->centerColor = Ft_Esd_LinearRoller_GetGradientColor(context->ColorMin, context->ColorMax, ((float)tickValue - minTick) / range);
		Ft_Esd_Render_LineF(x0, y0, x1, y1, width << 3, context->centerColor);
	}
}
