/*
Ft_Esd_Joypad
C Source
*/
#include "Ft_Esd_Joypad.h"
#include "Ft_Esd.h"
#include <math.h>

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

ESD_METHOD(Ft_Esd_Joypad_Render_Func, Context = Ft_Esd_Joypad)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
void Ft_Esd_Joypad_Render_Func(Ft_Esd_Joypad *context, int x, int y, int width, int height);

void Ft_Esd_Joypad_Render_Func(Ft_Esd_Joypad *context, int x, int y, int width, int height)
{
	EVE_HalContext *phost = Esd_GetHost();

	// Draw background circle
	// Use smaller value as reference
	int radius = (width < height ? width : height) / 2;

	if (context->DefaultBackground)
	{
		EVE_CoCmd_dl(phost, COLOR_A(140));

		// Border/Outer
		EVE_CoCmd_dl(phost, COLOR_RGB(255, 255, 255));
		Ft_Esd_CircleLine_Draw_Point(x + (width / 2), y + (height / 2), radius + 4);

		// Inner
		EVE_CoCmd_dl(phost, COLOR_A(170));
		EVE_CoCmd_dl(phost, COLOR_RGB(0, 0, 0));
		Ft_Esd_CircleLine_Draw_Point(x + (width / 2), y + (height / 2), radius);
	}
	// else
	//{
	//  ifdefault  background is not shown, create touch area without displaying
	// EVE_CoCmd_dl(phost, COLOR_A(0));
	// EVE_CoCmd_dl(phost, COLOR_RGB(150, 150, 150));
	// Ft_Esd_CircleLine_Draw_Point(x +(width/2),y+(height/2), radius);
	// }
}

ESD_METHOD(Ft_Esd_Knob_Render_Func, Context = Ft_Esd_Joypad)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(tagID, Type = int)
ESD_PARAMETER(knobX, Type = int)
ESD_PARAMETER(knobY, Type = int)
ESD_PARAMETER(knobRadius, Type = int)
ESD_PARAMETER(knobColor, Type = ft_argb32_t)
void Ft_Esd_Knob_Render_Func(Ft_Esd_Joypad *context, int x, int y, int width, int height, int tagID, int knobX, int knobY, int knobRadius, ft_argb32_t knobColor)
{
	EVE_HalContext *phost = Esd_GetHost();

	EVE_CoDl_colorArgb_ex(phost, knobColor);

	int radius = (width < height ? width : height) / 2;

	// Compute angle from origin
	double tempHeight = y - knobY;
	double tempWidth = x - knobX;
	double temp1 = abs(tempHeight);
	double temp2 = abs(tempWidth);
	double angle = atan(temp1 / temp2);

	// if(temp2 == 0)
	//	angle = 0;
	// else
	angle *= 57.2958;

	// Distance from center
	double distance = sqrt((tempHeight * tempHeight) + (tempWidth * tempWidth));

	if (context->IsDown)
	{
		// Angle with reference
		if (tempHeight >= 0 && tempWidth <= 0) // first
			context->AngleValue = angle;
		else if (tempHeight >= 0 && tempWidth >= 0) // second
			context->AngleValue = 180 - angle;
		else if (tempHeight <= 0 && tempWidth >= 0) // thrid
			context->AngleValue = 180 + angle;
		else // fourth
			context->AngleValue = 360 - angle;

		if (context->AngleValue < 0 || context->AngleValue > 360)
			context->AngleValue = -1; // set to -1 as 0 is a valid angle

		knobRadius = knobRadius > radius ? radius / 4 : knobRadius;

		// Check within pad
		if (distance <= radius)
		{
			Ft_Esd_CircleLine_Draw_Point(knobX + (width / 2), knobY + (height / 2), knobRadius);
			context->KnobX_Value = knobX + (width / 2);
			context->KnobY_Value = knobY + (height / 2);
			context->DistanceFrCenterValue = distance;
		}
		else // exit boundary
		{
			angle = angle * M_PI / 180.0f;
			double opposite = sin(angle) * radius;
			double adjacent = opposite / tan(angle);
			int centerX = x + (width / 2);
			int centerY = y + (height / 2);

			if (opposite == 0)
				adjacent = radius;

			context->DistanceFrCenterValue = radius;

			if (tempHeight >= 0 && tempWidth <= 0) // first
			{
				Ft_Esd_CircleLine_Draw_Point(centerX + (int)adjacent, centerY - (int)opposite, knobRadius);
				context->KnobX_Value = centerX + (int)adjacent;
				context->KnobY_Value = centerY - (int)opposite;
			}
			else if (tempHeight >= 0 && tempWidth >= 0) // second
			{
				Ft_Esd_CircleLine_Draw_Point(centerX - (int)adjacent, centerY - (int)opposite, knobRadius);
				context->KnobX_Value = centerX - (int)adjacent;
				context->KnobY_Value = centerY - (int)opposite;
			}
			else if (tempHeight <= 0 && tempWidth >= 0) // thrid
			{
				Ft_Esd_CircleLine_Draw_Point(centerX - (int)adjacent, centerY + (int)opposite, knobRadius);
				context->KnobX_Value = centerX - (int)adjacent;
				context->KnobY_Value = centerY + (int)opposite;
			}
			else // fourth
			{
				Ft_Esd_CircleLine_Draw_Point(centerX + (int)adjacent, centerY + (int)opposite, knobRadius);
				context->KnobX_Value = centerX + (int)adjacent;
				context->KnobY_Value = centerY + (int)opposite;
			}
		}
	}
	else
	{
		Ft_Esd_CircleLine_Draw_Point(x + (width / 2), y + (height / 2), knobRadius);
		context->KnobX_Value = x + (width / 2);
		context->KnobY_Value = y + (height / 2);

		context->DistanceFrCenterValue = 0;
		context->AngleValue = -1;
	}

	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
}
/* end of file */
