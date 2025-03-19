#include "Ft_Esd.h"
#include "Ft_Esd_CircularGradientSlider.h"

ESD_METHOD(Ft_Esd_CircularGradientSlider_Function, Context = Ft_Esd_CircularGradientSlider)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(www, Type = int)
ESD_PARAMETER(hhh, Type = int)
ESD_PARAMETER(tx, Type = int)
ESD_PARAMETER(ty, Type = int)
ESD_PARAMETER(border, Type = int)
void Ft_Esd_CircularGradientSlider_Function(Ft_Esd_CircularGradientSlider *context, int x, int y, int www, int hhh, int tx, int ty, int border)
{
	int rad = 0;
	if (www < hhh)
		rad = www / 2;
	else
		rad = hhh / 2;

	int cxx = x + (www / 2);
	int cyy = y + (hhh / 2);

	context->CPX = cxx;
	context->CPY = cyy;

	context->Angle = context->Angle % 360;
	context->Origin = context->Origin % 360;

	int angle = context->AngleA;
	int origin = context->OriginA;
	int getA = context->GetA;

	float dA = (float)(angle + origin) * (3.14159265f / 180.0f);
	float dY = (float)sin(dA) * (rad);
	float dX = (float)cos(dA) * (rad);
	dY = -dY;

	int cx = (int)dX + cxx;
	int cy = (int)dY + cyy;

	if ((tx >= (cx - border)) && (tx <= (cx + border)))
	{
		if ((ty >= (cy - border)) && (ty <= (cy + border)))
		{
			int endPoint = context->Origin + context->Angle;
			int newOrigin = context->Origin;
			if (endPoint > 360)
			{
				newOrigin = context->Origin - 360;
			}
			if (getA > newOrigin)
			{
				context->Angle = getA - newOrigin;
				return;
			}
		}
	}

	dA = (float)origin * (3.14159265f / 180.0f);
	dY = (float)sin(dA) * (rad);
	dX = (float)cos(dA) * (rad);
	dY = -dY;

	int cpx = (int)dX + cxx;
	int cpy = (int)dY + cyy;

	if ((tx >= (cpx - border)) && (tx <= (cpx + border)))
	{
		if ((ty >= (cpy - border)) && (ty <= (cpy + border)))
		{
			int endPoint = context->Origin + context->Angle;
			if (endPoint > 360)
			{
				endPoint = endPoint - 360;
			}
			context->Origin = context->GetA;
			int newEndPoint = context->Origin + context->Angle;
			int newOrigin = context->Origin;
			if (newEndPoint > 360)
			{
				newOrigin = context->Origin - 360;
			}
			context->Angle = endPoint - newOrigin;
		}
	}
}
