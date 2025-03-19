#include "Ft_Esd_RingSlider.h"
#include "Ft_Esd.h"
#include <math.h>

#include "Ft_Esd_TouchTag.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

ESD_METHOD(Ft_Esd_Circle_Render_Func, Context = Ft_Esd_RingSlider)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(radius, Type = int)
ESD_PARAMETER(origin, Type = int)
ESD_PARAMETER(angle, Type = int)
ESD_PARAMETER(ringWidth, Type = int)
ESD_PARAMETER(clockwise, Type = bool)
ESD_PARAMETER(color, Type = ft_argb32_t)
void Ft_Esd_Circle_Render_Func(Ft_Esd_RingSlider *context, int x, int y, int width, int height, int radius, int origin, int angle, int ringWidth, bool clockwise, ft_argb32_t color);

void Ft_Esd_Circle_Render_Func(Ft_Esd_RingSlider *context, int x, int y, int width, int height, int radius, int origin, int angle, int ringWidth, bool clockwise, ft_argb32_t color)
{
	EVE_HalContext *phost = Esd_GetHost();
	EVE_CoDl_colorArgb_ex(phost, color);

	// Center position
	x = x + (width / 2);
	y = y + (height / 2);

	// Use whichever is smaller as reference
	int RadiusOfRing = 0;
	if (width < height)
		RadiusOfRing = width / 2;
	else
		RadiusOfRing = height / 2;

	double DistanceFrCenter = RadiusOfRing - (ringWidth / 2);

	double tempAngle = 0;
	double opposite = 0;
	double adjacent = 0;

	if (clockwise)
		tempAngle = ((360 - origin) + (360 - angle)) % 360;
	else
		tempAngle = (origin + angle) % 360;

	if (tempAngle == 0)
		tempAngle = 360;

	tempAngle = tempAngle * M_PI / 180.0f;
	opposite = sin(tempAngle) * DistanceFrCenter;
	adjacent = opposite / tan(tempAngle);

	Ft_Esd_CircleLine_Draw_Point(x + (int)adjacent, y - (int)opposite, radius);
	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
	EVE_CoDl_tag(phost, 255);
}

void Ft_Esd_RingSlider_Update_TouchTracker_NoWrap(Ft_Esd_RingSlider *context)
{
	int step = (context->TrackAngle - context->StartTrackAngle) / 1024;
	int currAngle = context->StartAngle;
	int newAngle = currAngle;
	ft_int16_t max = context->ESD_Partial_Ring.MaxAngle(context->ESD_Partial_Ring.Owner);
	ft_int16_t min = 0;
	while (step > 180)
	{
		step -= 360;
	}
	while (step < -180)
	{
		step += 360;
	}

	int adjustAngle = (!!context->Clockwise(context->Owner)) ? (currAngle + step) : (currAngle - step);

	if (adjustAngle > max)
	{
		adjustAngle = max;
		context->IsOutOfBound = FT_TRUE;
	}
	else if (adjustAngle < min)
	{
		adjustAngle = min;
		context->IsOutOfBound = FT_TRUE;
	}
	else if (currAngle != adjustAngle)
	{
		context->IsOutOfBound = FT_FALSE;
	}
	// if (step != 0) 	//eve_printf_debug(" Start: %d, Pre: %d, Track: %d, StartTrack: %d, ", context->StartAngle, context->PreAngle, context->TrackAngle, context->StartTrackAngle);
	context->NewAngle = newAngle;
	context->ClampedNewAngle = adjustAngle;
	context->StartAngle = context->ClampedNewAngle;
	context->StartTrackAngle += step * 1024;
	// if (step != 0) 	//eve_printf_debug(" step: %d, new: %d, adjust: %d ", step, newAngle, adjustAngle, context->PreAngle);
}

ESD_METHOD(Ft_Esd_RingSlider_Update_TouchTracker, Context = Ft_Esd_RingSlider)
ESD_PARAMETER(tag, Type = ft_int16_t, Default = 255)
void Ft_Esd_RingSlider_Update_TouchTracker(Ft_Esd_RingSlider *context, ft_int16_t tag)
{
	EVE_HalContext *phost = Esd_GetHost();

	// ...
	void *owner = context->Owner;
	int x = context->Widget.GlobalX + (context->Widget.GlobalWidth >> 1);
	int y = context->Widget.GlobalY + (context->Widget.GlobalHeight >> 1);
	// eve_printf_debug("\n x: %d, y: %d tag: %d", x, y, tag);
	Ft_Gpu_CoCmd_Track(Ft_Esd_Host, x, y, 1, 1, tag);

	ft_uint32_t tmp = Ft_Gpu_Hal_Rd32(Ft_Esd_Host, REG_TRACKER);
	int tagId = tmp & 0xFF;

	if (tagId == tag)
	{
		int angle = ((tmp >> (16 + 4)) * 90 + (90 << 10)) % (360 << 10);
		if ((tmp >> 16) == 0)
			return;

		context->TrackAngle = angle;
		// eve_printf_debug(" angle: %d,[%d] tagRd: %d ", angle, tmp >> (16 + 4), tagId);
		if (context->IsTracking)
		{
			Ft_Esd_RingSlider_Update_TouchTracker_NoWrap(context);
		}
		else
		{
			context->IsTracking = FT_TRUE;
			context->StartTrackAngle = context->TrackAngle;
			if (context->ClampedNewAngle < 0)
				context->ClampedNewAngle += 360;
			else if (context->ClampedNewAngle > 360)
				context->ClampedNewAngle -= 360;
			context->StartAngle = context->ClampedNewAngle;
			// eve_printf_debug(" >> Start: %d, track: %d\n", context->StartAngle, context->StartTrackAngle);
		}
	}
}

/* end of file */
