#include "Ft_Esd.h"
#include "Ft_Esd_SlidingButton.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

void TextFlashing(Ft_Esd_SlidingButton *context)
{
	if (context->Direction == 1) // increasing
	{
		if (context->Alpha < 150)
			context->Alpha += 2;
		else if (context->Alpha >= 150)
			context->Direction = -1;
	}
	else // decreasing
	{
		if (context->Alpha >= 2)
			context->Alpha -= 2;
		else if (context->Alpha <= 150)
			context->Direction = 1;
	}
}

ESD_METHOD(FT_Esd_Draw_Button, Context = Ft_Esd_SlidingButton)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(s, Type = const char *)
ESD_PARAMETER(paddingX, Type = int)
ESD_PARAMETER(paddingY, Type = int)
ESD_PARAMETER(font, Type = ft_int16_t)
ESD_PARAMETER(textColor, Type = ft_rgb32_t)
ESD_PARAMETER(backgroundColor, Type = ft_argb32_t)
void FT_Esd_Draw_Button(Ft_Esd_SlidingButton *context, int x, int y, int width, int height, const char *s, int paddingX, int paddingY, ft_int16_t font, ft_rgb32_t textColor, ft_argb32_t backgroundColor)
{
	EVE_HalContext *phost = Esd_GetHost();

	if (width < height)
		return;

	int radius = width > height ? height : width;
	EVE_CoDl_lineWidth(phost, 1);
	Esd_Render_RectF(x << 4, y << 4, width << 4, height << 4, radius << 3, backgroundColor);

	TextFlashing(context);

	ft_argb32_t color = ESD_COMPOSE_RGB_ALPHA(textColor, context->Alpha);
	EVE_CoDl_colorArgb_ex(phost, color);
	EVE_CoCmd_text_ex(phost, x + radius + paddingX, y + paddingY, font, 0, 0, 0, 0, 0, s);
}

ESD_METHOD(Button_Periodic_Update, Context = Ft_Esd_SlidingButton)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(knobColor, Type = ft_argb32_t)
void Button_Periodic_Update(Ft_Esd_SlidingButton *context, int x, int y, int width, int height, ft_argb32_t knobColor)
{
	EVE_HalContext *phost = Esd_GetHost();
	int radius = width > height ? height : width;

	context->TempHolder = x + width - radius;

	ft_argb32_t color = 0xFFFFFFFF;
	EVE_CoDl_colorArgb_ex(phost, color);

	if (context->ControlActivated)
	{
		context->Knob_X = x + width - radius;

		color = ESD_COMPOSE_RGB_ALPHA(0xD3D3D3, 255);
		EVE_CoDl_colorArgb_ex(phost, color);
		Ft_Esd_CircleLine_Draw_Point(context->Knob_X + (radius / 2), y + (radius / 2), (radius / 2) + 2); // Border
		// color = ESD_COMPOSE_RGB_ALPHA(0xFFFFFF, 255);
		EVE_CoDl_colorArgb_ex(phost, knobColor);
		Ft_Esd_CircleLine_Draw_Point(context->Knob_X + (radius / 2), y + (radius / 2), radius / 2); // knob
	}
	else if (context->IsDown)
	{
		// shifted position of control knob
		if (context->Knob_X < x)
			context->Knob_X = x;
		else if (context->Knob_X > (x + width - radius))
		{
			context->Knob_X = x + width - radius;
		}
		color = ESD_COMPOSE_RGB_ALPHA(0xD3D3D3, 255);
		EVE_CoDl_colorArgb_ex(phost, color);
		Ft_Esd_CircleLine_Draw_Point(context->Knob_X + (radius / 2), y + (radius / 2), (radius / 2) + 2); // Border
		// color = ESD_COMPOSE_RGB_ALPHA(0xFFFFFF, 255);
		EVE_CoDl_colorArgb_ex(phost, knobColor);
		Ft_Esd_CircleLine_Draw_Point(context->Knob_X + (radius / 2), y + (radius / 2), radius / 2); // knob
	}
	else
	{
		// default the position of control knob
		color = ESD_COMPOSE_RGB_ALPHA(0xD3D3D3, 255);
		EVE_CoDl_colorArgb_ex(phost, color);
		Ft_Esd_CircleLine_Draw_Point(x + (radius / 2), y + (radius / 2), (radius / 2) + 2); // Border
		color = ESD_COMPOSE_RGB_ALPHA(0xFFFFFF, 255);
		// EVE_CoDl_colorArgb_ex(phost, color);
		EVE_CoDl_colorArgb_ex(phost, knobColor);
		Ft_Esd_CircleLine_Draw_Point(x + (radius / 2), y + (radius / 2), radius / 2); // knob
	}

	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
}
