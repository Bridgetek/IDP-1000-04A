#include "Ft_Esd.h"
#include "Ft_Esd_Dl.h"
#include "Ft_Esd_CircleLine.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

void Ft_Esd_CircleLine_SetDSTAlpha();
void Ft_Esd_CircleLine_ClearDSTAlpha();
void Ft_Esd_CircleLine_Draw_Point(int x, int y, int radius);
void Ft_Esd_CircleLine_Draw_Point_Float(ft_float_t x, ft_float_t y, ft_float_t radius);
void Ft_Esd_CircleLine_Draw_Square(int x, int y, int side);

ESD_METHOD(Ft_Esd_CircleLine_Render_Func, Context = Ft_Esd_CircleLine)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(radius, Type = int)
ESD_PARAMETER(border, Type = int)
ESD_PARAMETER(color, Type = ft_argb32_t)
void Ft_Esd_CircleLine_Render_Func(Ft_Esd_CircleLine *context, int x, int y, int radius, int border, ft_argb32_t color);

void Ft_Esd_CircleLine_Render_Func(Ft_Esd_CircleLine *context, int x, int y, int radius, int border, ft_argb32_t color)
{
	EVE_HalContext *phost = Esd_GetHost();

	if (radius < 0)
		return;
	int a = ((color & 0xFF000000)) >> 24;
	int r = ((color & 0x00FF0000)) >> 16;
	int g = ((color & 0x0000FF00)) >> 8;
	int b = ((color & 0x000000FF));
	int innerRadius = radius - border;
	if (innerRadius < 0)
		innerRadius = 0;

#if (EVE_DL_OPTIMIZE)
	phost->DlPrimitive = 0;
#endif
	EVE_CoCmd_dl(phost, SAVE_CONTEXT());

	Ft_Esd_Rect16 scissor = Ft_Esd_Rect16_Crop(context->Widget.GlobalRect, Ft_Esd_Dl_Scissor_Get());
	EVE_CoCmd_dl(phost, SCISSOR_XY(scissor.X, scissor.Y));
	EVE_CoCmd_dl(phost, SCISSOR_SIZE(scissor.Width, scissor.Height));

	EVE_CoCmd_dl(phost, COLOR_MASK(0, 0, 0, 1));

	// Draw the rectangle covering whole circle with alpha value 0
	Ft_Esd_CircleLine_ClearDSTAlpha();
	Ft_Esd_CircleLine_Draw_Square(x - radius - 1, y - radius - 1, (radius * 2) + 2);

	// Draw outer circle with desired alpha value
	EVE_CoCmd_dl(phost, COLOR_A(a));
	Ft_Esd_CircleLine_SetDSTAlpha();
	Ft_Esd_CircleLine_Draw_Point(x, y, radius);

	// Draw inner circle with alpha value 0
	Ft_Esd_CircleLine_ClearDSTAlpha();
	if (innerRadius > 0)
	{
		Ft_Esd_CircleLine_Draw_Point(x, y, innerRadius);
	}

	// Finally draw final color
	EVE_CoCmd_dl(phost, COLOR_RGB(r, g, b));
	EVE_CoCmd_dl(phost, COLOR_A(a));
	EVE_CoCmd_dl(phost, COLOR_MASK(1, 1, 1, 1));
	EVE_CoCmd_dl(phost, BLEND_FUNC(DST_ALPHA, ONE_MINUS_DST_ALPHA));
	Ft_Esd_CircleLine_Draw_Square(x - radius, y - radius, radius * 2);
	//Reset vertex format to default
	EVE_CoDl_vertexFormat(phost, 4);
	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
}

void Ft_Esd_CircleLine_SetDSTAlpha()
{
	EVE_HalContext *phost = Esd_GetHost();

	EVE_CoCmd_dl(phost, BLEND_FUNC(SRC_ALPHA, ZERO));
}

void Ft_Esd_CircleLine_ClearDSTAlpha()
{
	EVE_HalContext *phost = Esd_GetHost();

	EVE_CoCmd_dl(phost, BLEND_FUNC(ZERO, ZERO));
}

void Ft_Esd_CircleLine_Draw_Square(int x, int y, int side)
{
	EVE_HalContext *phost = Esd_GetHost();

	EVE_CoDl_vertexFormat(phost, 3);
	EVE_CoCmd_dl(phost, BEGIN(RECTS));
	EVE_CoCmd_dl(phost, LINE_WIDTH(16));
	EVE_CoDl_vertex2f(phost, x * 8, y * 8);
	x += side;
	y += side;
	EVE_CoDl_vertex2f(phost, x * 8, y * 8);
	// EVE_CoCmd_dl(phost, END());
}

void Ft_Esd_CircleLine_Draw_Point(int x, int y, int radius)
{
	Ft_Esd_CircleLine_Draw_Point_Float((ft_float_t)x, (ft_float_t)y, (ft_float_t)radius);
}

// FIXME: Use ft_int16_f4_t as fixed point instead of float
void Ft_Esd_CircleLine_Draw_Point_Float(ft_float_t x, ft_float_t y, ft_float_t radius)
{
	EVE_HalContext *phost = Esd_GetHost();

	EVE_CoDl_vertexFormat(phost, 3);
	EVE_CoCmd_dl(phost, BEGIN(FTPOINTS));
	int iR = (int)((radius * 16.0f) + 0.5f);
	EVE_CoCmd_dl(phost, POINT_SIZE(iR));
	int iX = (int)((x * 8.0f) + 0.5f);
	int iY = (int)((y * 8.0f) + 0.5f);
	EVE_CoDl_vertex2f(phost, iX, iY);
	// EVE_CoCmd_dl(phost, END());
}
