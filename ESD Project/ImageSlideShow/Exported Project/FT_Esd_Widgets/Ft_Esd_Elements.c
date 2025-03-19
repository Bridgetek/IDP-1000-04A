#include "Ft_Esd_Elements.h"
#include "Ft_Esd_Dl.h"

#include "math.h"

#define VERTEX2F2(x, y) VERTEX2F((x), (y))

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

void Ft_Esd_Elements_PanelSunken(ft_rgb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t width, ft_int16_t height, ft_int16_t radius)
{
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
	ft_int32_t lw = ((ft_int32_t)radius << 4) + 8;
	ft_int32_t x0 = ((ft_int32_t)x << 2) + ((ft_int32_t)radius << 2) + 4;
	ft_int32_t y0 = ((ft_int32_t)y << 2) + ((ft_int32_t)radius << 2) + 4;
	ft_int32_t x1 = ((ft_int32_t)x << 2) + ((ft_int32_t)width << 2) - 4 - ((ft_int32_t)radius << 2);
	ft_int32_t y1 = ((ft_int32_t)y << 2) + ((ft_int32_t)height << 2) - 4 - ((ft_int32_t)radius << 2);
	EVE_CoDl_begin(phost, RECTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
	Ft_Esd_Dl_LINE_WIDTH(lw);
	EVE_CoDl_vertex2f(phost, x0 + 2, y0 + 2);
	EVE_CoDl_vertex2f(phost, x1 + 4, y1 + 4);
	Ft_Esd_Dl_COLOR_RGB(0x000000);
	EVE_CoDl_vertex2f(phost, x0 - 5, y0 - 5);
	EVE_CoDl_vertex2f(phost, x1 - 1, y1 - 1);
	Ft_Esd_Dl_COLOR_RGB(color);
	EVE_CoDl_vertex2f(phost, x0 - 2, y0 - 2);
	EVE_CoDl_vertex2f(phost, x1 + 2, y1 + 2);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
}

void Ft_Esd_Elements_PanelRaised(ft_argb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t width, ft_int16_t height, ft_int16_t radius)
{
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
	ft_int32_t lw = ((ft_int32_t)radius << 4) + 12;
	ft_int32_t x0 = ((ft_int32_t)x << 2) + ((ft_int32_t)radius << 2) + 6;
	ft_int32_t y0 = ((ft_int32_t)y << 2) + ((ft_int32_t)radius << 2) + 6;
	ft_int32_t x1 = ((ft_int32_t)x << 2) + ((ft_int32_t)width << 2) - 6 - ((ft_int32_t)radius << 2);
	ft_int32_t y1 = ((ft_int32_t)y << 2) + ((ft_int32_t)height << 2) - 6 - ((ft_int32_t)radius << 2);
	EVE_CoDl_begin(phost, RECTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_COLOR_RGB(0x000000);
	Ft_Esd_Dl_LINE_WIDTH(lw);
	EVE_CoDl_vertex2f(phost, x0 + 1, y0 + 1);
	EVE_CoDl_vertex2f(phost, x1 + 7, y1 + 7);
	Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
	EVE_CoDl_vertex2f(phost, x0 - 5, y0 - 5);
	EVE_CoDl_vertex2f(phost, x1 + 1, y1 + 1);
	Ft_Esd_Dl_COLOR_RGB(color);
	EVE_CoDl_vertex2f(phost, x0 - 3, y0 - 3);
	EVE_CoDl_vertex2f(phost, x1 + 3, y1 + 3);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
}

void Ft_Esd_Elements_CircleSunken(ft_argb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t radius)
{
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
#if 0
	// Abuse CMD_CLOCK
	Ft_Gpu_CoCmd_BgColor(phost, color);
	Ft_Gpu_CoCmd_Clock(phost, x, y, radius, OPT_NOTICKS | OPT_NOHANDS, 0, 0, 0, 0);
#else
	ft_int32_t ps = ((ft_int32_t)radius << 4);
	ft_int32_t x2 = ((ft_int32_t)x << 2);
	ft_int32_t y2 = ((ft_int32_t)y << 2);
	EVE_CoDl_begin(phost, FTPOINTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_POINT_SIZE(ps);
	Ft_Esd_Dl_COLOR_A(color >> 24);
	Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
	EVE_CoDl_vertex2f(phost, x2 + 2, y2 + 2);
	Ft_Esd_Dl_COLOR_RGB(0x000000);
	EVE_CoDl_vertex2f(phost, x2 - 3, y2 - 3);
	Ft_Esd_Dl_COLOR_RGB(color);
	EVE_CoDl_vertex2f(phost, x2, y2);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
#endif
}

void Ft_Esd_Elements_CircleRaised(ft_argb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t radius)
{
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
	ft_int32_t ps = ((ft_int32_t)radius << 4);
	ft_int32_t x2 = ((ft_int32_t)x << 2);
	ft_int32_t y2 = ((ft_int32_t)y << 2);
	EVE_CoDl_begin(phost, FTPOINTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_POINT_SIZE(ps);
	Ft_Esd_Dl_COLOR_A(color >> 24);
	Ft_Esd_Dl_COLOR_RGB(0x000000);
	EVE_CoDl_vertex2f(phost, x2 + 4, y2 + 4);
	Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
	EVE_CoDl_vertex2f(phost, x2 - 2, y2 - 2);
	Ft_Esd_Dl_COLOR_RGB(color);
	EVE_CoDl_vertex2f(phost, x2, y2);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
}

void Ft_Esd_Elements_CircleFlat(ft_rgb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t radius)
{
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
	ft_int32_t ps = ((ft_int32_t)radius << 4);
	ft_int32_t x2 = ((ft_int32_t)x << 2);
	ft_int32_t y2 = ((ft_int32_t)y << 2);
	EVE_CoDl_begin(phost, FTPOINTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_POINT_SIZE(ps);
	Ft_Esd_Dl_COLOR_A(color >> 24);
	Ft_Esd_Dl_COLOR_RGB(color);
	EVE_CoDl_vertex2f(phost, x2, y2);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
}

void Ft_Esd_Elements_Panel(Ft_Esd_Theme *theme, ft_int16_t x, ft_int16_t y, ft_int16_t width, ft_int16_t height, ft_int16_t radius, ft_bool_t raised)
{
	if (!theme)
		theme = Ft_Esd_Theme_GetCurrent();
	if (raised)
		Ft_Esd_Elements_PanelRaised(theme->DefaultColor, x, y, width, height, radius);
	else
		Ft_Esd_Elements_PanelSunken(theme->BackColor, x, y, width, height, radius);
}

void Ft_Esd_Elements_Panel_Color(ft_argb32_t color, ft_int16_t x, ft_int16_t y, ft_int16_t width, ft_int16_t height, ft_int16_t radius, ft_bool_t raised)
{
	EVE_HalContext *phost = Esd_GetHost();
	if (raised)
	{
		ft_int32_t lw = ((ft_int32_t)radius << 4) + 12;
		ft_int32_t x0 = ((ft_int32_t)x << 2) + ((ft_int32_t)radius << 2) + 6;
		ft_int32_t y0 = ((ft_int32_t)y << 2) + ((ft_int32_t)radius << 2) + 6;
		ft_int32_t x1 = ((ft_int32_t)x << 2) + ((ft_int32_t)width << 2) - 6 - ((ft_int32_t)radius << 2);
		ft_int32_t y1 = ((ft_int32_t)y << 2) + ((ft_int32_t)height << 2) - 6 - ((ft_int32_t)radius << 2);
		EVE_CoDl_begin(phost, RECTS);
		EVE_CoDl_vertexFormat(phost, 2);
		Ft_Esd_Dl_COLOR_A(color >> 24);
		Ft_Esd_Dl_COLOR_RGB(0x000000);
		Ft_Esd_Dl_LINE_WIDTH(lw);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 + 1, y0 + 1);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 + 7, y1 + 7);
		Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 - 5, y0 - 5);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 + 1, y1 + 1);
		Ft_Esd_Dl_COLOR_RGB(color);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 - 3, y0 - 3);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 + 3, y1 + 3);
#if FT_ESD_DL_END
		EVE_CoDl_end(phost);
#endif
	}
	else
	{
		ft_int32_t lw = ((ft_int32_t)radius << 4) + 8;
		ft_int32_t x0 = ((ft_int32_t)x << 2) + ((ft_int32_t)radius << 2) + 4;
		ft_int32_t y0 = ((ft_int32_t)y << 2) + ((ft_int32_t)radius << 2) + 4;
		ft_int32_t x1 = ((ft_int32_t)x << 2) + ((ft_int32_t)width << 2) - 4 - ((ft_int32_t)radius << 2);
		ft_int32_t y1 = ((ft_int32_t)y << 2) + ((ft_int32_t)height << 2) - 4 - ((ft_int32_t)radius << 2);
		EVE_CoDl_begin(phost, RECTS);
		EVE_CoDl_vertexFormat(phost, 2);
		Ft_Esd_Dl_COLOR_A(color >> 24);
		Ft_Esd_Dl_COLOR_RGB(0xFFFFFF);
		Ft_Esd_Dl_LINE_WIDTH(lw);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 + 2, y0 + 2);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 + 4, y1 + 4);
		Ft_Esd_Dl_COLOR_RGB(0x000000);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 - 5, y0 - 5);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 - 1, y1 - 1);
		Ft_Esd_Dl_COLOR_RGB(color);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x0 - 2, y0 - 2);
		EVE_CoDl_vertex2f(Ft_Esd_Host, x1 + 2, y1 + 2);
#if FT_ESD_DL_END
		EVE_CoDl_end(phost);
#endif
	}
}

// Deprecated
void FT_Esd_Render_Rect_Grad(int16_t x, int16_t y, int16_t w, int16_t h, ft_argb32_t color1, ft_argb32_t color2, int16_t direction)
{
	EVE_HalContext *phost = Esd_GetHost();

	// FIXME: Use rect for parameters
	Esd_Rect16 rect = {
		.X = x,
		.Y = y,
		.Width = w,
		.Height = h,
	};

	if (color1 == color2)
	{
		// Not a gradient
		EVE_CoDl_saveContext(phost);
		Esd_Scissor_Set(rect);
		Esd_Render_Rect(x, y, w, h, 1, color1);
		EVE_CoDl_restoreContext(phost);
		return;
	}

	eve_scope()
	{
		double radius = direction * M_PI / 180.0f;
		double sine = sin(radius), cosine = cos(radius);
		int16_t x0 = x + (w >> 1);
		int16_t y0 = y + (h >> 1);
		int32_t wwhh = (int32_t)w * w + (int32_t)h * h;
		int16_t l = (int16_t)(sqrt((double)wwhh * 0.8)); // use 80% to apply gradient effect
		int16_t half = l >> 1;
		int16_t dy = (int16_t)(half * sine);
		int16_t dx = (int16_t)(half * cosine);

		Ft_Esd_Rect16 s = Ft_Esd_Dl_Scissor_Set(rect);
		Ft_Gpu_CoCmd_Gradient(Ft_Esd_Host, x0 - dx, y0 - dy, color1, x0 + dx, y0 + dy, color2);
		Ft_Esd_Dl_Scissor_Reset(s);
	}
}

void Ft_Esd_Elements_Gradient_Panel(ft_int16_t x, ft_int16_t y, ft_int16_t width, ft_int16_t height, ft_int16_t radius, ft_rgb32_t color1, ft_rgb32_t color2, ft_int16_t direction, ft_bool_t raised)
{
	const int w = 8, halfW = 4, quatorW = 2;
	Ft_Gpu_Hal_Context_t *phost = Ft_Esd_Host;
	ft_int32_t lw = ((ft_int32_t)radius << 4) + w;
	ft_int32_t x0 = ((ft_int32_t)x << 2) + ((ft_int32_t)radius << 2) + halfW;
	ft_int32_t y0 = ((ft_int32_t)y << 2) + ((ft_int32_t)radius << 2) + halfW;
	ft_int32_t x1 = ((ft_int32_t)x << 2) + ((ft_int32_t)width << 2) - halfW - ((ft_int32_t)radius << 2);
	ft_int32_t y1 = ((ft_int32_t)y << 2) + ((ft_int32_t)height << 2) - halfW - ((ft_int32_t)radius << 2);
	EVE_CoDl_begin(phost, RECTS);
	EVE_CoDl_vertexFormat(phost, 2);
	Ft_Esd_Dl_COLOR_RGB((raised) ? 0x000000 : 0xFFFFFF);
	Ft_Esd_Dl_LINE_WIDTH(lw);
	EVE_CoDl_vertex2f(phost, x0 + quatorW, y0 + quatorW);
	EVE_CoDl_vertex2f(phost, x1 + halfW, y1 + halfW);
	Ft_Esd_Dl_COLOR_RGB((raised) ? 0xFFFFFF : 0x000000);
	EVE_CoDl_vertex2f(phost, x0 - halfW, y0 - halfW);
	EVE_CoDl_vertex2f(phost, x1 - quatorW, y1 - quatorW);
#if FT_ESD_DL_END
	EVE_CoDl_end(phost);
#endif
	FT_Esd_Render_Rect_Grad(x + (radius >> 1) + 1, y + (radius >> 1) + 1, width - radius, height - radius, color1, color2, direction);
}

void Ft_Esd_Adjust_Geometry(ft_int16_t *x, ft_int16_t *y, ft_int16_t *w, ft_int16_t *h,
    ft_uint16_t wMin, ft_uint16_t hMin, ft_uint16_t wMargin, ft_uint16_t hMargin)
{
	if (*w < 0)
		*w = 0;
	if (*h < 0)
		*h = 0;

	// eve_printf_debug(">  : %d, %d, %d, %d\n", *x, *y, *w, *h);
	// eve_printf_debug(">> : %d, %d, %d, %d\n", wMin, hMin, wMargin, hMargin);

	if (*w > *h)
	{
		if (*w < wMin)
		{ // reduce height requried
			ft_int16_t reducedH = hMargin;
			*x += (reducedH >> 1);
			*y += ((*h - reducedH) >> 1);
			*w = *w - reducedH;
			*h = reducedH;
			// eve_printf_debug("0 : %d, %d, %d, %d\n", *x, *y, *w, *h);
		}
		else
		{
			*x += (*h >> 1);
			//*y = context->GlobalY;
			*w = *w - *h;
			//*h = h;
			// eve_printf_debug("1 : %d, %d, %d, %d\n", *x, *y, *w, *h);
		}
	}
	else
	{
		if (*h < hMin)
		{ // reduce width requried
			ft_int16_t reducedW = wMargin;
			*y += (reducedW >> 1);
			*x += ((*w - reducedW) >> 1);
			*h = *h - reducedW;
			*w = reducedW;
			// eve_printf_debug("2 : %d, %d, %d, %d\n", *x, *y, *w, *h);
		}
		else
		{
			*y += (*w >> 1);
			//*x = context->GlobalX;
			*h = *h - *w;
			// w = w;
			// eve_printf_debug("3 : %d, %d, %d, %d\n", *x, *y, *w, *h);
		}
	}
}

/* end of file */
