/*
PolygonWidget
C Source
*/
#include "Ft_Esd_Elements.h"
#include "Ft_Esd.h"
#include "Ft_Esd_Widget.h"
#include "Ft_Esd_Primitives.h"
#include "Ft_Esd_PolygonWidget.h"

#define WIDTH 800.00
#define HEIGHT 480.00
ft_int32_t polygonWidth = 250;
ft_int32_t polygonHeight = 250;

#define MAX_VERTEX 20
ft_int32_t X_Array[MAX_VERTEX];
ft_int32_t Y_Array[MAX_VERTEX];

ESD_METHOD(DrawPolygon, Context = Ft_Esd_PolygonWidget)
ESD_PARAMETER(color, Type = ft_argb32_t, Default = 0x00000000)
ESD_PARAMETER(fillOrBorder, Type = ft_bool_t, Default = FT_FALSE)

void drawPolygon(ft_argb32_t color, ft_bool_t fillOrBorder, int vertices)
{
	int a = ((color & 0xFF000000)) >> 24;
	int r = ((color & 0x00FF0000)) >> 16;
	int g = ((color & 0x0000FF00)) >> 8;
	int b = ((color & 0x000000FF));

	Esd_Dl_COLOR_ARGB(color);
#if (EVE_DL_OPTIMIZE)
	Esd_GetHost()->DlPrimitive = 0;
#endif
	EVE_CoDl_saveContext(Ft_Esd_Host);

	if (fillOrBorder)
	{
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, COLOR_MASK(0, 0, 0, 0));
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, STENCIL_OP(KEEP, INVERT));
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, STENCIL_FUNC(ALWAYS, 255, 255));
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, BEGIN(EDGE_STRIP_B));
	}
	else
	{
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, BEGIN(LINE_STRIP));
	}
	
	EVE_CoDl_vertexFormat(Ft_Esd_Host, 3);

	for (int i = 0; i < vertices; ++i)
	{
		if ((X_Array[i] < 0) || (Y_Array[i] < 0))
			continue;
		EVE_CoDl_vertex2f(Ft_Esd_Host, X_Array[i] << 3, Y_Array[i] << 3);
	}

	EVE_CoDl_vertex2f(Ft_Esd_Host, X_Array[0] << 3, Y_Array[0] << 3);
	EVE_CoDl_end(Ft_Esd_Host);

	if (fillOrBorder)
	{
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, COLOR_MASK(1, 1, 1, 1));
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, STENCIL_FUNC(EQUAL, 255, 255));
		Ft_Gpu_Hal_WrCmd32(Ft_Esd_Host, BEGIN(EDGE_STRIP_R));
		EVE_CoDl_vertex2f(Ft_Esd_Host, 0, 0);
		EVE_CoDl_vertex2f(Ft_Esd_Host, 0, 511 << 4);
		EVE_CoDl_end(Ft_Esd_Host);
	}

	EVE_CoDl_restoreContext(Ft_Esd_Host);
}

ESD_METHOD(Ft_Esd_PolygonWidget_DrawPolygon, Context = Ft_Esd_PolygonWidget)
ESD_PARAMETER(xPos, Type = ft_int32_t, Default = 0)
ESD_PARAMETER(yPos, Type = ft_int32_t, Default = 0)
ESD_PARAMETER(width, Type = ft_int32_t, Default = 250)
ESD_PARAMETER(height, Type = ft_int32_t, Default = 250)
ESD_PARAMETER(borderWidth, Type = ft_int16_t, Default = 0)
ESD_PARAMETER(borderColor, Type = ft_argb32_t, Default = 0x00000000)
ESD_PARAMETER(fillColor, Type = ft_argb32_t, Default = 0xFFFFFF00)
ft_void_t Ft_Esd_PolygonWidget_DrawPolygon(Ft_Esd_PolygonWidget *context,
    ft_int32_t xPos, ft_int32_t yPos,
    ft_int32_t width, ft_int32_t height,
    ft_int16_t borderWidth,
    ft_argb32_t borderColor,
    ft_argb32_t fillColor)
{
	size_t sizeX = 0, sizeY = 0;
	int* ArrayX = context->VertexsX(context, &sizeX);
	int* ArrayY = context->VertexsY(context, &sizeY);
	ft_uint32_t vertices = fmin(sizeX, sizeY);
	if(vertices > 0 && ArrayX != NULL && ArrayY != NULL)
	{
		Esd_Dl_LINE_WIDTH(borderWidth << 3);
		for(int i = 0; i < vertices; i++)
		{
			X_Array[i] = ArrayX[i] >= 0 ? ArrayX[i] + xPos : ArrayX[i];
			Y_Array[i] = ArrayY[i] >= 0 ? ArrayY[i] + yPos : ArrayY[i];
		}
		drawPolygon(fillColor, FT_TRUE, vertices);

		if (0 != borderWidth)
		{
			drawPolygon(borderColor, FT_FALSE, vertices);
		}
	}
}

/* end of file */
