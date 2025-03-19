#include "Ft_Esd_Core.h"
#include "Ft_Esd_Dl.h"
#include "Ft_Esd.h"
#include "Ft_Esd_Gradient_Arc_Line.h"

#include <math.h>

extern void Ft_Esd_CircleLine_Draw_Point(int x, int y, int radius);
extern void Ft_Esd_ArcLine_GetTanglePoint(float angle, float radius, float *x, float *y, ft_bool_t isClockwise);

typedef enum
{
	TOP_RIGHT,
	BOTTOM_RIGHT,
	BOTTOM_LEFT,
	TOP_LEFT,
	LAST_SECTOR
} SectorName;

typedef enum
{
	START_SECTOR_TYPE,
	END_SECTOR_TYPE,
	LAST_SECTOR_TYPE
} SectorType;

ESD_METHOD(Ft_Esd_Gradient_Arc_Line_Func, Context = Ft_Esd_Gradient_Arc_Line)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(radius, Type = int)
ESD_PARAMETER(border, Type = int)
ESD_PARAMETER(startColor, Type = ft_argb32_t)
ESD_PARAMETER(endColor, Type = ft_argb32_t)
ESD_PARAMETER(isClockwise, Type = ft_bool_t)
ESD_PARAMETER(origin, Type = int)
ESD_PARAMETER(value, Type = int)

void Ft_Esd_Gradient_Arc_Line_Func(Ft_Esd_Gradient_Arc_Line *context, int x, int y, int radius, int border, ft_argb32_t startColor, ft_argb32_t endColor, ft_bool_t isClockwise, int origin, int value);

SectorName IndicateSector(int angleValue);

SectorName IndicateSector(int angleValue)
{
	if ((90 > angleValue) && (0 <= angleValue))
	{
		return TOP_RIGHT;
	}
	else if ((180 > angleValue) && (90 <= angleValue))
	{
		return BOTTOM_RIGHT;
	}
	else if ((270 > angleValue) && (180 <= angleValue))
	{
		return BOTTOM_LEFT;
	}
	else if ((360 > angleValue) && (270 <= angleValue))
	{
		return TOP_LEFT;
	}
	else
	{
		printf("Some thing went wrong!");
	}
}

SectorName IndicateSectorClock(int angleValue)
{
	if ((90 > angleValue) && (0 <= angleValue))
	{
		return BOTTOM_RIGHT;
	}
	else if ((180 > angleValue) && (90 <= angleValue))
	{
		return TOP_RIGHT;
	}
	else if ((270 > angleValue) && (180 <= angleValue))
	{
		return TOP_LEFT;
	}
	else if ((360 > angleValue) && (270 <= angleValue))
	{
		return BOTTOM_LEFT;
	}
	else
	{
		printf("Some thing went wrong!");
	}
}

void Ft_Esd_DrawGradientSector(SectorName sectorID, ft_bool_t isClockwise, int *pSecValue, SectorType secType, int RBig, int CX, int CY)
{
	EVE_HalContext *phost = Esd_GetHost();
	float nX, nY;
	int angleDraw;

	if (pSecValue[sectorID] >= 90) // no need to draw
	{
		return;
	}
	else if (pSecValue[sectorID] == 0)
	{
		switch (sectorID)
		{
		case TOP_RIGHT: {
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY - RBig));
			break;
		}
		case BOTTOM_RIGHT: {
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY + RBig));
			break;
		}
		case BOTTOM_LEFT: {
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY + RBig));
			break;
		}
		case TOP_LEFT: {
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
			EVE_CoCmd_dl(phost, VERTEX2F(CX, CY - RBig));
			break;
		}
		}
		return;
	}

	switch (sectorID)
	{
	case TOP_RIGHT: {
		if (secType == START_SECTOR_TYPE)
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
			angleDraw = 90 - pSecValue[TOP_RIGHT] - 90 * 1;
			CX = CX - 10;
		}
		else
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
			angleDraw = pSecValue[TOP_RIGHT] - 90 * 1;
		}
		nX = CX, nY = CY;
		EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
		Ft_Esd_ArcLine_GetTanglePoint(angleDraw, RBig, &nX, &nY, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
		break;
	}
	case BOTTOM_RIGHT: {
		if (secType == START_SECTOR_TYPE)
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
			angleDraw = 90 - pSecValue[BOTTOM_RIGHT] - 90 * 0;
		}
		else
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
			angleDraw = pSecValue[BOTTOM_RIGHT] - 90 * 0;
			CX = CX - 10;
		}
		nX = CX, nY = CY;
		EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
		Ft_Esd_ArcLine_GetTanglePoint(angleDraw, RBig, &nX, &nY, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
		break;
	}
	case BOTTOM_LEFT: {
		if (secType == START_SECTOR_TYPE)
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
			angleDraw = 90 - pSecValue[BOTTOM_LEFT] - 90 * 3;
			CX = CX - 10;
		}
		else
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
			angleDraw = pSecValue[BOTTOM_LEFT] - 90 * 3;
		}
		nX = CX, nY = CY;
		EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
		Ft_Esd_ArcLine_GetTanglePoint(angleDraw, RBig, &nX, &nY, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
		break;
	}
	case TOP_LEFT: {
		if (secType == START_SECTOR_TYPE)
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
			angleDraw = 90 - pSecValue[TOP_LEFT] - 90 * 2;
		}
		else
		{
			EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
			angleDraw = pSecValue[TOP_LEFT] - 90 * 2;
			CX = CX - 10;
		}
		nX = CX, nY = CY;
		EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
		Ft_Esd_ArcLine_GetTanglePoint(angleDraw, RBig, &nX, &nY, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
		break;
	}
	default: {
		printf("Something went wrong!\n");
		break;
	}
	}
}

void Ft_Esd_GArcLine_DrawArc(Ft_Esd_Gradient_Arc_Line *context, ft_bool_t isClockwise, int origin, int value, int RBig, int RSmall, int CX, int CY)
{
	EVE_HalContext *phost = Esd_GetHost();

	int startAngle = origin % 360;
	int endAngle = (value + (origin % 360)) % 360;
	ft_bool_t __isClockwise = isClockwise;
	// isClockwise = true;

	/// printf("startAngle: %d, endAngle: %d, isClockwise: %s\n", startAngle, endAngle, isClockwise? "true" : "false");
	int realValue = value % 360;

	/// printf("Thajjjj Start sector: %d, End sector: %d\n", IndicateSector(startAngle), IndicateSector(endAngle));
	////printf("Deen Start sector: %d, End sector: %d\n", startAngle , endAngle);

	int sectorValue[LAST_SECTOR] = { 0, 0, 0, 0 };
	memset(sectorValue, 0x00, sizeof(sectorValue));
	uint8_t startSector = IndicateSector(startAngle);
	uint8_t endSector = IndicateSector(endAngle);
	uint32_t remainAngle = value % 360;

	bool sameSector = (endSector == startSector);

	if (sameSector)
	{
		// printf("Special case same sector draw!\n");
		float nX = CX, nY = CY;

		if (remainAngle <= 270)
		{
			EVE_CoCmd_dl(phost, COLOR_A(0));
			EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
			EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));

			switch (startSector)
			{
			case TOP_RIGHT: {
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				EVE_CoCmd_dl(phost, VERTEX2F(CX-10, CY));
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));

				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
				break;
			}
			case BOTTOM_RIGHT: {
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));

				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				EVE_CoCmd_dl(phost, VERTEX2F(CX-10, CY));
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
				break;
			}
			case BOTTOM_LEFT: {
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));

				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				EVE_CoCmd_dl(phost, VERTEX2F(CX-10, CY));
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
				break;
			}
			case TOP_LEFT: {
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));

				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				EVE_CoCmd_dl(phost, VERTEX2F(CX-10, CY));
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY));
				break;
			}

			default: {
				break;
			}
			}

			for (int sectorIdx = 0; sectorIdx < LAST_SECTOR; sectorIdx++)
			{
				if (sectorIdx != startSector)
				{
					Ft_Esd_DrawGradientSector(sectorIdx, isClockwise, sectorValue, START_SECTOR_TYPE, RBig, CX, CY);
				}
			}
		}
		else // Case the value bigger than 270
		{
			for (int sectorIdx = 1; sectorIdx <= LAST_SECTOR; sectorIdx++)
			{
				int secIdx = (sectorIdx + startSector) % LAST_SECTOR;
				sectorValue[secIdx] = 90;
			}

			// Hide all object
			EVE_CoCmd_dl(phost, COLOR_A(0));

			switch (startSector)
			{
			case TOP_RIGHT: {
				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // CENTER
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Bottom angle

				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(ZERO, INCR));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				nX = CX, nY = CY;
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // Center
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Upper angle

				// This one to reset the outsize of cut part to zero
				// Because the outsize of cutting part have a difference value in stencil buffer
				// This one only change the value of one section
				EVE_CoCmd_dl(phost, STENCIL_FUNC(NOTEQUAL, 11, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				EVE_CoCmd_dl(phost, VERTEX2F(CX + RBig, CY + RBig));
				break;
			}
			case BOTTOM_RIGHT: {
				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_R));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // CENTER
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Bottom angle

				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(ZERO, INCR));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				nX = CX, nY = CY;
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // Center
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Upper angle

				// This one to reset the outsize of cut part to zero
				// Because the outsize of cutting part have a difference value in stencil buffer
				// This one only change the value of one section
				EVE_CoCmd_dl(phost, STENCIL_FUNC(NOTEQUAL, 11, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				EVE_CoCmd_dl(phost, VERTEX2F(CX + RBig, CY));
				break;
			}
			case BOTTOM_LEFT: {
				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // CENTER
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Bottom angle

				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(ZERO, INCR));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				nX = CX, nY = CY;
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // Center
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Upper angle

				// This one to reset the outsize of cut part to zero
				// Because the outsize of cutting part have a difference value in stencil buffer
				// This one only change the value of one section
				EVE_CoCmd_dl(phost, STENCIL_FUNC(NOTEQUAL, 11, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_B));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				EVE_CoCmd_dl(phost, VERTEX2F(CX - RBig, CY));
				break;
			}
			case TOP_LEFT: {
				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_L));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // CENTER
				nX = CX, nY = CY;
				Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Bottom angle

				EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(ZERO, INCR));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				nX = CX, nY = CY;
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY)); // Center
				Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig, &nX, &nY, isClockwise);
				EVE_CoCmd_dl(phost, VERTEX2F((int)nX, (int)nY)); // Upper angle

				// This one to reset the outsize of cut part to zero
				// Because the outsize of cutting part have a difference value in stencil buffer
				// This one only change the value of one section
				EVE_CoCmd_dl(phost, STENCIL_FUNC(NOTEQUAL, 11, 255));
				EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
				EVE_CoCmd_dl(phost, BEGIN(EDGE_STRIP_A));
				EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
				EVE_CoCmd_dl(phost, VERTEX2F(CX - RBig, CY));
				break;
			}

			default: {
				break;
			}
			}
		}
	}
	else // Not same sector and less than 270 degree
	{
		// Hide all object
		EVE_CoCmd_dl(phost, COLOR_A(0));

		EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 10, 255));
		EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, ZERO));

		sectorValue[startSector] = 90 - (startAngle % 90);
		remainAngle -= sectorValue[startSector];
		Ft_Esd_DrawGradientSector(startSector, isClockwise, sectorValue, START_SECTOR_TYPE, RBig, CX, CY);

		for (int sectorIdx = 1; sectorIdx < LAST_SECTOR; sectorIdx++)
		{
			// This one is the rounding buffer index
			int secIdx = (sectorIdx + startSector) % LAST_SECTOR;

			if (remainAngle >= 90)
			{
				sectorValue[secIdx] = 90;
				remainAngle -= 90;
			}
			else
			{
				sectorValue[secIdx] = remainAngle;
				remainAngle = 0;
			}
			SectorType secType = (endSector == secIdx) ? END_SECTOR_TYPE : LAST_SECTOR_TYPE;
			Ft_Esd_DrawGradientSector(secIdx, isClockwise, sectorValue, secType, RBig, CX, CY);
		}

		// Debug
		for (int i = 0; i < LAST_SECTOR; i++)
		{
			// printf("Sector %d: %d, ", i, sectorValue[i]);
		}
		// printf("\n");
	}

	// Draw an end point
	if (context->ShowEndPoint)
	{
		EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 0, 0));
		EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
		EVE_CoCmd_dl(phost, BEGIN(POINTS));
		float endPointRadius = (RBig - RSmall) / 2;
		float _x = CX, _y = CY;
		EVE_CoCmd_dl(phost, POINT_SIZE((int)endPointRadius));
		Ft_Esd_ArcLine_GetTanglePoint(endAngle - 90, RBig - endPointRadius, &_x, &_y, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)_x, (int)_y));
	}

	// Draw an start point
	if (context->ShowStartPoint)
	{
		EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 0, 0));
		EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, ZERO));
		EVE_CoCmd_dl(phost, BEGIN(POINTS));
		float endPointRadius = (RBig - RSmall) / 2;
		float _x = CX, _y = CY;
		EVE_CoCmd_dl(phost, POINT_SIZE((int)endPointRadius));
		Ft_Esd_ArcLine_GetTanglePoint(startAngle - 90, RBig - endPointRadius, &_x, &_y, isClockwise);
		EVE_CoCmd_dl(phost, VERTEX2F((int)_x, (int)_y));
	}
	// Draw the ring layer
	// Small ring
	EVE_CoCmd_dl(phost, STENCIL_FUNC(NEVER, 12, 255));
	EVE_CoCmd_dl(phost, STENCIL_OP(REPLACE, INCR));
	EVE_CoCmd_dl(phost, BEGIN(POINTS));
	EVE_CoCmd_dl(phost, POINT_SIZE(RSmall));
	EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));

	// Big ring
	EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 0, 255));
	EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, INCR));
	EVE_CoCmd_dl(phost, BEGIN(POINTS));
	EVE_CoCmd_dl(phost, POINT_SIZE(RBig - 16 * 2));
	EVE_CoCmd_dl(phost, VERTEX2F(CX, CY));
	// Draw the ring layer - END

	// Reset the color alpha channel to show the object
	EVE_CoCmd_dl(phost, COLOR_A(255));
}

void Ft_Esd_Gradient_Arc_Line_Func(Ft_Esd_Gradient_Arc_Line *context, int x, int y, int radius, int border, ft_argb32_t startColor, ft_argb32_t endColor, ft_bool_t isClockwise, int origin, int value)
{
	EVE_HalContext *phost = Esd_GetHost();
	if (value >= 360)
		value = 359;

	int GlobalX, GlobalY, GlobalW, GlobalH;
	GlobalX = context->G_X * 16;
	GlobalY = context->G_Y * 16;
	GlobalW = context->G_W * 16;
	GlobalH = context->G_H * 16;

	int CenterX = GlobalX + (GlobalW / 2);
	int CenterY = GlobalY + (GlobalH / 2);
	int RBigRing = (GlobalW > GlobalH) ? (GlobalH / 2) : (GlobalW / 2);
	int _RBigRing = RBigRing - 16 * 2;
	int RSmallRing = RBigRing - border * 16;

	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());

	Ft_Esd_Rect16 scissor = {
		.X = context->Widget.GlobalX - 1,
		.Y = context->Widget.GlobalY - 1,
		.Width = context->Widget.GlobalWidth + 2,
		.Height = context->Widget.GlobalHeight + 2
	};
	scissor = Ft_Esd_Rect16_Crop(scissor, Ft_Esd_Dl_Scissor_Get());

	// Set the boundary for widget
	EVE_CoCmd_dl(phost, SCISSOR_XY(scissor.X, scissor.Y));
	EVE_CoCmd_dl(phost, SCISSOR_SIZE(scissor.Width, scissor.Height));
	origin = origin % 360;
	if (false == isClockwise)
	{
		int newValue = value + origin;
		newValue = newValue % 360;
		origin = 360 - newValue;
	}
	isClockwise = true;
	origin = origin + 90;
	origin = origin % 360;
	Ft_Esd_GArcLine_DrawArc(context, isClockwise, origin, value, RBigRing, RSmallRing, CenterX, CenterY);
	// Paint the gradient color for ring
	EVE_CoCmd_dl(phost, STENCIL_FUNC(EQUAL, 1, 255));
	EVE_CoCmd_dl(phost, STENCIL_OP(KEEP, KEEP));
	float GStartX = (float)(context->G_X + (context->G_W / 2));
	float GStartY = (float)context->G_Y;
	float GEndX = (float)(context->G_X + (context->G_W / 2));
	float GEndY = (float)context->G_Y;
	int GradientAngle = 0;
	Ft_Esd_ArcLine_GetTanglePoint((float)GradientAngle, (context->G_W / 2), &GStartX, &GStartY, true);
	Ft_Esd_ArcLine_GetTanglePoint((float)GradientAngle + (float)180, (context->G_W / 2), &GEndX, &GEndY, true);
	EVE_CoCmd_gradient(phost, (int)GStartX, (int)GStartY, endColor, (int)GEndX, (int)GEndY, startColor);
	/// EVE_CoCmd_gradient(phost, context->G_X + (context->G_W / 2), context->G_Y, startColor, context->G_X + (context->G_W / 2), context->G_Y + context->G_H, endColor);
	// Esd_Render_MultiGradient(context->G_X, context->G_Y, context->G_W, context->G_H, startColor, endColor, startColor, endColor);
	EVE_CoCmd_dl(phost, STENCIL_FUNC(NEVER, 1, 255));
	EVE_CoCmd_dl(phost, STENCIL_OP(ZERO, ZERO));
	EVE_CoCmd_dl(phost, BEGIN(RECTS));
	EVE_CoCmd_dl(phost, VERTEX2F(GlobalX, GlobalY));
	EVE_CoCmd_dl(phost, VERTEX2F(GlobalX + GlobalW, GlobalY + GlobalH));

	// EVE_CoCmd_dl(phost, CLEAR_STENCIL(0));

	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
}
