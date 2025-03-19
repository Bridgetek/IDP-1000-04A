#include "Ft_Esd.h"
#include "Ft_Esd_Numberpad.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

typedef struct
{
	int Point[9][2];

} ListOfPoints;

ESD_METHOD(Ft_Esd_Draw_Display, Context = Ft_Esd_Numberpad)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
ESD_PARAMETER(touching, Type = bool)
ESD_PARAMETER(ID, Type = int)
void Ft_Esd_Draw_Display(Ft_Esd_Numberpad *context, int x, int y, int width, int height, bool touching, int ID)
{
	// one time initialise
	if (context->Container == NULL)
	{
		printf("Initialise user pad\n");
		context->Container = esd_malloc(sizeof(ListOfPoints));
	}

	EVE_HalContext *phost = Esd_GetHost();
	width = width / 6;
	height = height / 8;

	// Draw label of the numbers
	EVE_CoCmd_text_ex(phost, x + width, y + height, context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "1");
	EVE_CoCmd_text_ex(phost, x + (3 * width), y + height, context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "2");
	EVE_CoCmd_text_ex(phost, x + (5 * width), y + height, context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "3");
	EVE_CoCmd_text_ex(phost, x + width, y + (3 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "4");
	EVE_CoCmd_text_ex(phost, x + (3 * width), y + (3 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "5");
	EVE_CoCmd_text_ex(phost, x + (5 * width), y + (3 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "6");
	EVE_CoCmd_text_ex(phost, x + width, y + (5 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "7");
	EVE_CoCmd_text_ex(phost, x + (3 * width), y + (5 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "8");
	EVE_CoCmd_text_ex(phost, x + (5 * width), y + (5 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "9");
	EVE_CoCmd_text_ex(phost, x + (3 * width), y + (7 * height), context->myFont, OPT_CENTER, 0, context->baseLine, context->capsHeight, context->xOffset, "0");
}

ESD_METHOD(Ft_Esd_Create_TouchArea, Context = Ft_Esd_Numberpad)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
void Ft_Esd_Create_TouchArea(Ft_Esd_Numberpad *context, int x, int y, int width, int height)
{
	EVE_HalContext *phost = Esd_GetHost();
	int value = (1 << 24) | 0x00FFFFFF;
	EVE_CoDl_colorArgb_ex(phost, value);

	for (int i = 1; i < 8; i += 2)
		for (int j = 1; j < 6; j += 2)
		{
			if ((i == 7 && j == 1))
				; // do nothing; do not create touch area the space beside number 0
			else if (i == 7 && j == 5)
				; // do nothing; do not create touch area the space beside number 0
			else
				Ft_Esd_CircleLine_Draw_Point(x + j * (width / 6), y + i * (height / 8), 15);
		}

	EVE_CoCmd_dl(phost, RESTORE_CONTEXT());
}

ESD_METHOD(Ft_Esd_Highlight_Keypressed, Context = Ft_Esd_Numberpad)
ESD_PARAMETER(Keypressed, Type = int)
ESD_PARAMETER(x, Type = int)
ESD_PARAMETER(y, Type = int)
ESD_PARAMETER(width, Type = int)
ESD_PARAMETER(height, Type = int)
int Ft_Esd_Highlight_Keypressed(Ft_Esd_Numberpad *context, int Keypressed, int x, int y, int width, int height)
{
	EVE_HalContext *phost = Esd_GetHost();

	ft_argb32_t color = 0x96d3d3d3;
	EVE_CoDl_colorArgb_ex(phost, color);

	if (context->isDown)
	{
		if (context->TouchPattern)
		{
			for (int i = 0; i < context->PointerIndex; i++)
			{
				Ft_Esd_CircleLine_Draw_Point(((ListOfPoints *)context->Container)->Point[i][0], ((ListOfPoints *)context->Container)->Point[i][1], (context->Radius) / 4 * 3);
			}
		}
		else
		{
			if (context->Variable)
				Ft_Esd_CircleLine_Draw_Point(((ListOfPoints *)context->Container)->Point[0][0], ((ListOfPoints *)context->Container)->Point[0][1], (context->Radius) / 4 * 3);
		}
	}
	else
	{
		if (context->Alpha < 0)
			return;

		context->Alpha -= 15;
		int value = (context->Alpha << 24) | 0x00FFFFFF;
		// printf("Decreasing alpha %d\n", context->Alpha);
		EVE_CoDl_colorArgb_ex(phost, value);
		if (context->Alpha > 0)
		{
			Ft_Esd_CircleLine_Draw_Point(((ListOfPoints *)context->Container)->Point[context->PointerIndex][0], ((ListOfPoints *)context->Container)->Point[context->PointerIndex][1], (context->Radius) / 4 * 3);
			// Ft_Esd_CircleLine_Draw_Point( context->XValue , context->YValue, (context->Radius)/4*3);
		}
	}
}

// Draw connection lines
ESD_METHOD(Ft_Esd_Connect_Lines, Context = Ft_Esd_Numberpad)
ESD_PARAMETER(curX, Type = int)
ESD_PARAMETER(curY, Type = int)
void Ft_Esd_Connect_Lines(Ft_Esd_Numberpad *context, int curX, int curY)
{
	if (!context->TouchPattern)
		return;

	EVE_HalContext *phost = Esd_GetHost();
	int index = context->PointerIndex;
	ft_argb32_t color = 0xFF24ff0c; // change color

	if (context->PointerIndex == 0)
		return;

	if (context->PointerIndex == 1)
	{
		if (context->isDown)
			Ft_Esd_Render_LineF(((ListOfPoints *)context->Container)->Point[index - 1][0] << 4, ((ListOfPoints *)context->Container)->Point[index - 1][1] << 4,
			    curX << 4, curY << 4, 25, color);
		return;
	}

	for (int x = 0; x < context->PointerIndex - 1; x++)
	{
		Ft_Esd_Render_LineF(((ListOfPoints *)context->Container)->Point[x][0] << 4, ((ListOfPoints *)context->Container)->Point[x][1] << 4,
		    ((ListOfPoints *)context->Container)->Point[x + 1][0] << 4, ((ListOfPoints *)context->Container)->Point[x + 1][1] << 4, 25, color);
	}

	if (context->isDown)
		Ft_Esd_Render_LineF(((ListOfPoints *)context->Container)->Point[index - 1][0] << 4, ((ListOfPoints *)context->Container)->Point[index - 1][1] << 4,
		    curX << 4, curY << 4, 25, color);
}

// add points
ESD_METHOD(Ft_Esd_AddPoint, Context = Ft_Esd_Numberpad)
ESD_PARAMETER(touched, Type = bool)
void Ft_Esd_AddPoint(Ft_Esd_Numberpad *context, bool touched)
{
	if (touched == false)
		return;

	int x = context->XValue;
	int y = context->YValue;

	if (context->TouchPattern) // touch pattern on
	{
		if (context->PointerIndex > 0) // more than 1 point
		{
			if (((ListOfPoints *)context->Container)->Point[context->PointerIndex - 1][0] == x && ((ListOfPoints *)context->Container)->Point[context->PointerIndex - 1][1] == y)
			{
				return;
			}
			else
			{
				((ListOfPoints *)context->Container)->Point[context->PointerIndex][0] = x;
				((ListOfPoints *)context->Container)->Point[context->PointerIndex][1] = y;
				context->PointerIndex++;
			}
		}
		else // point 0
		{
			((ListOfPoints *)context->Container)->Point[context->PointerIndex][0] = x;
			((ListOfPoints *)context->Container)->Point[context->PointerIndex][1] = y;
			context->PointerIndex++;
		}
		// printf("Add value. Current Pointer is %d\n", context->Pointer);
		if (context->PointerIndex >= 10)
		{
			context->PointerIndex = 0;
		}
	}
	else // touch pattern off
	{
		if (!context->Variable)
		{
			((ListOfPoints *)context->Container)->Point[context->PointerIndex][0] = x;
			((ListOfPoints *)context->Container)->Point[context->PointerIndex][1] = y;
			context->Variable = true;
		}
	}
}

// reset Pointer Counter
ESD_METHOD(Ft_Esd_Reset_PointerIndex, Context = Ft_Esd_Numberpad)
void Ft_Esd_Reset_PointerIndex(Ft_Esd_Numberpad *context)
{
	context->PointerIndex = 0;
}

// detect the key being pressed
ESD_METHOD(Ft_Esd_Detect_KeyDepressed, Context = Ft_Esd_Numberpad, Type = int, Buffered)
ESD_PARAMETER(X, Type = int)
ESD_PARAMETER(Y, Type = int)
ESD_PARAMETER(Width, Type = int)
ESD_PARAMETER(Height, Type = int)
ESD_PARAMETER(CurX, Type = int)
ESD_PARAMETER(CurY, Type = int)
ESD_PARAMETER(Tag, Type = int)
ESD_PARAMETER(CurTag, Type = int)
int Ft_Esd_Detect_KeyDepressed(Ft_Esd_Numberpad *context, int X, int Y, int Width, int Height, int CurX, int CurY, int Tag, int CurTag)
{
	int KeyPressed = -1;
	int W1, H1;

	W1 = Width / 3;
	H1 = Height / 4;

	if (Tag == CurTag)
	{

		context->Radius = W1 > H1 ? H1 / 2 : W1 / 2;
		int tempW = W1 / 2;
		int tempH = H1 / 2;

		if (CurY >= 0 && CurY < Y + H1) // Row 1
		{
			if (CurX >= 0 && CurX < X + W1)
			{
				KeyPressed = 1;
				context->XValue = X + tempW;
				context->YValue = Y + tempH;
			}
			else if (CurX >= X + W1 && CurX < X + (2 * W1))
			{
				KeyPressed = 2;
				context->XValue = X + 3 * tempW;
				context->YValue = Y + tempH;
			}
			else if (CurX >= X + (2 * W1) && CurX < X + (3 * W1))
			{
				KeyPressed = 3;
				context->XValue = X + 5 * tempW;
				context->YValue = Y + tempH;
			}
		}
		else if (CurY >= Y + H1 && CurY < Y + (2 * H1)) // Row 2
		{
			if (CurX >= 0 && CurX < X + W1)
			{
				KeyPressed = 4;
				context->XValue = X + tempW;
				context->YValue = Y + 3 * tempH;
			}
			else if (CurX >= X + W1 && CurX < X + (2 * W1))
			{
				KeyPressed = 5;
				context->XValue = X + 3 * tempW;
				context->YValue = Y + 3 * tempH;
			}
			else if (CurX >= X + (2 * W1) && CurX < X + (3 * W1))
			{
				KeyPressed = 6;
				context->XValue = X + 5 * tempW;
				context->YValue = Y + 3 * tempH;
			}
		}
		else if (CurY >= Y + (2 * H1) && CurY < Y + (3 * H1)) // Row 3
		{
			if (CurX >= 0 && CurX < X + W1)
			{
				KeyPressed = 7;
				context->XValue = X + tempW;
				context->YValue = Y + 5 * tempH;
			}
			else if (CurX >= X + W1 && CurX < X + (2 * W1))
			{
				KeyPressed = 8;
				context->XValue = X + 3 * tempW;
				context->YValue = Y + 5 * tempH;
			}
			else if (CurX >= X + (2 * W1) && CurX < X + (3 * W1))
			{
				KeyPressed = 9;
				context->XValue = X + 5 * tempW;
				context->YValue = Y + 5 * tempH;
			}
		}
		else
		{
			KeyPressed = 0;
			context->XValue = X + 3 * tempW;
			context->YValue = Y + 7 * tempH;
		}
	}
	return KeyPressed;
}

// Check to Emit pressed signal
ESD_METHOD(Ft_Esd_CheckEmitPressedSignal, Context = Ft_Esd_Numberpad, Type = bool, Buffered)
ESD_PARAMETER(isDown, Type = bool)
ESD_PARAMETER(keyPressed, Type = int)
bool Ft_Esd_CheckEmitPressedSignal(Ft_Esd_Numberpad *context, bool isDown, int keyPressed)
{
	bool Emit = false;

	if (keyPressed == -1)
		Emit = false;
	else if (context->Variable && !context->hasSent)
	{
		context->lastSentValue = keyPressed;
		context->hasSent = true;
		Emit = true;
	}
	else if (context->hasSent == false && keyPressed != -1)
	{
		context->lastSentValue = keyPressed;
		context->hasSent = true;
		Emit = true;
	}
	else if ((context->lastSentValue != keyPressed) && (context->hasSent) && (context->TouchPattern))
	{
		context->lastSentValue = keyPressed;
		context->hasSent = true;
		Emit = true;
	}

	return Emit;
}

ESD_METHOD(Ft_Esd_Free_Signal, Context = Ft_Esd_Numberpad)
void Ft_Esd_Free_Signal(Ft_Esd_Numberpad *context)
{
	if (context->Container != NULL)
	{
		esd_free(context->Container);
		context->Container = NULL;
	}
	// ...
}
