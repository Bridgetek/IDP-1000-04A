#include "Ft_Esd.h"
#include "Ft_Esd_TextBox.h"
#include "math.h"

ESD_METHOD(Ft_Esd_TextBox_Set_Fill_Width, Context = Ft_Esd_TextBox)
void Ft_Esd_TextBox_Set_Fill_Width(Ft_Esd_TextBox *context)
{
	EVE_HalContext *phost = Esd_GetHost();
	int adjustX;
	if (context->isCenterX)
	{

		adjustX = context->AdjustX * 2;
	}
	else
	{
		adjustX = context->AdjustX;
	}
	int width = context->Widget.GlobalWidth - abs(adjustX);
	EVE_CoCmd_fillWidth(phost, width);
}
