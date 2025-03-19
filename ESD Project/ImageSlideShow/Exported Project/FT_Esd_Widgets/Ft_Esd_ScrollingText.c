#include "Ft_Esd.h"
#include "Ft_Esd_Math.h"
#include "Ft_Esd_ScrollingText.h"

ESD_METHOD(Ft_Esd_ScrollingTextCheckValue, Type = int, Context = Ft_Esd_ScrollingText)
ESD_PARAMETER(x, Type = int, Default = 0)
ESD_PARAMETER(fontBaseLine, Type = int, Default = 0)    //  fontBaseLine =  fontBaseLine + fontXoffset
ESD_PARAMETER(fontHeight, Type = int, Default = 0)		//  fontHeight =  fontHeight + fontXoffset
ESD_PARAMETER(str, Type = char *, Default = "Str")
int Ft_Esd_ScrollingTextCheckValue(Ft_Esd_ScrollingText *context, int x, int fontBaseLine, int fontHeight, char * str)
{
	EVE_HalContext *phost = Esd_GetHost();
	int size = 0;
    while (str[size] != '\0') size++;		//  find the string length
	int font = max(fontBaseLine, fontHeight);	  //  find maxmum numbers from fontBaseLine and fontHeight.
	int v = font * size * 425;
	v = v/1000;
	v = -v;
	int www = phost->Width;
	int GX = context->GlobalX;
	v = v - GX;
    www = www - GX;
	///printf("The output === %d === %d === %d === %d ==== == %s\n", x, val, www, v, str);
    if(x<v)
		x = www;
	else if(x>www)
		x=v;
	return x;
}