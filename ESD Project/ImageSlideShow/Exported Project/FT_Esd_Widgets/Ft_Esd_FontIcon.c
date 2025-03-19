#include "Ft_Esd.h"
#include "Ft_Esd_FontIcon.h"
#include "Ft_Esd_Primitives.h"

ESD_METHOD(Ft_Esd_FontIcon_Render_Bitmap, Context = Ft_Esd_FontIcon)
void Ft_Esd_FontIcon_Render_Bitmap(Ft_Esd_FontIcon *context)
{
	Ft_Esd_BitmapCell bitmapCell = context->BitmapCell(context->Owner);
	if (!bitmapCell.Info)
		return;
	Ft_Esd_Size16 bitmapSize;
	bitmapSize.Width = (bitmapCell.Info->Width * context->ScaleX) >> 16;
	bitmapSize.Height = (bitmapCell.Info->Height * context->ScaleY) >> 16;
	Ft_Esd_Size16 scaledSize = Ft_Esd_Primitive_GetScaledSize(
	    context->Widget.GlobalRect.Size, bitmapSize, context->Scaling);
	Ft_Esd_Rect16 alignedRect = Ft_Esd_Primitive_GetAlignedRect(
	    context->Widget.GlobalRect.Size, scaledSize, context->Align);
	Ft_Esd_Render_BitmapFreeform(bitmapCell, context->Color, context->Widget.GlobalRect, alignedRect, context->MinAlpha(context->Owner));
	if (context->AutoResize & ESD_AUTORESIZE_WIDTH)
		Ft_Esd_Widget_SetWidth(&context->Widget, scaledSize.Width);
	if (context->AutoResize & ESD_AUTORESIZE_HEIGHT)
		Ft_Esd_Widget_SetHeight(&context->Widget, scaledSize.Height);
}