#include "Ft_Esd.h"
#include "Ft_Esd_Image.h"
#include "Ft_Esd_Primitives.h"

ESD_METHOD(Ft_Esd_Image_Render_Bitmap, Context = Ft_Esd_Image)
void Ft_Esd_Image_Render_Bitmap(Ft_Esd_Image *context)
{
	Ft_Esd_BitmapCell bitmapCell = context->BitmapCell(context->Owner);
	if (!bitmapCell.Info)
		return;
	if (bitmapCell.Info->Width == 0 && bitmapCell.Info->Height == 0)
	{
		// Preload bitmap, since we need width and height here already.
		// This loads the metadata, and also loads the image into RAM_G.
		Esd_LoadBitmap(bitmapCell.Info);
	}
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
