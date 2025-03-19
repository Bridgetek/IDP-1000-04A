#include "Ft_Esd.h"
#include "Ft_Esd_Video.h"
#include "Ft_Esd_Primitives.h"


ESD_METHOD(Ft_Esd_Video_Render_Signal, Context = Ft_Esd_Video)
void Ft_Esd_Video_Render_Signal(Ft_Esd_Video *context)
{
	Esd_BitmapCell bitmapCell = context->VideoFile(context->Owner);
	if (!bitmapCell.Info)
		return;
	if (bitmapCell.Info->Width == 0 && bitmapCell.Info->Height == 0)
	{
		Esd_LoadBitmap(bitmapCell.Info);
	}
	Ft_Esd_Size16 bitmapSize;
	bitmapSize.Width = (bitmapCell.Info->Width * context->ScaleX) >> 16;
	bitmapSize.Height = (bitmapCell.Info->Height * context->ScaleY) >> 16;
	Ft_Esd_Size16 scaledSize = Ft_Esd_Primitive_GetScaledSize(
	    context->Widget.GlobalRect.Size, bitmapSize, context->Scaling);
	Ft_Esd_Rect16 alignedRect = Ft_Esd_Primitive_GetAlignedRect(
	    context->Widget.GlobalRect.Size, scaledSize, context->Align);
	Ft_Esd_Render_BitmapFreeform(bitmapCell, context->Color, context->Widget.GlobalRect, alignedRect, 1);
	if (context->AutoResize & ESD_AUTORESIZE_WIDTH)
		Ft_Esd_Widget_SetWidth(&context->Widget, scaledSize.Width);
	if (context->AutoResize & ESD_AUTORESIZE_HEIGHT)
		Ft_Esd_Widget_SetHeight(&context->Widget, scaledSize.Height);
}

ESD_METHOD(Ft_Esd_Video_Playing, Context = Ft_Esd_Video)
void Ft_Esd_Video_Playing(Ft_Esd_Video *context)
{
	//Esd_CoWidget_StopBgVideo();
	Esd_BitmapCell bitmapCell = context->VideoFile(context->Owner);
	//Esd_CoWidget_PlayVideo(bitmapCell, OPT_SOUND | OPT_OVERLAY | OPT_NODL);
	//Esd_CoWidget_PlayBgVideo(bitmapCell);
	if(context->Sound)
	{
		Esd_CoWidget_PlayVideo(bitmapCell, OPT_SOUND | OPT_OVERLAY | OPT_NODL);
		if(context->Loop)
		{
			Ft_Esd_Video_Playing(context);
		}
	}
	else
	{
		Esd_CoWidget_PlayBgVideo(bitmapCell);
	}
}
ESD_METHOD(Ft_Esd_Video_Loop_Play, Context = Ft_Esd_Video)
void Ft_Esd_Video_Loop_Play(Ft_Esd_Video *context)
{
	Esd_Context *ec = Esd_CurrentContext;
	if(context->Loop && ec->bAllFramePlayed)
	{
		Ft_Esd_Video_Playing(context);
	}
}

ESD_METHOD(Ft_Esd_Video_Stop_Signal, Context = Ft_Esd_Video)
void Ft_Esd_Video_Stop_Signal(Ft_Esd_Video *context)
{
	Esd_CoWidget_StopBgVideo();
}
