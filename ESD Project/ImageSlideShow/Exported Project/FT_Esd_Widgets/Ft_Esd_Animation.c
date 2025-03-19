/*
Ft_Esd_Animation
C Source
*/

#include "Ft_Esd_Animation.h"

void Ft_Esd_Animation_ReserveChannel(Ft_Esd_Animation *context)
{
	if (context->Channel < 0)
	{
		for (int i = 0; i < sizeof(Esd_CurrentContext->AnimationChannelsReserved); ++i)
		{
			if ((Esd_CurrentContext->AnimationChannelsReserved & (1UL << i)) == 0)
			{
				Esd_CurrentContext->AnimationChannelsReserved |= (1UL << i);
				context->Channel = i;
			}
		}
	}
}

void Ft_Esd_Animation_FreeChannel(Ft_Esd_Animation *context)
{
	if (context->Channel >= 0)
	{
		Esd_CurrentContext->AnimationChannelsReserved &= ~(1UL << context->Channel);
		Esd_CurrentContext->AnimationChannelsSetup &= ~(1UL << context->Channel);
		context->Channel = -1;
	}
}

void Ft_Esd_Animation_PlayRestore(Ft_Esd_Animation *context, uint32_t addr)
{
	EVE_HalContext *phost = Esd_Host;
	Esd_CurrentContext->AnimationChannelsSetup |= (1UL << context->Channel);
	Esd_CurrentContext->AnimationChannelsActive |= (1UL << context->Channel);
	if (ESD_DL_IS_FLASH_ADDRESS(addr))
	{
		EVE_CoCmd_animStart(phost, context->Channel, ESD_DL_GET_FLASH_ADDRESS(addr), context->Loop);
	}
	else
	    if (EVE_CHIPID >= EVE_BT817)
	{
		EVE_CoCmd_animStartRam(phost, context->Channel, addr, context->Loop);
	}
	else
	{
		eve_printf("Animation storage type not supported\n");
		Ft_Esd_Animation_FreeChannel(context);
		context->Counter = -1;
		return;
	}
}

void Ft_Esd_Animation_PlayImpl(Ft_Esd_Animation *context, uint32_t addr)
{
	Ft_Esd_Animation_ReserveChannel(context);
	if (context->Channel >= 0)
	{
		Ft_Esd_Animation_PlayRestore(context, addr);
	}
}

// Render implementation
ESD_METHOD(Ft_Esd_Animation_Render_Impl, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_Render_Impl(Ft_Esd_Animation *context)
{
	EVE_HalContext *phost = Esd_Host;

	if (!context->Playing && !context->IsSingle)
		return;

	Esd_AnimationInfo *animationInfo = context->AnimationInfo(context->Owner);
	uint32_t addr = animationInfo ? Esd_LoadAnimation(animationInfo) : GA_INVALID;

	if (addr == GA_INVALID)
	{
		if (addr != context->LastAddr)
		{
			// eve_printf("[DEVELOPER] Animation changed to NULL or failed to load\n");
			context->Counter = 0;
			context->LastAddr = GA_INVALID;
			if (context->Channel >= 0)
			{
				EVE_CoCmd_animStop(phost, context->Channel);
				Ft_Esd_Animation_FreeChannel(context);
			}
		}
		return;
	}

	// Attempt to restart any failed starts
	if (addr != context->LastAddr)
	{
		// eve_printf("[DEVELOPER] Animation changed\n");
		context->Counter = 0;
		context->LastAddr = addr;
		if (context->Playing)
		{
			if (context->Channel >= 0)
			{
				EVE_CoCmd_animStop(phost, context->Channel);
			}
			Ft_Esd_Animation_PlayImpl(context, addr);
		}
	}
	if (context->Counter < 0)
	{
		return; // Animation disabled due to coprocessor error
	}
	if (context->Counter == 1 && Esd_CurrentContext->HasReset)
	{
		eve_printf("Animation may have failed to load, render disabled\n");
		Ft_Esd_Animation_FreeChannel(context);
		context->Counter = -1;
		return;
	}
	if (context->Playing && context->Channel < 0)
	{
		context->Counter = 0;
		Ft_Esd_Animation_PlayImpl(context, addr);
	}
	if (context->Channel >= 0 && (Esd_CurrentContext->AnimationChannelsSetup & (1UL << context->Channel)) == 0)
	{
		context->Counter = 0;
		Ft_Esd_Animation_PlayRestore(context, addr);
	}
	if (context->Channel >= 0 && context->Counter > 0 && (Esd_CurrentContext->AnimationChannelsActive & (1UL << context->Channel)) == 0)
	{
		EVE_CoCmd_animStop(phost, context->Channel);
		Ft_Esd_Animation_FreeChannel(context);
		context->Playing = false; // End of ANIM_ONCE
	}

	// Prepare context
	EVE_CoDl_colorArgb_ex(phost, context->Color);
	EVE_CoDl_vertexFormat(phost, 4);

	// Render
	Ft_Esd_Rect16 alignedRect = Ft_Esd_Primitive_GetAlignedRect(
	    context->Widget.GlobalRect.Size, animationInfo->Rect.Size, context->Align);
	int32_t renderX = context->Widget.GlobalRect.X + alignedRect.X - animationInfo->Rect.X;
	int32_t renderY = context->Widget.GlobalRect.Y + alignedRect.Y - animationInfo->Rect.Y;
	if (context->Playing)
	{
		/* WORKAROUND: Bitmap handle 0 (current handle) gets corrupted when starting the animation 4 times without stopping on BT817 HW */
		EVE_CoDl_bitmapHandle(phost, ESD_CO_SCRATCH_HANDLE);

		EVE_CoCmd_animXY(phost, context->Channel, renderX, renderY);
		EVE_CoCmd_animDraw(phost, context->Channel);
	}
	else if (context->IsSingle && context->Frame >= 0 && context->Frame < animationInfo->NumFrames)
	{
		/* WORKAROUND: Bitmap handle 0 (current handle) gets corrupted on frame 32 on BT817 HW */
		EVE_CoDl_bitmapHandle(phost, ESD_CO_SCRATCH_HANDLE);

		if (ESD_DL_IS_FLASH_ADDRESS(addr))
		{
			EVE_CoCmd_animFrame(phost, renderX, renderY, ESD_DL_GET_FLASH_ADDRESS(addr), context->Frame);
		}
		else
		    if (EVE_CHIPID >= EVE_BT817)
		{
			EVE_CoCmd_animFrameRam(phost, renderX, renderY, addr, context->Frame);
		}
		else
		{
			eve_printf("Animation storage type not supported\n");
			context->Counter = -1;
			return;
		}
	}

	// Auto resize widget
	if (context->AutoResize & ESD_AUTORESIZE_WIDTH)
		Ft_Esd_Widget_SetWidth(&context->Widget, animationInfo->Rect.Width);
	if (context->AutoResize & ESD_AUTORESIZE_HEIGHT)
		Ft_Esd_Widget_SetHeight(&context->Widget, animationInfo->Rect.Height);
	++context->Counter;
	if (context->Counter < 0)
		context->Counter = 2;
}

ESD_METHOD(Ft_Esd_Animation_Play_Signal, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_Play_Signal(Ft_Esd_Animation *context)
{
	context->Playing = true;
	context->IsSingle = false;

	EVE_HalContext *phost = Esd_Host;

	Esd_AnimationInfo *animationInfo = context->AnimationInfo(context->Owner);
	if (!animationInfo)
		return;

	uint32_t addr = Esd_LoadAnimation(animationInfo);
	if (addr == GA_INVALID)
		return;

	Ft_Esd_Animation_PlayImpl(context, addr);
	context->Counter = 0;
}

ESD_METHOD(Ft_Esd_Animation_Stop_Signal, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_Stop_Signal(Ft_Esd_Animation *context)
{
	EVE_HalContext *phost = Esd_Host;

	if (context->Channel >= 0)
	{
		EVE_CoCmd_animStop(phost, context->Channel);
		Ft_Esd_Animation_FreeChannel(context);
	}

	context->Playing = false;
	context->IsSingle = false;
}

ESD_METHOD(Ft_Esd_Animation_Slot_Signal, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_Slot_Signal(Ft_Esd_Animation *context)
{
	Ft_Esd_Animation_Stop_Signal(context);
	context->IsSingle = true;
}

ESD_METHOD(Ft_Esd_Animation_End_Signal, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_End_Signal(Ft_Esd_Animation *context)
{
	Ft_Esd_Animation_Stop_Signal(context);
}

/// A function to get the number of frames in an animation. Returns 0 if the animation passed is NULL
ESD_METHOD(Ft_Esd_Animation_GetActiveImpl, Type = bool, Context = Ft_Esd_Animation)
bool Ft_Esd_Animation_GetActiveImpl(Ft_Esd_Animation *context)
{
	if (context->Counter < 0)
		return false;
	if (context->IsSingle)
		return true;
	if (context->Channel < 0)
		return false;
	return (Esd_CurrentContext->AnimationChannelsActive & (1UL << context->Channel)) != 0;
}

ESD_METHOD(Ft_Esd_Animation_Enable_Signal, Context = Ft_Esd_Animation)
void Ft_Esd_Animation_Enable_Signal(Ft_Esd_Animation *context)
{
	if (context->AutoPlay)
		Ft_Esd_Animation_Play_Signal(context);
}

/* end of file */
