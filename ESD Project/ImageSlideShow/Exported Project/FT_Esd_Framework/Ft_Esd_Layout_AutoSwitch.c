
#include "Ft_Esd_Layout_AutoSwitch.h"


static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_AutoSwitch__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Layout_AutoSwitch_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Layout_AutoSwitch_Update,
	(void (*)(void *))Ft_Esd_Layout_AutoSwitch_Render,
	(void (*)(void *))Ft_Esd_Layout_AutoSwitch_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_AutoSwitch__Initializer(Ft_Esd_Layout_AutoSwitch *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_AutoSwitch_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_AutoSwitch__Slots;
	context->Widget.LocalWidth = FT_DispWidth;
	context->Widget.LocalHeight = FT_DispHeight;
	context->Current = 0;
}

void setNextPicture(Ft_Esd_Layout_AutoSwitch *context)
{
	Ft_Esd_Widget *old = context->Current;
	Ft_Esd_Widget *current = context->Current->Next;
	if (!current)
	{
		current = old->Parent->First;
	}
	context->Current = current;
	Ft_Esd_Widget_SetActive(old, false);
	Ft_Esd_Widget_SetActive(current, true);
	context->Widget.Recalculate = FT_TRUE;
}

void Ft_Esd_Layout_AutoSwitch_Start(Ft_Esd_Layout_AutoSwitch *context)
{
	context->Current = context->Widget.First;
	context->Widget.Recalculate = FT_FALSE;
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_START);
}

void Ft_Esd_Layout_AutoSwitch_Update(Ft_Esd_Layout_AutoSwitch *context)
{
	if (!context->Current)
		return;

	static ft_uint32_t time = 0;
	time += Ft_Esd_GetDeltaMs();
	if (time > context->AnimationDuration)
	{
		setNextPicture(context);
		time = 0;
	}

	Ft_Esd_Widget *current = context->Current;

	bool recalculate = context->Widget.Recalculate;
	if (recalculate)
	{

		context->Widget.Recalculate = FT_FALSE;
		Ft_Esd_Widget *child;
		child = (context->Widget).Last;

		while (child)
		{
			if (child == current)
			{
				Ft_Esd_Widget_SetGlobalRect(child,
				    context->Widget.GlobalX,
				    context->Widget.GlobalY,
				    Esd_GetHost()->Width,
				    Esd_GetHost()->Height);
			}
			else
			{
				Ft_Esd_Widget_SetActive(child, FT_FALSE);
			}
			child = child->Previous;
		}
	}

	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_UPDATE);

	if (recalculate)
		Ft_Esd_Widget_CheckDefaultIdle(&context->Widget);
}

void Ft_Esd_Layout_AutoSwitch_Render(Ft_Esd_Layout_AutoSwitch *context)
{
	Ft_Esd_Widget_IterateChildActiveValidSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);
}

void Ft_Esd_Layout_AutoSwitch_Idle(Ft_Esd_Layout_AutoSwitch *context)
{
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_IDLE);
}


/* end of file */
