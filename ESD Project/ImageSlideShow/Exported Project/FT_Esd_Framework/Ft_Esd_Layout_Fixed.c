#include "Ft_Esd_Layout_Fixed.h"

static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Fixed__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Layout_Fixed_Update,
	(void (*)(void *))Ft_Esd_Layout_Fixed_Render,
	(void (*)(void *))Ft_Esd_Layout_Fixed_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Fixed__Initializer(Ft_Esd_Layout_Fixed *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Fixed_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Fixed__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
}

void Ft_Esd_Layout_Fixed_Update(Ft_Esd_Layout_Fixed *context)
{
	bool recalculate = context->Widget.Recalculate;
	if (recalculate)
	{
		context->Widget.Recalculate = 0;
		Ft_Esd_Widget *child = context->Widget.First;
		while (child)
		{
			Ft_Esd_Widget *const next = child->Next;
			if (child->Active)
			{
				Ft_Esd_Widget_SetGlobalRect(child,
				    context->Widget.GlobalX + child->LocalX,
				    context->Widget.GlobalY + child->LocalY,
				    child->LocalWidth,
				    child->LocalHeight);
			}
			child = next;
		}
	}
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_UPDATE);
	if (recalculate)
		Ft_Esd_Widget_CheckDefaultIdle(&context->Widget);
}

void Ft_Esd_Layout_Fixed_Render(Ft_Esd_Layout_Fixed *context)
{
	Ft_Esd_Widget_IterateChildActiveValidSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);
}

void Ft_Esd_Layout_Fixed_Idle(Ft_Esd_Layout_Fixed *context)
{
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_IDLE);
}


/* end of file */
