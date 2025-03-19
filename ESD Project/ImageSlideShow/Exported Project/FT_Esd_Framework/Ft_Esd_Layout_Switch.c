
#include "Ft_Esd_Layout_Switch.h"

ESD_CORE_EXPORT void Esd_CoWidget_PopupSpinner();


static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Switch__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Layout_Switch_Update,
	(void (*)(void *))Ft_Esd_Layout_Switch_Render,
	(void (*)(void *))Ft_Esd_Layout_Switch_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Switch__Initializer(Ft_Esd_Layout_Switch *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Switch_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Switch__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
	context->Spinner = FT_TRUE;
	// context->Scissor = FT_FALSE;
	context->AutoResize = FT_FALSE;
	context->Current = 0;
	context->Next = 0;
}

void Ft_Esd_Layout_Switch_Update(Ft_Esd_Layout_Switch *context)
{
	Ft_Esd_Widget *next = context->Next;
	Ft_Esd_Widget *current = context->Current;
	bool recalculate = context->Widget.Recalculate;
	if (recalculate)
	{
		context->Widget.Recalculate = 0;
		Ft_Esd_Widget *child;
		child = context->Widget.Last;
		while (child)
		{
			if (child->Active)
			{
				if (child == next)
				{
					current = child;
					context->Next = 0;
					break;
				}
				if (child != current)
				{
					if (context->Spinner)
					{
						next = child;
						context->Next = child;
						Esd_CoWidget_PopupSpinner();
						context->Widget.Recalculate = 1;
						break;
					}
					current = child;
					break;
				}
			}
			child = child->Previous;
		}
		child = context->Widget.Last;
		while (child)
		{
			if (child == current)
			{
				Ft_Esd_Widget_SetGlobalRect(child,
				    context->Widget.GlobalX,
				    context->Widget.GlobalY,
				    context->Widget.GlobalWidth,
				    context->Widget.GlobalHeight);
			}
			else if (child == next)
			{
				child->GlobalValid = FT_FALSE;
			}
			else
			{
				Ft_Esd_Widget_SetActive(child, FT_FALSE);
			}
			child = child->Previous;
		}
	}
	context->Current = current;
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_UPDATE);
	if (current && context->AutoResize)
	{
		Ft_Esd_Widget_SetWidth((Ft_Esd_Widget *)context, current->LocalWidth);
		Ft_Esd_Widget_SetHeight((Ft_Esd_Widget *)context, current->LocalHeight);
	}
	if (recalculate)
		Ft_Esd_Widget_CheckDefaultIdle(&context->Widget);
}

void Ft_Esd_Layout_Switch_Render(Ft_Esd_Layout_Switch *context)
{
	Ft_Esd_Widget_IterateChildActiveValidSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);
}

void Ft_Esd_Layout_Switch_Idle(Ft_Esd_Layout_Switch *context)
{
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_IDLE);
}


/* end of file */
