
#include "Ft_Esd_Layout_Dynamic_Switch.h"
#include "Ft_Esd_Theme.h"


static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Dynamic_Switch__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Layout_Dynamic_Switch_Update,
	(void (*)(void *))Ft_Esd_Layout_Dynamic_Switch_Render,
	(void (*)(void *))Ft_Esd_Layout_Dynamic_Switch_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Dynamic_Switch__Initializer(Ft_Esd_Layout_Dynamic_Switch *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Dynamic_Switch_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Dynamic_Switch__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
	context->AutoResize = FT_FALSE;
	context->Current = 0;
	context->Next = 0;
	context->animationduration = 500L;
	context->elapsedtime = 0L;
	context->animationmode = FT_FALSE;
	context->firsthalf = FT_FALSE;
	context->secondhalf = FT_FALSE;
	Ft_Esd_Theme *theme = Ft_Esd_Theme_GetCurrent();
	context->colorinit = Ft_Esd_Theme_GetClearColor(theme);
}

void Ft_Esd_Layout_Dynamic_Switch_Update(Ft_Esd_Layout_Dynamic_Switch *context)
{
	Ft_Esd_Widget *next = context->Next;
	Ft_Esd_Widget *current = context->Current;
	if (context->Widget.Recalculate)
	{
		context->Widget.Recalculate = 0;
		Ft_Esd_Widget *child;
		child = context->Widget.Last;
		while (child)
		{
			if (child->Active)
			{
				if (child == next && !context->animationmode)
				{
					current = child;
					context->Next = 0;
					break;
				}
				if (child != current)
				{
					if (current && !context->animationmode)
					{
						next = child;
						context->Next = child;
						context->Widget.Recalculate = 1;
						context->animationmode = FT_TRUE;
						context->firsthalf = FT_TRUE;
						context->secondhalf = FT_FALSE;
						context->elapsedtime = 0L;
						break;
					}
					if (!context->animationmode)
					{
						current = child;
						break;
					}
				}
			}
			child = child->Previous;
		}

		if (context->animationmode && context->firsthalf)
		{
			context->Widget.Recalculate = 1;
			context->elapsedtime += Ft_Esd_GetDeltaMs();

			uint32_t time;
			time = ((context->animationduration >> 1) > context->elapsedtime) ? ((context->animationduration >> 1) - context->elapsedtime) : 0;

			ft_uint8_t alpha = 0;
			if (time)
			{
				alpha = ((context->animationduration - context->elapsedtime) << 8) / (context->animationduration >> 1);
			}
			else
			{
				context->firsthalf = FT_FALSE;
				context->secondhalf = FT_TRUE;
				current = next;
				next = 0;
			}

			context->color = context->colorinit + (alpha << 24);
		}
		else if (context->animationmode && context->secondhalf)
		{
			context->Widget.Recalculate = 1;
			context->elapsedtime += Ft_Esd_GetDeltaMs();

			uint32_t time;
			time = ((context->animationduration) > context->elapsedtime) ? ((context->animationduration) - context->elapsedtime) : 0;
			ft_uint8_t alpha = 0;
			if (time)
			{
				alpha = ((context->elapsedtime) << 8) / (context->animationduration >> 1);
			}
			else
			{
				context->secondhalf = FT_FALSE;
				context->animationmode = FT_FALSE;
			}
			context->color = context->colorinit + (alpha << 24);
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
}

void Ft_Esd_Layout_Dynamic_Switch_Render(Ft_Esd_Layout_Dynamic_Switch *context)
{
	EVE_HalContext *phost = Esd_GetHost();

	if (context->animationmode)
	{
		Ft_Esd_Widget_IterateChildActiveValidSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);
		EVE_CoDl_blendFunc(phost, ONE_MINUS_SRC_ALPHA, SRC_ALPHA);
		Esd_Render_Rect(0, 0, FT_DispWidth, FT_DispHeight, 0, context->color);
		EVE_CoDl_blendFunc_default(phost);
	}
	else
	{
		Ft_Esd_Widget_IterateChildActiveValidSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);
	}
}

void Ft_Esd_Layout_Dynamic_Switch_Idle(Ft_Esd_Layout_Dynamic_Switch *context)
{
	Ft_Esd_Widget_IterateChildActiveSlotReverse((Ft_Esd_Widget *)context, FT_ESD_WIDGET_IDLE);
}


/* end of file */
