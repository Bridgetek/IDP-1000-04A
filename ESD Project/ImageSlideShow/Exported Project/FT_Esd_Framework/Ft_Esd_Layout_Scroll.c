#include "Ft_Esd_Layout_Scroll.h"
#include "Ft_Esd_Dl.h"

static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Scroll__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Layout_Scroll_Update,
	(void (*)(void *))Ft_Esd_Layout_Scroll_Render,
	(void (*)(void *))Ft_Esd_Layout_Scroll_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Scroll__Initializer(Ft_Esd_Layout_Scroll *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Scroll_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Scroll__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
	context->MinX = 0;
	context->MaxX = 0;
	context->MinY = 0;
	context->MaxY = 0;
	context->ScrollX = 0;
	context->ScrollY = 0;
	context->Align = ESD_ALIGN_TOPFILL;
	context->Scissor = FT_FALSE;
	context->ScrollXChanged = (void *)Ft_Esd_Noop;
	context->ScrollYChanged = (void *)Ft_Esd_Noop;
	context->LastValues = context->Values;
}

void Ft_Esd_Layout_Scroll_Recalculate(Ft_Esd_Layout_Scroll *context)
{
	Ft_Esd_Widget *child;
	child = context->Widget.First;
	ft_int16_t maxWidth = 0, maxHeight = 0;
	while (child)
	{
		if (child->Active)
		{
			if (child->LocalWidth > maxWidth)
				maxWidth = child->LocalWidth;
			if (child->LocalHeight > maxHeight)
				maxHeight = child->LocalHeight;
		}
		child = child->Next;
	}
	ft_int16_t globalX = context->Widget.GlobalX;
	ft_int16_t globalY = context->Widget.GlobalY;
	ft_int16_t globalWidth = context->Widget.GlobalWidth;
	ft_int16_t globalHeight = context->Widget.GlobalHeight;
	ft_uint8_t alignX = ESD_ALIGN_HORIZONTAL(context->Align);
	ft_uint8_t alignY = ESD_ALIGN_VERTICAL(context->Align);
	ft_int16_t scrollableX = maxWidth - globalWidth;
	ft_int16_t scrollableY = maxHeight - globalHeight;
	if (scrollableX > 0)
	{
		switch (alignX)
		{
		case ESD_ALIGN_LEFTX:
			context->MinX = 0;
			context->MaxX = scrollableX;
			break;
		case ESD_ALIGN_CENTERX:
			context->MinX = -(scrollableX >> 1);
			context->MaxX = context->MinX + scrollableX;
			break;
		case ESD_ALIGN_RIGHTX:
			context->MinX = -scrollableX;
			context->MaxX = 0;
			break;
		case ESD_ALIGN_FILLX:
			context->MinX = 0;
			context->MaxX = 0;
			break;
		}
	}
	else
	{
		context->MinX = 0;
		context->MaxX = 0;
	}
	if (scrollableY > 0)
	{
		switch (alignY)
		{
		case ESD_ALIGN_TOPY:
			context->MinY = 0;
			context->MaxY = scrollableY;
			break;
		case ESD_ALIGN_CENTERY:
			context->MinY = -(scrollableY >> 1);
			context->MaxY = context->MinY + scrollableY;
			break;
		case ESD_ALIGN_BOTTOMY:
			context->MinY = -scrollableY;
			context->MaxY = 0;
			break;
		case ESD_ALIGN_FILLY:
			context->MinY = 0;
			context->MaxY = 0;
			break;
		}
	}
	else
	{
		context->MinY = 0;
		context->MaxY = 0;
	}
	child = context->Widget.First;
	ft_int16_t scrollX = min(max(context->MinX, context->ScrollX), context->MaxX);
	ft_int16_t scrollY = min(max(context->MinY, context->ScrollY), context->MaxY);
	context->ScrollX = scrollX;
	context->ScrollY = scrollY;
	while (child)
	{
		if (child->Active)
		{
			ft_int16_t childX, childY, childWidth, childHeight;
			switch (alignX)
			{
			case ESD_ALIGN_LEFTX:
				childX = globalX - scrollX;
				childWidth = child->LocalWidth;
				break;
			case ESD_ALIGN_CENTERX:
				childX = globalX - scrollX + ((globalWidth - child->LocalWidth) >> 1);
				childWidth = child->LocalWidth;
				break;
			case ESD_ALIGN_RIGHTX:
				childX = globalX - scrollX + (globalWidth - child->LocalWidth);
				childWidth = child->LocalWidth;
				break;
			case ESD_ALIGN_FILLX:
				childX = globalX;
				childWidth = globalWidth;
				break;
			}
			switch (alignY)
			{
			case ESD_ALIGN_TOPY:
				childY = globalY - scrollY;
				childHeight = child->LocalHeight;
				break;
			case ESD_ALIGN_CENTERY:
				childY = globalY - scrollY + ((globalHeight - child->LocalHeight) >> 1);
				childHeight = child->LocalHeight;
				break;
			case ESD_ALIGN_BOTTOMY:
				childY = globalY - scrollY + (globalHeight - child->LocalHeight);
				childHeight = child->LocalHeight;
				break;
			case ESD_ALIGN_FILLY:
				childY = globalY;
				childHeight = globalHeight;
				break;
			}
			Ft_Esd_Widget_ScrollGlobalRect(child, childX, childY, childWidth, childHeight);
		}
		child = child->Next;
	}
}

void Ft_Esd_Layout_Scroll_Update(Ft_Esd_Layout_Scroll *context)
{
	bool recalculateA = context->Widget.Recalculate;
	if (recalculateA || (context->Values != context->LastValues))
	{
		context->Widget.Recalculate = FT_FALSE;
		ft_int16_t lastScrollX = context->LastScrollX;
		ft_int16_t lastScrollY = context->LastScrollY;
		Ft_Esd_Layout_Scroll_Recalculate(context);
		context->LastValues = context->Values;
		ft_int16_t scrollX = context->ScrollX;
		ft_int16_t scrollY = context->ScrollY;
		if (scrollX != lastScrollX)
		{
			context->ScrollXChanged(context->Owner, scrollX);
		}
		if (scrollY != lastScrollY)
		{
			context->ScrollYChanged(context->Owner, scrollY);
		}
	}
	Ft_Esd_Widget_IterateChildActiveSlot((Ft_Esd_Widget *)context, FT_ESD_WIDGET_UPDATE);
	bool recalculateB = context->Widget.Recalculate;
	if (recalculateB)
	{
		context->Widget.Recalculate = FT_FALSE;
		ft_int16_t lastScrollX = context->ScrollX;
		ft_int16_t lastScrollY = context->ScrollX;
		Ft_Esd_Layout_Scroll_Recalculate(context);
		ft_int16_t scrollX = context->ScrollX;
		ft_int16_t scrollY = context->ScrollY;
		if (scrollX != lastScrollX)
		{
			context->ScrollXChanged(context->Owner, scrollX);
		}
		if (scrollY != lastScrollY)
		{
			context->ScrollYChanged(context->Owner, scrollY);
		}
	}
	if (recalculateA || recalculateB)
		Ft_Esd_Widget_CheckDefaultIdle(&context->Widget);
}

void Ft_Esd_Layout_Scroll_Render(Ft_Esd_Layout_Scroll *context)
{
	Ft_Esd_Rect16 scissor;
	if (context->Scissor)
	{
		scissor = Ft_Esd_Dl_Scissor_Set(context->Widget.GlobalRect);
	}

	Ft_Esd_Widget_IterateChildActiveValidSlot((Ft_Esd_Widget *)context, FT_ESD_WIDGET_RENDER);

	if (context->Scissor)
	{
		Ft_Esd_Dl_Scissor_Reset(scissor);
	}
}

void Ft_Esd_Layout_Scroll_Idle(Ft_Esd_Layout_Scroll *context)
{
	Ft_Esd_Widget_IterateChildActiveSlot((Ft_Esd_Widget *)context, FT_ESD_WIDGET_IDLE);
}

ft_int16_t Ft_Esd_Layout_Scroll_Width(Ft_Esd_Layout_Scroll *context)
{
	return context->Widget.GlobalWidth;
}

ft_int16_t Ft_Esd_Layout_Scroll_Height(Ft_Esd_Layout_Scroll *context)
{
	return context->Widget.GlobalHeight;
}

ft_uint16_t Ft_Esd_Layout_Scroll_RangeX(Ft_Esd_Layout_Scroll *context)
{
	return context->MaxX - context->MinX;
}

ft_uint16_t Ft_Esd_Layout_Scroll_RangeY(Ft_Esd_Layout_Scroll *context)
{
	return context->MaxY - context->MinY;
}


/* end of file */
