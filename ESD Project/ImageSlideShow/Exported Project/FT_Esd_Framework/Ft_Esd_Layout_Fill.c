#include "Ft_Esd_Layout_Fill.h"

static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Fill__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Widget_Update,
	(void (*)(void *))Ft_Esd_Widget_Render,
	(void (*)(void *))Ft_Esd_Widget_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Fill__Initializer(Ft_Esd_Layout_Fill *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Fill_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Fill__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
}


/* end of file */
