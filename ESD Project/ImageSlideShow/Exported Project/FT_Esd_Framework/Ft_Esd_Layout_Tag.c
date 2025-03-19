

#include "Ft_Esd_Layout_Tag.h"
#include "Ft_Esd_Dl.h"

ft_uint8_t Ft_Esd_Layout_Tag_Tag__Default(void *context) { return 255; }

static Ft_Esd_WidgetSlots s_Ft_Esd_Layout_Tag__Slots = {
	(void (*)(void *))Ft_Esd_Widget_Initialize,
	(void (*)(void *))Ft_Esd_Widget_Start,
	(void (*)(void *))Ft_Esd_Widget_Enable,
	(void (*)(void *))Ft_Esd_Widget_Update,
	(void (*)(void *))Ft_Esd_Layout_Tag_Render,
	(void (*)(void *))Ft_Esd_Widget_Idle,
	(void (*)(void *))Ft_Esd_Widget_Disable,
	(void (*)(void *))Ft_Esd_Widget_End,
};

void Ft_Esd_Layout_Tag__Initializer(Ft_Esd_Layout_Tag *context)
{
	Ft_Esd_Widget__Initializer((Ft_Esd_Widget *)context);
	context->Widget.ClassId = Ft_Esd_Layout_Tag_CLASSID;
	context->Widget.Slots = &s_Ft_Esd_Layout_Tag__Slots;
	context->Widget.LocalWidth = 50;
	context->Widget.LocalHeight = 50;
	context->Tag = Ft_Esd_Layout_Tag_Tag__Default;
}

void Ft_Esd_Layout_Tag_Render(Ft_Esd_Layout_Tag *context)
{
	void *owner = context->Owner;
	Ft_Esd_Dl_TAG(context->Tag(owner));
	Ft_Esd_Widget_Render((Ft_Esd_Widget *)context);
	Ft_Esd_Dl_TAG(255);
}


/* end of file */
