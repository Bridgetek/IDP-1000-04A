#include "Ft_Esd.h"
#include "Ft_Esd_Gradient.h"
#include "Ft_Esd_TouchTag.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

ESD_METHOD(GradientManual_PeriodicUpdate, Context = Ft_Esd_Gradient)
ESD_PARAMETER(color1, Type = ft_argb32_t)
ESD_PARAMETER(color2, Type = ft_argb32_t)
void GradientManual_PeriodicUpdate(Ft_Esd_Gradient *context, ft_argb32_t color1, ft_argb32_t color2)
{
	EVE_HalContext *phost = Esd_GetHost();
	void *owner = context->Owner;
	if (!context->Manual(owner))
		return;

	Esd_Rect16 state = Esd_Scissor_Set(context->Widget.GlobalRect);
	EVE_CoCmd_gradient(phost, context->Point_1_X, context->Point_1_Y, color1, context->Point_2_X, context->Point_2_Y, color2);
	Esd_Scissor_Reset(state);

}

ESD_METHOD(Ft_Esd_GradientManual_TouchTracker, Context = Ft_Esd_Gradient)
void Ft_Esd_GradientManual_TouchTracker(Ft_Esd_Gradient *context)
{
}
