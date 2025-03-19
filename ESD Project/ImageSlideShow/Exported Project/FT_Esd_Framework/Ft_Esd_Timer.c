#include "Ft_Esd.h"
#include "Ft_Esd_Timer.h"

ESD_METHOD(Ft_Esd_Timer_Process, Context = Ft_Esd_Timer)
void Ft_Esd_Timer_Process(Ft_Esd_Timer *context)
{
	while (context->RemainingMs <= 0L)
	{
		context->Fired(context->Owner);
		if (context->Repeat)
		{
			context->RemainingMs += context->TimeoutMs;
		}
		else
		{
			context->RemainingMs = 0L;
			break;
		}
	}
}

/* end of file */
