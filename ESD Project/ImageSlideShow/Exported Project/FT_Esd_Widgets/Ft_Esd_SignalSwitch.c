#include "Ft_Esd.h"
#include "Ft_Esd_SignalSwitch.h"

enum Esd_Switch_State_Enum
{
	_IDLE = 0,
	_Changing = 1,
	_ToPrevious = 2,
	_ToNext = 4,
	_Recovering = 8,
	_Shifting = 16,
	_Loading = 32,
	_All = 255
};

ESD_METHOD(Ft_Esd_SignalSwitch_Initialise_Signal, Context = Ft_Esd_SignalSwitch)
void Ft_Esd_SignalSwitch_Initialise_Signal(Ft_Esd_SignalSwitch *context);

ESD_METHOD(Esd_Switch_NextPage, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_NextPage(Ft_Esd_SignalSwitch *context);

ESD_METHOD(Esd_Switch_PreviousPage, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_PreviousPage(Ft_Esd_SignalSwitch *context);

ESD_METHOD(Esd_Switch_Update_Handle, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_Update_Handle(Ft_Esd_SignalSwitch *context);
// this function is different from the similar one in scroll swith, it will be only be called one time during initialazation, in order to jump to the page defined by initial index
ESD_METHOD(Esd_Switch_Recover_Position, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_Recover_Position(Ft_Esd_SignalSwitch *context);
#if 0
ESD_METHOD(Esd_Switch_Recover_Position, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_Recover_Position(Ft_Esd_SignalSwitch *context);
#endif

ESD_METHOD(Esd_Switch_Shift_Position, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_Shift_Position(Ft_Esd_SignalSwitch *context);

ESD_METHOD(Esd_Switch_Update_Widget, Context = Ft_Esd_SignalSwitch)
ESD_PARAMETER(newWidget, DisplayName = "Widget", Type = Ft_Esd_Widget *)
void Esd_Switch_Update_Widget(Ft_Esd_SignalSwitch *context, Ft_Esd_Widget *newWidget);

ESD_METHOD(Esd_Switch_PreUpdate_Widget, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_PreUpdate_Widget(Ft_Esd_SignalSwitch *context);

ESD_METHOD(Esd_Switch_Render_Signal, Context = Ft_Esd_SignalSwitch)
void Esd_Switch_Render_Signal(Ft_Esd_SignalSwitch *context);

void Esd_Switch_PreUpdate_Widget(Ft_Esd_SignalSwitch *context)
{
	//??
	// if (!(context->State & _Changing))
	// return;
	eve_printf_debug("Esd_Switch_PreUpdate_Widget is called\n");
	context->State &= ~_Loading;

	if ((context->State & _ToNext) && (context->Next != NULL))
	{
		context->Previous = context->Current;
		context->Current = context->Next;
		context->Next = NULL;
		context->State |= _Loading;
	}
	else if ((context->State & _ToPrevious) && (context->Previous != NULL))
	{
		context->Next = context->Current;
		context->Current = context->Previous;
		context->Previous = NULL;
		context->State |= _Loading;
	}

	context->Widget.Recalculate = FT_TRUE;
	// context->ScrollXStart = context->ScrollX;
	// context->State |= _Shifting;
}

void Esd_SignalSwitch_Load_NewPage(Ft_Esd_SignalSwitch *context)
{
	eve_printf_debug("Esd_SignalSwitch_Load_NewPage is called\n");
	if (context->State & _Loading)
	{
		void *owner = context->Owner;
		// printf("Start Loading %d\n", context->State);
		context->NewPage(owner);
		Ft_Esd_Widget *newWidget = context->New(owner);
		Esd_Switch_Update_Widget(context, newWidget);
		context->State &= ~_Loading;
		// printf("Done Loading %d\n", context->State);
	}
}

void Esd_Switch_Update_Widget(Ft_Esd_SignalSwitch *context, Ft_Esd_Widget *newWidget)
{
	eve_printf_debug("Esd_Switch_Update_Widget is called\n");
	if (!(context->State & _Changing))
	{
		eve_printf_debug("state is not _Changing, is %d\n", (int)context->State);
		return;
	}
	if (!(context->State & _Loading) || (context->State & _Shifting))
	{
		eve_printf_debug("state is not _Loading or _Shifting, is %d\n", (int)context->State);
		return;
	}
	if (context->State & _ToNext)
	{
		context->Next = context->New(context->Owner);
		if (context->Next && context->Next != context->Current)
		{

			context->Next->GlobalValid = FT_FALSE;
			if (context->Direction == ESD_HORIZONTAL)
			{
				context->Next->LocalX = Esd_Switch_GetStart(context->Index + 1, context);
				context->Next->LocalY = 0;
			}
			else if (context->Direction == ESD_VERTICAL)
			{
				context->Next->LocalY = Esd_Switch_GetStart(context->Index + 1, context);
				context->Next->LocalX = 0;
			}
			else
			{
				eve_printf("direction error %d\n", (int)context->Direction);
			}
		}
	}
	else if (context->State & _ToPrevious)
	{
		context->Previous = context->New(context->Owner);
		if (context->Previous && context->Previous != context->Current)
		{
			context->Previous->GlobalValid = FT_FALSE;
			if (context->Direction == ESD_HORIZONTAL)
			{
				context->Previous->LocalX = Esd_Switch_GetStart(context->Index - 1, context);
				context->Previous->LocalY = 0;
			}
			else if (context->Direction == ESD_VERTICAL)
			{
				context->Previous->LocalY = Esd_Switch_GetStart(context->Index - 1, context);
				context->Previous->LocalX = 0;
			}
			else
			{
				eve_printf("direction error %d\n", (int)context->Direction);
			}
		}
	}
	context->Widget.Recalculate = FT_TRUE;
	context->State &= ~(_Changing | _Loading);
}
void Esd_Switch_NextPage(Ft_Esd_SignalSwitch *context)
{
	// button click no effect if still changing
	if (context->State & _Shifting)
	{
		printf("still in _Shifting state, quit\n");
		return;
	}
	if (context->State & _ToNext)
	{
		printf("still in _ToNext state, quit\n");
		return;
	}
	if (context->State & _ToPrevious)
	{
		printf("still in _ToPrevious state, quit\n");
		return;
	}

	int index = context->Index;
	eve_printf_debug("In next page, init index is %d\n", index);
	Ft_Esd_Widget *removingWidget = NULL;
	// clear flags
	context->State &= ~(_ToNext | _ToPrevious);

	if (context->Index < context->Count)
	{
		index = Ft_Esd_Int16_ClampedValue((context->Index % context->Count) + 1, 0, context->Count);
		// click next button, change to right for sure, and set the changing flag
		context->State |= _ToNext | _Changing;
		if (index == context->Count && !context->Looping)
			index--;
	}
	if (index == context->Index)
	{
		eve_printf_debug("In next page, index is equal to context index %d\n", index);
		context->State |= _Recovering;
		context->Shifting = true;
	}
	else
	{

		// here confirm move to right
		if ((context->State & _ToNext) && context->Previous != NULL)
			removingWidget = context->Previous;

		if (context->Current->Previous == removingWidget)
			context->Current->Previous = NULL;
		else if (context->Current->Next == removingWidget)
			context->Current->Next = NULL;

		if (removingWidget == context->Current)
			removingWidget = NULL;

		if (removingWidget != NULL)
		{
			// if (removingWidget->Instanced == 1) // To be revisited: not able to check if it is lazy allocated
			{
				Ft_Esd_Widget_Detach(removingWidget);
				Ft_Esd_Widget_SetActive(removingWidget, FT_FALSE);
				Ft_Esd_Widget_Disable(removingWidget);
				// printf("flags: %x, %d\n", removingWidget->Flags, removingWidget->Instanced);
			}
			// context->RemovingWidget = removingWidget;
		}
		context->NewIndex = (context->Index - (context->Index - index) * 2);
		// context->NewIndex  = index;
		eve_printf_debug("Next page NexIndex is %d\n", (int)context->NewIndex);
		if (context->Looping)
		{
			if (index == context->Count)
			{
				index = 0;
				context->NewIndex = index + 1;
				if (context->Direction == ESD_HORIZONTAL)
				{
					context->Current->LocalX = Esd_Switch_GetStart(-1, context);
					context->Next->LocalX = Esd_Switch_GetStart(0, context);
					context->ScrollPanel.ScrollX = context->Current->LocalX;
					context->UpdatePos = context->Current->LocalX;
					eve_printf_debug("Looping index = count %d\n", (int)context->Count);
				}
				else if (context->Direction == ESD_VERTICAL)
				{
					context->Current->LocalY = Esd_Switch_GetStart(-1, context);
					context->Next->LocalY = Esd_Switch_GetStart(0, context);
					context->ScrollPanel.ScrollY = context->Current->LocalY;
					context->UpdatePos = context->Current->LocalY;
					eve_printf_debug("Looping index = count %d\n", (int)context->Count);
				}
				else
				{
					eve_printf("direction error %d\n", (int)context->Direction);
				}
			}
		}

		if (context->Looping)
		{
			if (context->NewIndex < 0)
				context->NewIndex += context->Count;
			if (context->NewIndex >= context->Count)
				context->NewIndex %= context->Count;
		}
		else
		{
			if (context->NewIndex < 0)
				context->NewIndex = index;
			if (context->NewIndex >= context->Count)
				context->NewIndex = index;
		}
		eve_printf_debug("Next page Index %d -> %d (new %d), ScrollTarget %d\n", context->Index, index, context->NewIndex, Esd_Switch_GetStart(index, context));
		context->Index = index;

		//??
		context->Target = Esd_Switch_GetStart(context->Index, context);
		eve_printf_debug("Next page After touch up, Target is %d\n", (int)context->Target);
		context->State |= _Shifting;
		context->Shifting = true;
	}
	// Esd_Switch_PreUpdate_Widget(context);
}

void Esd_Switch_PreviousPage(Ft_Esd_SignalSwitch *context)
{
	if (context->State & _Shifting)
	{
		printf("still in _Shifting state, quit\n");
		return;
	}
	if (context->State & _ToNext)
	{
		printf("still in _ToNext state, quit\n");
		return;
	}
	if (context->State & _ToPrevious)
	{
		printf("still in _ToPrevious state, quit\n");
		return;
	}
	int index = context->Index;
	Ft_Esd_Widget *removingWidget = NULL;
	eve_printf_debug("In previous page, init index is %d\n", index);
	context->State &= ~(_ToNext | _ToPrevious);

	if (context->Index < context->Count)
	{
		// index = Ft_Esd_Int16_ClampedValue((context->Index % context->Count) + 1, 0, context->Count);
		index = Ft_Esd_Int16_ClampedValue((context->Index % context->Count) - 1, -1, context->Count - 1);
		context->State |= _ToPrevious | _Changing;
		if (index == -1 && !context->Looping)
			index++;
	}

	if (index == context->Index)
	{
		eve_printf_debug("In previous page, index is equal to context index %d\n", index);
		context->State |= _Recovering;
	}
	else
	{
		// here confirm move to left
		if ((context->State & _ToPrevious) && context->Previous != NULL)
			removingWidget = context->Next;

		if (context->Current->Previous == removingWidget)
			context->Current->Previous = NULL;
		else if (context->Current->Next == removingWidget)
			context->Current->Next = NULL;

		if (removingWidget == context->Current)
			removingWidget = NULL;

		if (removingWidget != NULL)
		{
			// if (removingWidget->Instanced == 1) // To be revisited: not able to check if it is lazy allocated
			{
				Ft_Esd_Widget_Detach(removingWidget);
				Ft_Esd_Widget_SetActive(removingWidget, FT_FALSE);
				Ft_Esd_Widget_Disable(removingWidget);
				// printf("flags: %x, %d\n", removingWidget->Flags, removingWidget->Instanced);
			}
			// context->RemovingWidget = removingWidget;
		}
		context->NewIndex = (context->Index - (context->Index - index) * 2);
		// context->NewIndex  = index;
		eve_printf_debug("Previous page before new index is %d\n", context->NewIndex);
		if (context->Looping)
		{
			if (index == -1)
			{
				index = context->Count - 1;
				context->NewIndex = context->Count - 2;
				eve_assert(context->Current);

				if (context->Direction == ESD_HORIZONTAL)
				{
					context->Current->LocalX = Esd_Switch_GetStart(context->Count, context);
					eve_assert(context->Previous);
					context->Previous->LocalX = Esd_Switch_GetStart(context->Count - 1, context);

					context->ScrollPanel.ScrollX = context->Current->LocalX;
					context->UpdatePos = context->Current->LocalX;
					eve_printf_debug("Looping index = count %d\n", (int)context->Count);
				}
				else if (context->Direction == ESD_VERTICAL)
				{
					context->Current->LocalY = Esd_Switch_GetStart(context->Count, context);
					eve_assert(context->Previous);
					context->Previous->LocalY = Esd_Switch_GetStart(context->Count - 1, context);

					context->ScrollPanel.ScrollY = context->Current->LocalY;
					context->UpdatePos = context->Current->LocalY;
					eve_printf_debug("Looping index = count %d\n", (int)context->Count);
				}
				else
				{
					eve_printf("direction wrong %d\n", (int)context->Direction);
				}
			}
		}
		if (context->Looping)
		{
			if (context->NewIndex < 0)
				context->NewIndex += context->Count;
			if (context->NewIndex >= context->Count)
				context->NewIndex %= context->Count;
		}
		else
		{
			if (context->NewIndex < 0)
				context->NewIndex = index;
			if (context->NewIndex >= context->Count)
				context->NewIndex = index;
			eve_printf_debug("Previous page Index %d -> %d (new %d), ScrollXTarget %d\n", context->Index, index, context->NewIndex, Esd_Switch_GetStart(index, context));
		}
		//??
		context->Index = index;
		context->Target = Esd_Switch_GetStart(context->Index, context);
		eve_printf_debug("Previous After touch up, Target is %d\n", (int)context->Target);
		// Esd_Switch_PreUpdate_Widget(context);
		context->State |= _Shifting;
		context->Shifting = true;
	}
}

int Esd_Switch_GetUnit(Ft_Esd_SignalSwitch *context)
{
	if (context->Direction == ESD_HORIZONTAL)
	{
		return context->Widget.LocalWidth;
	}
	else if (context->Direction == ESD_VERTICAL)
	{
		return context->Widget.LocalHeight;
	}
	else
	{
		eve_printf("directon wrong %d\n", (int)context->Direction);
		return -1;
	}
}

int Esd_Switch_GetStart(int index, Ft_Esd_SignalSwitch *context)
{
	// int unit = Esd_Switch_GetUnit(context);
	if (context->Looping)
	{
		index = Ft_Esd_Int16_ClampedValue(index, -1, context->Count + 1);
		if (context->Direction == ESD_HORIZONTAL)
		{
			return (index + 1) * context->Widget.LocalWidth;
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			return (index + 1) * context->Widget.LocalHeight;
		}
		else
		{
			eve_printf("direction wrong %d\n", (int)context->Direction);
			return -1;
		}
	}
	else
	{
		index = Ft_Esd_Int16_ClampedValue(index, 0, context->Count - 1);
		// return context->Spacing + index * unit;
		if (context->Direction == ESD_HORIZONTAL)
		{
			return index * context->Widget.LocalWidth;
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			return index * context->Widget.LocalHeight;
		}
		else
		{
			eve_printf("direction wrong %d\n", (int)context->Direction);
			return -1;
		}
	}
}
int Esd_Switch_GetSpeed(int dx, int scale)
{
	scale = Ft_Esd_Int16_ClampedValue(scale, 1, 16);
	int max = ((abs(dx / 20) + 1) << 1) * scale;
	int step = (dx > 0) ? min(max, dx) : max(-max, dx);
	return step;
}

void Esd_Switch_Recover_Position(Ft_Esd_SignalSwitch *context)
{

	if (!(context->State & _Recovering))
	{
		printf("not in recovery state\n");
		return;
	}
	if (!context->Current)
	{
		printf("not current page\n");
		return;
	}
	eve_printf_debug("In recovery, start scroll panel x is %d\n", context->ScrollPanel.ScrollX);
	int newTarget = Esd_Switch_GetStart(context->Index, context);

	int delta = (newTarget - context->UpdatePos);
	int step = Esd_Switch_GetSpeed(delta, context->SwitchSpeed);
	eve_printf_debug("In recovery index is %d, target is %d, pos is %d\n", context->Index, newTarget, context->UpdatePos);
	eve_printf_debug("In recovery delta is %d, step is %d\n", delta, step);
	// printf("In recovery scroll panel x is %d\n",context->ScrollPanel.ScrollX);

	if (delta == step)
	{
		context->UpdatePos = newTarget;
		context->State &= ~(_Changing | _Recovering);
		context->Shifting = false;
	}
	else
	{
		context->UpdatePos = context->UpdatePos + step;
	}
	// else
	//{
	// context->X = context->X + step;
	// }
	if (context->Direction == ESD_HORIZONTAL)
	{
		context->ScrollPanel.ScrollX = context->UpdatePos;
		eve_printf_debug("After recovery, scroll panel x is %d\n", context->ScrollPanel.ScrollX);
	}
	else if (context->Direction == ESD_VERTICAL)
	{
		context->ScrollPanel.ScrollY = context->UpdatePos;
		eve_printf_debug("After recovery, scroll panel y is %d\n", context->ScrollPanel.ScrollY);
	}
	else
	{
		eve_printf("wrong direction %d\n", (int)context->Direction);
	}
}

void Esd_Switch_Shift_Position(Ft_Esd_SignalSwitch *context)
{

	if (!(context->State & _Shifting))
	{
		printf("Ft_Esd_ScrollSwitch_Shift_Position skip %d\n", context->State);
		return;
	}
	// leave overflow as no action now
	eve_printf_debug("In shift target: %d, pos: %d\n", context->Target, context->UpdatePos);
	int newTarget = context->Target;

	int delta = newTarget - context->UpdatePos;
	eve_printf_debug("In shift delta is %d, get Unit is %d\n", delta, (int)Esd_Switch_GetUnit(context));

	// printf("dx is %d, step is %d\n",dx,step);
	if (abs(delta) > Esd_Switch_GetUnit(context))
	{
		printf("Potentical Error: abs delta is %d, get unit is %d\n", abs(delta), (int)Esd_Switch_GetUnit(context));
	}
	int step = Esd_Switch_GetSpeed(delta, context->SwitchSpeed);
	eve_printf_debug("Ft_Esd_ScrollSwitch_Shift_Position %d + %d -> %d\n", context->UpdatePos, step, context->UpdatePos + step);
	if (delta == step)
	{
		context->UpdatePos = newTarget;
		context->State &= ~_Shifting;
		context->Shifting = false;
	}
	else
	{
		context->UpdatePos = context->UpdatePos + step;

		// update child widget position
		// Todo
	}
	if (context->Direction == ESD_HORIZONTAL)
	{
		context->ScrollPanel.ScrollX = context->UpdatePos;
		eve_printf_debug("After recovery, scroll panel x is %d\n", context->ScrollPanel.ScrollX);
	}
	else if (context->Direction == ESD_VERTICAL)
	{
		context->ScrollPanel.ScrollY = context->UpdatePos;
		eve_printf_debug("After recovery, scroll panel y is %d\n", context->ScrollPanel.ScrollY);
	}
	else
	{
		eve_printf("wrong direction %d\n", (int)context->Direction);
	}
}

void Esd_Switch_Update_Handle(Ft_Esd_SignalSwitch *context)
{
	void *owner = context->Owner;

	int state = context->State;

	ft_bool_t isChanging = state & _Changing;
	ft_bool_t isRecovering = state & _Recovering;
	ft_bool_t isShifting = state & _Shifting;
	context->Shifting = false;
	// context->ScrollPanel.TouchScroll = FT_FALSE;
	// printf("current state is %d\n",(int)state);
	if (isRecovering)
	{
		context->ScrollPanel.TouchScroll = FT_FALSE;
		Esd_Switch_Recover_Position(context);
	}
	else if (isShifting)
	{
		Esd_Switch_Shift_Position(context);
		int newState = context->State;
		ft_bool_t newIsShifting = newState & _Shifting;
		context->Shifting = false;
		if (!newIsShifting)
		{
			Esd_Switch_PreUpdate_Widget(context);
			// printf("Finished Shifting %d\n", context->State );
			Esd_SignalSwitch_Load_NewPage(context);
		}
	}
	else
	{
		context->State &= ~(_ToPrevious | _ToNext);
	}
	// context->Widget.Recalculate = FT_TRUE;
}
void Ft_Esd_SignalSwitch_Initialise_Signal(Ft_Esd_SignalSwitch *context)
{
	// ...
	int unit = Esd_Switch_GetUnit(context);
	// context->FixedPositioning.Widget.LocalWidth = (unit) * (context->Count) + context->Spacing;
	if (context->Looping)
	{
		if (context->Direction == ESD_HORIZONTAL)
		{
			context->FixedPositioning.Widget.LocalWidth = (unit) * (context->Count + 2);
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			context->FixedPositioning.Widget.LocalHeight = (unit) * (context->Count + 2);
		}
		else
		{
			eve_printf("wrong direction %d\n", (int)context->Direction);
		}
	}
	else
	{
		if (context->Direction == ESD_HORIZONTAL)
		{
			context->FixedPositioning.Widget.LocalWidth = (unit) * (context->Count);
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			context->FixedPositioning.Widget.LocalWidth = (unit) * (context->Count);
		}
		else
		{
			eve_printf("wrong direction %d\n", (int)context->Direction);
		}
	}
	context->Previous = context->InitialPrevious(context->Owner);
	context->Current = context->InitialCurrent(context->Owner);
	context->Next = context->InitialNext(context->Owner);
	context->Index = context->InitialIndex(context->Owner);
	if (context->Direction == ESD_HORIZONTAL)
	{
		context->Previous->LocalY = 0;
		context->Current->LocalY = 0;
		context->Next->LocalY = 0;
		// LocalX is the length from the most left side to the target page's left side
		context->Previous->LocalX = Esd_Switch_GetStart(context->Index - 1, context);
		context->Current->LocalX = Esd_Switch_GetStart(context->Index, context);
		context->Next->LocalX = Esd_Switch_GetStart(context->Index + 1, context);
		eve_printf_debug("previous local x %d, current %d, next %d, index %d\n", context->Previous->LocalX, context->Current->LocalX, context->Next->LocalX, context->Index);
		// XTarget is the next postion of current page. It should be the value of width x index
		context->Target = context->Current->LocalX;
	}
	else if (context->Direction == ESD_VERTICAL)
	{
		context->Previous->LocalX = 0;
		context->Current->LocalX = 0;
		context->Next->LocalX = 0;
		// LocalX is the length from the most left side to the target page's left side
		context->Previous->LocalY = Esd_Switch_GetStart(context->Index - 1, context);
		context->Current->LocalY = Esd_Switch_GetStart(context->Index, context);
		context->Next->LocalY = Esd_Switch_GetStart(context->Index + 1, context);
		eve_printf_debug("previous local y %d, current %d, next %d, index %d\n", context->Previous->LocalY, context->Current->LocalY, context->Next->LocalY, context->Index);
		// Target is the next postion of current page. It should be the value of height y index
		context->Target = context->Current->LocalY;
	}
	else
	{
		eve_printf("wrong direction %d\n", (int)context->Direction);
	}

	eve_printf_debug("Target is %d\n", context->Target);
	context->State = _Changing | _Recovering;
	context->Shifting = true;
	eve_printf_debug("during init, current state is %d\n", (int)context->State);
}

void Esd_Switch_Render_Signal(Ft_Esd_SignalSwitch *context)
{
	static bool firstStart = true;

	int minX = context->Widget.GlobalX;
	int maxX = context->Widget.GlobalX + context->Widget.GlobalWidth;
	int minY = context->Widget.GlobalY;
	int maxY = context->Widget.GlobalY + context->Widget.GlobalHeight;

	if (context->Previous != NULL)
	{
		if (context->Direction == ESD_HORIZONTAL)
		{
			if (context->Previous->GlobalX + context->Widget.GlobalWidth < minX)
			{
				context->Previous->GlobalValid = FT_FALSE;
			}
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			if (context->Previous->GlobalY + context->Widget.GlobalHeight < minY)
			{
				context->Previous->GlobalValid = FT_FALSE;
			}
		}
		else
		{
			eve_printf("wrong direction %d\n", (int)context->Direction);
		}

		if (firstStart == true)
		{
			context->Previous->GlobalValid = FT_FALSE;
			firstStart = false;
		}
	}

	if (context->Next != NULL)
	{
		if (context->Direction == ESD_HORIZONTAL)
		{
			if (context->Next->GlobalX > maxX)
			{
				context->Next->GlobalValid = FT_FALSE;
			}
		}
		else if (context->Direction == ESD_VERTICAL)
		{
			if (context->Next->GlobalY > maxY)
			{
				context->Next->GlobalValid = FT_FALSE;
			}
		}
		else
		{
			eve_printf("wrong direction %d\n", (int)context->Direction);
		}
	}
}
