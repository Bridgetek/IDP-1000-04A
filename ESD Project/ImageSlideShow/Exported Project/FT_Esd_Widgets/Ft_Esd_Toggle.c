#include "Ft_Esd.h"
#include "Ft_Esd_Toggle.h"

#include "Ft_Esd_Dl.h"
#include "Ft_Esd_Primitives.h"

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;

ft_int16_t Ft_Esd_Toggle_RealHeight(Ft_Esd_Toggle *context)
{
	// FIXME: Support non-rom-font

	void *owner = context->Owner;
	uint8_t i_4 = context->Font(owner);
	uint16_t left_17 = Esd_GetRomFontHeight(i_4);
	int16_t right_16 = 5;
	int16_t left_16 = left_17 * right_16;
	int16_t right_17 = 4;
	return left_16 / right_17;
}

ESD_METHOD(Ft_Esd_Toggle_Render_CoCmd, Context = Ft_Esd_Toggle)
ESD_PARAMETER(tag, Type = ft_uint8_t, Default = 255)
ESD_PARAMETER(x, Type = ft_int16_t, Default = 0) // SCREEN_SIZE
ESD_PARAMETER(y, Type = ft_int16_t, Default = 0) // SCREEN_SIZE
ESD_PARAMETER(width, Type = ft_int16_t, Default = 60) // SCREEN_SIZE
ESD_PARAMETER(font, Type = ft_int16_t, Default = 21, Min = 0, Max = 31) // BITMAP_HANDLE
ESD_PARAMETER(options, Type = Ft_CoPro_Opt, Default = 0) // TODO: Option flags / Bit enum?
ESD_PARAMETER(state, Type = ft_uint16_t, Default = 0)
ESD_PARAMETER(s, Type = const ft_char8_t *, Default = "on\x00off")
void Ft_Esd_Toggle_Render_CoCmd(Ft_Esd_Toggle *context, ft_uint8_t tag, ft_int16_t x, ft_int16_t y, ft_int16_t w, ft_int16_t font, ft_uint16_t options, ft_uint16_t state, const ft_char8_t *s)
{
	// Just use TAG on Toggle itself
	Ft_Esd_Dl_TAG(tag);
	Ft_Gpu_CoCmd_Toggle(Ft_Esd_Host, x, y, w, font, options, state, s);
	Ft_Esd_Dl_TAG(255);
}
