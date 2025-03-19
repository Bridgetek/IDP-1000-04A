
#include "Ft_Esd_Elements.h"
#include "Ft_Esd_Dl.h"
#include "Ft_Esd.h"
#include "Ft_Esd_Widget.h"
#include "Ft_Esd_CircularGradient.h"
#include "Ft_Esd_Primitives.h"

#define PIXELS(x) int((x)*16)
#define BITMAP_WIDTH 512
#define BITMAP_HEIGHT 1
#define BITMAP_TOTAL_SIZE (BITMAP_WIDTH * BITMAP_HEIGHT)

extern ESD_CORE_EXPORT EVE_HalContext *Esd_Host;
extern ESD_CORE_EXPORT Esd_GpuAlloc *Esd_GAlloc;

static Ft_Esd_GpuHandle pythagGpuHandle;
static Ft_Esd_GpuHandle gaussGpuHandle;

/* TODO: Use deflate compressed format for embedding */

// clang-format off
static eve_progmem_const uint8_t pythag_raw[] = {
	/*('file properties: ', 'resolution ', 256, 'x', 1, 'format ', 'L8', 'stride ', 512, ' total size ', 256) (Symmetric data to be generated)*/
	0,0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,21,22,23,24,25,26,27,28,29,
	29,30,31,32,33,34,35,35,36,37,38,39,40,40,41,42,43,44,45,45,46,47,48,48,49,50,51,52,52,53,54,55,
	55,56,57,58,58,59,60,61,61,62,63,63,64,65,65,66,67,68,68,69,70,70,71,72,72,73,74,74,75,75,76,77,
	77,78,79,79,80,80,81,82,82,83,83,84,85,85,86,86,87,87,88,88,89,90,90,91,91,92,92,93,93,94,94,95,
	95,96,96,97,97,98,98,99,99,100,100,100,101,101,102,102,103,103,104,104,104,105,105,106,106,106,107,107,108,108,108,109,
	109,110,110,110,111,111,111,112,112,112,113,113,113,114,114,114,115,115,115,116,116,116,116,117,117,117,118,118,118,118,119,119,
	119,119,120,120,120,120,121,121,121,121,121,122,122,122,122,122,123,123,123,123,123,123,124,124,124,124,124,124,125,125,125,125,
	125,125,125,125,126,126,126,126,126,126,126,126,126,126,126,126,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127,127
};

static eve_progmem_const uint8_t gauss_raw[] = {
	/*('file properties: ', 'resolution ', 256, 'x', 1, 'format ', 'L8', 'stride ', 512, ' total size ', 256) (Symmetric data to be generated)*/
	93,94,95,96,96,97,98,99,99,100,101,102,102,103,104,105,105,106,107,108,109,109,110,111,112,113,113,114,115,116,117,117,
	118,119,120,121,121,122,123,124,125,126,126,127,128,129,130,131,131,132,133,134,135,136,136,137,138,139,140,141,141,142,143,144,
	145,146,147,147,148,149,150,151,152,153,153,154,155,156,157,158,159,159,160,161,162,163,164,165,165,166,167,168,169,170,171,171,
	172,173,174,175,176,176,177,178,179,180,181,181,182,183,184,185,186,186,187,188,189,190,190,191,192,193,194,194,195,196,197,198,
	198,199,200,201,201,202,203,204,204,205,206,207,207,208,209,210,210,211,212,212,213,214,215,215,216,217,217,218,219,219,220,221,
	221,222,223,223,224,224,225,226,226,227,227,228,229,229,230,230,231,232,232,233,233,234,234,235,235,236,236,237,237,238,238,239,
	239,240,240,241,241,241,242,242,243,243,244,244,244,245,245,245,246,246,247,247,247,248,248,248,248,249,249,249,250,250,250,250,
	251,251,251,251,252,252,252,252,252,253,253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,254,254,254,254,254,254,255
};
// clang-format on

static void CircularGradient_BitmapTransform(int dir, int w, int h)
{
	EVE_HalContext *phost = Esd_GetHost();

	if (dir == 0)
	{
		EVE_CoCmd_bitmapTransform(phost, 0, 0, w, 0, 0, h, 0, 0, 511, 0, 0, 511, 0);
	}
	else
	{
		EVE_CoCmd_bitmapTransform(phost, 0, 0, w, 0, 0, h, 0, 0, 0, 511, 511, 0, 0);
	}
}

void writeBitmapIntoGPUMemory(uint8_t gradientType)
{
	EVE_HalContext *phost = Esd_GetHost();

	if (gradientType == ESD_PYTHAGOREAN)
	{
		uint8_t pythag_raw_symmetric[sizeof(pythag_raw) << 1];
		uint32_t size = sizeof(pythag_raw), count = size;
		while (count)
		{
			pythag_raw_symmetric[size + count - 1] = pythag_raw_symmetric[size - count] = pythag_raw[size - count];
			count--;
		}

		pythagGpuHandle = Ft_Esd_GpuAlloc_Alloc(Ft_Esd_GAlloc, BITMAP_TOTAL_SIZE, GA_GC_FLAG);
		uint32_t addr = Ft_Esd_GpuAlloc_Get(Ft_Esd_GAlloc, pythagGpuHandle);

		EVE_Cmd_startFunc(phost);
		EVE_Cmd_wr32(phost, CMD_MEMWRITE);
		EVE_Cmd_wr32(phost, addr);
		EVE_Cmd_wr32(phost, sizeof(pythag_raw_symmetric));
		EVE_Cmd_wrMem(phost, pythag_raw_symmetric, sizeof(pythag_raw_symmetric));
		EVE_Cmd_endFunc(phost);
	}
	else
	{
		uint8_t gauss_raw_symmetric[sizeof(gauss_raw) << 1];
		uint32_t size = sizeof(gauss_raw), count = size;
		while (count)
		{
			gauss_raw_symmetric[size + count - 1] = gauss_raw_symmetric[size - count] = gauss_raw[size - count];
			count--;
		}

		gaussGpuHandle = Ft_Esd_GpuAlloc_Alloc(Ft_Esd_GAlloc, BITMAP_TOTAL_SIZE, GA_GC_FLAG);
		uint32_t addr = Ft_Esd_GpuAlloc_Get(Ft_Esd_GAlloc, gaussGpuHandle);

		EVE_Cmd_startFunc(phost);
		EVE_Cmd_wr32(phost, CMD_MEMWRITE);
		EVE_Cmd_wr32(phost, addr);
		EVE_Cmd_wr32(phost, sizeof(gauss_raw_symmetric));
		EVE_Cmd_wrMem(phost, gauss_raw_symmetric, sizeof(gauss_raw_symmetric));
		EVE_Cmd_endFunc(phost);
	}
}

ESD_METHOD(Circular_Gradient_Widget_Render_Circular, Context = Ft_Esd_CircularGradient)
ESD_PARAMETER(x0, Type = int32_t, Default = 0)
ESD_PARAMETER(y0, Type = int32_t, Default = 0)
ESD_PARAMETER(x1, Type = int32_t, Default = 60)
ESD_PARAMETER(y1, Type = int32_t, Default = 20)
ESD_PARAMETER(color1, Type = ft_argb32_t, DisplayName = "outer color", Default = #FF3F3F3F)
ESD_PARAMETER(color2, Type = ft_argb32_t, DisplayName = "inner color", Default = #FFFFFFFF)
ESD_PARAMETER(gradientType, Type = Esd_CircularGradientType, DisplayName = "gradient type", Default = ESD_PYTHAGOREAN)
ft_void_t Circular_Gradient_Widget_Render_Circular(Ft_Esd_CircularGradient *context,
    int32_t x0, int32_t y0,
    int32_t x1, int32_t y1,
    ft_argb32_t color1, ft_argb32_t color2,
    uint8_t gradientType)
{
	EVE_HalContext *phost = Esd_GetHost();

	uint8_t handle = ESD_CO_SCRATCH_HANDLE;
	int32_t w = x1 - x0;
	int32_t h = y1 - y0;
	int32_t X0 = x0 * 8;
	int32_t X1 = x1 * 8;
	int32_t Y0 = y0 * 8;
	int32_t Y1 = y1 * 8;

	EVE_CoDl_bitmapHandle(phost, handle);

	if (gradientType == ESD_PYTHAGOREAN)
	{
		uint32_t addr = Esd_GpuAlloc_Get(Esd_GAlloc, pythagGpuHandle);
		if (addr == GA_INVALID)
			writeBitmapIntoGPUMemory(gradientType);
		if (EVE_CHIPID >= EVE_FT810)
		{
			EVE_CoCmd_setBitmap(phost, addr, L8, BITMAP_WIDTH, BITMAP_HEIGHT); // FIXME: Implement an FT800 software fallback for CMD_SETBITMAP
		}
	}
	else
	{
		uint32_t addr = Esd_GpuAlloc_Get(Esd_GAlloc, gaussGpuHandle);
		if (addr == GA_INVALID)
			writeBitmapIntoGPUMemory(gradientType);
		if (EVE_CHIPID >= EVE_FT810)
		{
			EVE_CoCmd_setBitmap(phost, addr, L8, BITMAP_WIDTH, BITMAP_HEIGHT);
		}
	}
	EVE_CoDl_bitmapSize(phost, BILINEAR, REPEAT, REPEAT, w, h);

	// The code below will do the circular gradient
	EVE_CoDl_saveContext(phost);
	EVE_CoDl_begin(phost, BITMAPS);

	Ft_Esd_Rect16 scissor = {
		.X = x0, .Y = y0, .Width = w, .Height = h
	};
	Esd_Scissor_Set(scissor); // No need to restore scissor afterwards, since context is saved

	EVE_CoDl_colorMask(phost, false, false, false, true);

	EVE_CoDl_vertexFormat(phost, 3);
	EVE_CoDl_bitmapHandle(phost, handle); // At any instant only one bitmap is loaded in memory.
	// Hence we use only one Bitmap Handle. By default it is starting with zero (0)

	EVE_CoDl_blendFunc(phost, ONE, ZERO); // Draw the X bitmap into alpha
	CircularGradient_BitmapTransform(0, w, h);
	EVE_CoDl_vertex2f(phost, X0, Y0);

	if (gradientType == ESD_PYTHAGOREAN)
		EVE_CoDl_blendFunc(phost, ONE, ONE); // For pythagorean, want alpha addition
	else
		EVE_CoDl_blendFunc(phost, DST_ALPHA, ZERO); // But for Gaussian, multiplication
	CircularGradient_BitmapTransform(1, w, h); // Blend the Y bitmap into alpha
	EVE_CoDl_vertex2f(phost, X0, Y0);

	EVE_CoDl_colorMask(phost, true, true, true, false); // Now use the alpha buffer
	EVE_CoDl_begin(phost, RECTS);

	EVE_CoDl_colorRgb_ex(phost, color1);
	EVE_CoDl_blendFunc(phost, ONE, ZERO);
	EVE_CoDl_vertex2f(phost, X0, Y0);
	EVE_CoDl_vertex2f(phost, X1, Y1);

	EVE_CoDl_colorRgb_ex(phost, color2);
	EVE_CoDl_blendFunc(phost, DST_ALPHA, ONE_MINUS_DST_ALPHA);
	EVE_CoDl_vertex2f(phost, X0, Y0);
	EVE_CoDl_vertex2f(phost, X1, Y1);
	EVE_CoDl_restoreContext(phost);
}
