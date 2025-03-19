#include "Ft_Esd.h"
#include "Ft_Esd_QRCode.h"
#include "Ft_Esd_CoCmd.h"
#include "qrcodegen.h"

extern ESD_CORE_EXPORT Esd_GpuAlloc *Esd_GAlloc;
static uint8_t qrcode[qrcodegen_BUFFER_LEN_MAX];

static void Render_Scale_QRCodeImage(int16_t x, int16_t y, Esd_BitmapCell bitmapCell, esd_int32_f16_t xscale, esd_int32_f16_t yscale, int16_t width, int16_t height)
{
	EVE_HalContext *phost;
	Esd_BitmapInfo *bitmapInfo;
	uint16_t cell;
	uint8_t handle;

	if (!bitmapCell.Info)
		return;

	phost = Ft_Esd_Host;
	bitmapInfo = bitmapCell.Info;
	cell = bitmapCell.Cell;
	handle = Esd_CoDl_SetupBitmap(bitmapInfo);

	Esd_CoDl_BitmapSize(handle, NEAREST, BORDER, BORDER, width, height);

	EVE_CoDl_colorArgb_ex(phost, 0xffffffffUL);
	EVE_CoDl_saveContext(phost);
	EVE_CoCmd_scale(Ft_Esd_Host, xscale, yscale);
	EVE_CoCmd_setMatrix(Ft_Esd_Host);

	EVE_CoDl_begin(phost, BITMAPS);
	Esd_CoDl_Bitmap_Vertex(x, y, handle, cell);

	EVE_CoDl_end(phost);
	EVE_CoCmd_loadIdentity(Ft_Esd_Host);
	EVE_CoDl_restoreContext(phost);
}

static void PrepareQRcode(Ft_Esd_QRCode *context)
{

	char *text = context->URL;
	enum qrcodegen_Ecc errCorLvl = qrcodegen_Ecc_LOW; // Error correction level
	uint8_t tempBuffer[qrcodegen_BUFFER_LEN_MAX];
	bool generateQRCodeOk = qrcodegen_encodeText(text, tempBuffer, qrcode, errCorLvl,
	    qrcodegen_VERSION_MIN, qrcodegen_VERSION_MAX, qrcodegen_Mask_AUTO, true);

	if (generateQRCodeOk)
	{
		unsigned int size = qrcodegen_getSize(qrcode);
		eve_printf_debug("Size of QRCode matrix: %d", size);

		unsigned char pixelsOf8 = 0x0;

		// Set bitmap options
		context->BitmapInfo.Width = size;
		context->BitmapInfo.Height = size;
		context->BitmapInfo.Format = L1;
		context->BitmapInfo.Stride = (size % 8 == 0) ? (size / 8) : (size / 8) + 1;
		context->BitmapInfo.Size = context->BitmapInfo.Stride * size;
		eve_printf_debug("Size of QRCode image: %d", context->BitmapInfo.Size);

		// Drop any previous memory allocation
		// Memory management clears the previous allocation after frame swap (should not clear memory while bitmap is still on display)
		context->BitmapInfo.GpuHandle.Id = MAX_NUM_ALLOCATIONS;
		context->BitmapInfo.GpuHandle.Seq = 0;
		context->BitmapInfo.BitmapHandle = ~0;

		// Set cell
		context->BitmapCell.Info = &context->BitmapInfo;
		context->BitmapCell.Cell = 0; // default: 0, ask Truong

		Ft_Esd_BitmapInfo *bitmapInfo = &context->BitmapInfo;

		if (bitmapInfo->Size <= 0)
			return ~0;

		ft_uint32_t addr = Ft_Esd_GpuAlloc_Get(Ft_Esd_GAlloc, context->BitmapInfo.GpuHandle);
		if (addr == ~0)
		{
			// Allocate new memory if needed
			bitmapInfo->GpuHandle = Ft_Esd_GpuAlloc_Alloc(Ft_Esd_GAlloc, bitmapInfo->Size, GA_GC_FLAG);
			addr = Ft_Esd_GpuAlloc_Get(Ft_Esd_GAlloc, bitmapInfo->GpuHandle);
			if (addr == ~0)
			{
				eve_printf_debug("Cannot allocate memory");
				return;
			}
		}

		// Converse to little endian and add padding
		unsigned int validPixelsOnRow = size / 8;
		unsigned pixelPos = 0;
		for (int y = 0; y < size; y++)
		{
			int countPix = 0;
			for (int x = 0; x < size; x++)
			{
				if (qrcodegen_getModule(qrcode, x, y))
				{
					unsigned char bitPos = x % 8;
					unsigned char mask = 0x80;
					pixelsOf8 = pixelsOf8 | (mask >> bitPos);
				}

				if (countPix != validPixelsOnRow)
				{
					if ((x + 1) % 8 == 0)
					{
						tempBuffer[pixelPos] = pixelsOf8;
						pixelsOf8 = 0;
						pixelPos++;
						countPix++;
					}
				}
				else
				{
					if (x == size - 1)
					{
						tempBuffer[pixelPos] = pixelsOf8;
						pixelsOf8 = 0;
						pixelPos++;
					}
				}
			}
		}
		EVE_Hal_wrMem(Ft_Esd_Host, addr, (uint8_t *)tempBuffer, context->BitmapInfo.Size);
	}
	else
	{
		eve_printf_debug("Cannot generate QRCode\n");
	}
}

ESD_METHOD(Ft_Esd_QRCode_Render_Signal, Context = Ft_Esd_QRCode)
void Ft_Esd_QRCode_Render_Signal(Ft_Esd_QRCode *context)
{

	ft_int32_f16_t xscale_fixed = context->Scale * (1 << 16);
	ft_int32_f16_t yscale_fixed = context->Scale * (1 << 16);

	ft_int16_t width = context->BitmapInfo.Width * context->Scale;
	ft_int16_t height = context->BitmapInfo.Width * context->Scale;

	Render_Scale_QRCodeImage(context->Widget.GlobalX, context->Widget.GlobalY, context->BitmapCell, xscale_fixed, yscale_fixed, width, height);
}

ESD_METHOD(Ft_Esd_QRCode_Update_Signal, Context = Ft_Esd_QRCode)
void Ft_Esd_QRCode_Update_Signal(Ft_Esd_QRCode *context)
{
	if (strcmp(context->LastURL, context->URL) != 0)
	{
		PrepareQRcode(context);
		context->LastURL = context->URL;
	}
}
