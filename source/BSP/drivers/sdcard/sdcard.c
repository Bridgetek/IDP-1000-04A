#include "sdcard.h"

#include <pico.h>
#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>
#include <hardware/rtc.h>

#include "ff.h"
#include "diskio.h"

#define MOUNT_POINT ""

static bool s_FatFSLoaded = false;
static FATFS s_FatFS;

SDHOST_STATUS sdhost_card_detect(void)
{
	FRESULT fr; /* FatFs return code */

	fr = f_mount(&s_FatFS, MOUNT_POINT, 1);
	if (fr != FR_OK)
	{
		printf("mount error %d\n", fr);
		return SDHOST_CARD_REMOVED;
	}
	printf("mount ok\n");
	return SDHOST_CARD_INSERTED;
}

void sdhost_init(void)
{
	FRESULT fr; /* FatFs return code */

	fr = f_mount(&s_FatFS, MOUNT_POINT, 1);
	if (fr != FR_OK)
	{
		printf("mount error %d\n", fr);
	}
	printf("mount ok\n");

	switch (s_FatFS.fs_type)
	{
	case FS_FAT12:
		printf("Type is FAT12\n");
		break;
	case FS_FAT16:
		printf("Type is FAT16\n");
		break;
	case FS_FAT32:
		printf("Type is FAT32\n");
		break;
	case FS_EXFAT:
		printf("Type is EXFAT\n");
		break;
	default:
		printf("Type is unknown\n");
		break;
	}
	printf("Card size: %7.2f GB (GB = 1E9 bytes)\n\n", s_FatFS.csize * s_FatFS.n_fatent * 512E-9);
}
/**
 * @brief Mount the SDcard
 *
 * @return true True if ok
 * @return false False if error
 */
bool loadSdCard(void)
{
	SDHOST_STATUS status = sdhost_card_detect();
	if (status == SDHOST_CARD_INSERTED)
	{
		if (!s_FatFSLoaded && (f_mount(&s_FatFS, MOUNT_POINT, 1) != FR_OK))
		{
			printf("FatFS SD card mount failed\n");
		}
		else
		{
			printf("FatFS SD card mounted successfully\n");
			s_FatFSLoaded = true;
		}
	}
	else
	{
		printf("SD card not detected\n");
		s_FatFSLoaded = false;
	}
	return s_FatFSLoaded;
}

bool sdCardReady(void)
{
	/* no-op */
	return s_FatFSLoaded;
}

size_t readFile(uint8_t* buffer, size_t size, const char* filename)
{
	// Read up to `size` number of bytes from the file into `buffer`, then return the number of read bytes
	FRESULT fResult;
	FIL InfSrc;

	if (!s_FatFSLoaded)
	{
		printf("SD card not ready\n");
		return false;
	}

	fResult = f_open(&InfSrc, filename, FA_READ | FA_OPEN_EXISTING);
	if (fResult == FR_DISK_ERR)
	{
		printf("Re-mount SD card\n");
		s_FatFSLoaded = false;
		loadSdCard();
		fResult = f_open(&InfSrc, filename, FA_READ | FA_OPEN_EXISTING);
	}
	if (fResult == FR_OK)
	{
		size_t read;
		fResult = f_read(&InfSrc, buffer, size, &read);
		f_close(&InfSrc);
		return read;
	}
	else
	{
		printf("Unable to open file: \"%s\"\n", filename);
		return 0;
	}
}