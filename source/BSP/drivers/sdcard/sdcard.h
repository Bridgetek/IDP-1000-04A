#ifndef _TF_CARD_H_
#define _TF_CARD_H_

#include "bsp_hwdefs.h"

// clang-format off
/* SPI pin assignment */
#define PIN_SPI_CS		SD_CS
#define PIN_SPI_SCK		SD_SCK
#define PIN_SPI_MOSI	SD_MOSI
#define PIN_SPI_MISO	SD_MISO
#define TFCARD_SPI		SD_SPI


typedef enum
{
	SDHOST_OK = 0, /**< 0, OK */
	SDHOST_ERROR, /**< general error */
	SDHOST_CARD_INSERTED, /**< card inserted */
	SDHOST_CARD_REMOVED, /**< card removed */
	SDHOST_INVALID_RESPONSE_TYPE, /**< invalid response */
	SDHOST_CMD_TIMEOUT, /**< command timeout */
	SDHOST_UNUSABLE_CARD, /**< card is unusable */
	SDHOST_CMD2_FAILED, /**< command 2 (get CID) failed */
	SDHOST_CMD3_FAILED, /**< command 3 (get RCA) failed */
	SDHOST_CMD8_FAILED, /**< command 8 (voltage check) failed */
	SDHOST_CMD9_FAILED, /**< command 9 (send CSD) failed */
	SDHOST_CMD55_FAILED, /**< command 55 (app cmd) failed */
	SDHOST_ACMD41_FAILED, /**< command 41 failed */
	SDHOST_CANNOT_ENTER_TRANSFER_STATE, /**< cannot enter transfer state */
	SDHOST_CANNOT_SET_CARD_BUS_WIDTH, /**< cannot set bus width */
	SDHOST_RESPONSE_ERROR, /**< response error */
	SDHOST_WRITE_ERROR, /**< read error */
	SDHOST_READ_ERROR, /**< write error */
	SDHOST_NOT_INITIALISED, /**< host is not initialised by driver */
	SDHOST_CARD_NOT_INITIALISED, /**< card is not initialised by driver */
} SDHOST_STATUS;

/** Load SD card */
bool loadSdCard(void);
bool sdCardReady(void);

size_t readFile(uint8_t* buffer, size_t size, const char* filename);

//clang-format on
#endif // _TF_CARD_H_
