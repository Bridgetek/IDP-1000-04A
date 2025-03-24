#include <stdio.h>
#include <string.h>

#include "bsp_hwdefs.h"
#include "i2c_utils.h"
#include "efm8_mio.h"

/* READ REGISTERS*/

/* DIGITAL IO REGISTERS*/
#define REG_MIO_SW_RESET				 0x00
#define REG_MIO_FW_VERSION				 0x01
#define REG_MIO_DEVICE_ID_MSB			 0x02
#define REG_MIO_DEVICE_ID_LSB			 0x03
#define REG_MIO_RESERVED				 0x04
#define REG_MIO_EXTENDED_FUNCTION_STATUS 0x10
#define REG_MIO_STATUS					 0x11
#define REG_MIO_ROTARY					 0x12

#define REG_MIO_FW_UPGRADE	  0xFE
#define MIO_BOOTLOADER_STATUS 0x30
#define MIO_PART_ID			  0x5D10 // MIO app regs are big-endian
#define MIO_PART_ID_OLD		  0x5003
#define MIO_REG_ROTARY_OLDID  0x13
static bool is_old_id = false;

static int efm8_test_id()
{
	uint8_t buf[2];

	int ret = i2c_reg_rd8(EFM8_PICO_I2C_BUS, EFM8_I2C_ADDR,
						  REG_MIO_DEVICE_ID_MSB, buf, sizeof(buf));
	if (ret < 0) {
		printf("%s(): unable to read ID\n", __func__);
		return -1;
	}
	uint16_t id = ((buf[0] << 8) + buf[1]) & 0xFFFF;
	if (id != MIO_PART_ID && id != MIO_PART_ID_OLD) {
		printf("%s(): MIO ID fail: expected 0x%04x or 0x%04x, but got 0x%04x\n",
			   __func__, MIO_PART_ID, MIO_PART_ID_OLD, id);
		return -1;
	}
	is_old_id = (id == MIO_PART_ID_OLD);
	return 0;
}

int efm8_init()
{
	gpio_init(EFM8_IRQN_PICO_PIN);
	gpio_set_dir(EFM8_IRQN_PICO_PIN, GPIO_IN);
	return efm8_test_id();
}

int efm8_get_rotary()
{
	uint8_t	  reg_mio_rotary = is_old_id ? MIO_REG_ROTARY_OLDID : REG_MIO_ROTARY;
	uint8_t	  buf[2];
	const int max_retries = 20;
	int		  a = -1;
	int		  b;
	int		  c;
	int		  tries = 0;

	/* @#$#$%@!! why isn't the rotary switch GPIO giving latest setting!?!! */
	a = b = c = -1;
	do {
		a = b;
		b = c;
		if (i2c_reg_rd8(EFM8_PICO_I2C_BUS, EFM8_I2C_ADDR, reg_mio_rotary,
						buf, 1) > 0) {
			c = buf[0];
		}
		tries++;
		sleep_ms(50);
	} while ((a != b || b != c || c != a) && (tries < max_retries));

	if (tries >= max_retries) {
		return -1;
	}
	return a;
}


