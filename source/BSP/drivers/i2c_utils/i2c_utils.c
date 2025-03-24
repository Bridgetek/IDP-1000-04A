

#include <pico.h>
#include <pico/binary_info.h>

#include "bsp_hwdefs.h"
#include "i2c_utils.h"

void i2c_pins_init()
{
#ifdef I2C_BUS0_SPEED
	i2c_init(i2c0, I2C_BUS0_SPEED);
	gpio_set_function(I2C_BUS0_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_BUS0_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_BUS0_PIN_SDA);
	gpio_pull_up(I2C_BUS0_PIN_SCL);
	bi_decl(bi_2pins_with_func(I2C_BUS0_PIN_SDA, I2C_BUS0_PIN_SCL,
							   GPIO_FUNC_I2C));
#endif // I2C_BUS0_SPEED

#ifdef I2C_BUS1_SPEED
	i2c_init(i2c1, I2C_BUS1_SPEED);
	gpio_set_function(I2C_BUS1_PIN_SDA, GPIO_FUNC_I2C);
	gpio_set_function(I2C_BUS1_PIN_SCL, GPIO_FUNC_I2C);
	gpio_pull_up(I2C_BUS1_PIN_SDA);
	gpio_pull_up(I2C_BUS1_PIN_SCL);
// bi_decl(bi_2pins_with_func(I2C_BUS1_PIN_SDA, I2C_BUS1_PIN_SCL,
// 						   GPIO_FUNC_I2C));
#else
#warning I2C1 Bus is NOT initialised
#endif // I2C_BUS1_SPEED
}


int i2c_reg_rd8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes)
{
	int num_bytes_read = 0;

	// Check to make sure caller is asking for 1 or more bytes
	if (nbytes < 1) {
		return 0;
	}
	// Read data from register(s) over I2C
	i2c_write_blocking(i2c, addr, &reg, 1, true);
	num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

	return num_bytes_read;
}

int i2c_reg_rd16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords)
{
	int num_bytes_read = 0;

	// Check to make sure caller is asking for 1 or more bytes
	if (nwords < 1) {
		return 0;
	}
	// Read data from register(s) over I2C
	i2c_write_blocking(i2c, addr, &reg, 1, true);
	num_bytes_read = i2c_read_blocking(i2c, addr, (uint8_t *)buf, nwords * 2, false);

	return num_bytes_read / 2;
}

int i2c_reg_wr8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes)
{
	int		num_bytes_written = 0;
	uint8_t msg[nbytes + 1];

	// Check to make sure caller is sending 1 or more bytes
	if (nbytes < 1) {
		return 0;
	}
	// Append register address to front of data packet
	msg[0] = reg;
	for (int i = 0; i < nbytes; i++) {
		msg[i + 1] = buf[i];
	}
	// Write data to register(s) over I2C
	num_bytes_written = i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

	return num_bytes_written;
}

int i2c_reg_wr16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords)
{
	int		num_bytes_written = 0;
	uint8_t msg[nwords * 2 + 1];

	// Check to make sure caller is sending 1 or more bytes
	if (nwords < 1) {
		return 0;
	}
	// Append register address to front of data packet
	msg[0] = reg;
	for (int i = 0; i < nwords; i++) {
		msg[i * 2 + 1] = buf[i] >> 8;
		msg[i * 2 + 2] = buf[i] & 0xFF;
	}
	// Write data to register(s) over I2C
	num_bytes_written = i2c_write_blocking(i2c, addr, msg, (nwords * 2 + 1), false);

	return num_bytes_written;
}
