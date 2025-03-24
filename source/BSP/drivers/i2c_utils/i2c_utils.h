#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#include <stdint.h>
#include <hardware/i2c.h>

#define I2C_DEV_INIT_RETRIES 3
#define I2C_TOTAL_BUSSES	 2	// total number of MCU arch I2C busses
#define I2C_MAX_BUS_ERRS	 10 // maximum errors before resetting bus

typedef void (*i2c_busfailed_cb)(uint32_t);
typedef struct {
	char			 name[10];
	i2c_inst_t		*bus;
	uint8_t			 i2c_addr;
	i2c_busfailed_cb cb;
	int				 err_count;
} I2C_DEV_ENTRY;

void i2c_pins_init();
void i2c_bus_reset(i2c_inst_t *i2c);
void i2c_register_device(char *name, i2c_inst_t *bus, uint8_t i2c_addr, i2c_busfailed_cb cb);

int i2c_reg_rd8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);
int i2c_reg_rd16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords);
int i2c_reg_wr8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);
int i2c_reg_wr16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords);

#endif // __I2C_UTILS_H__