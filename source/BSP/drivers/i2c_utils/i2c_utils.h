#ifndef __I2C_UTILS_H__
#define __I2C_UTILS_H__

#include <stdint.h>
#include <hardware/i2c.h>

void i2c_pins_init();

int i2c_reg_rd8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);
int i2c_reg_rd16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords);
int i2c_reg_wr8(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);
int i2c_reg_wr16(i2c_inst_t *i2c, const uint8_t addr, const uint8_t reg, uint16_t *buf, const uint8_t nwords);

#endif // __I2C_UTILS_H__