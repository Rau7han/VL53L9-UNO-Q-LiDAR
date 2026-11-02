#ifndef SOFT_I2C_H
#define SOFT_I2C_H
#include <stdint.h>
void soft_i2c_init(void);
void soft_i2c_relax(void);
uint32_t soft_i2c_lines(void);
int soft_i2c_probe(uint8_t address);
int soft_i2c_read(uint8_t address, uint16_t reg, uint8_t *data, uint32_t size);
int soft_i2c_write(uint8_t address, uint16_t reg, const uint8_t *data, uint32_t size);
#endif
