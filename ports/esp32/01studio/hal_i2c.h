#ifndef MICROPY_INCLUDED_HAL_I2C_H
#define MICROPY_INCLUDED_HAL_I2C_H
#include "driver/i2c_master.h"


extern i2c_master_bus_handle_t i2c_bus_;


void hal_i2c_init();
i2c_master_dev_handle_t i2c_add_dev(uint8_t addr);
void i2c_write_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t value);
void i2c_read_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t* buffer);


#endif