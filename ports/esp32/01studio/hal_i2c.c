
#include "mpconfigboard.h"
#if (MICROPY_ENABLE_PCA9557 || MICROPY_ENABLE_ES8311)
#include "hal_i2c.h"

static bool is_init = false;
i2c_master_bus_handle_t i2c_bus_;
void hal_i2c_init()
{
    if(!is_init)
    {
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = 0,
            .sda_io_num = 1,
            .scl_io_num = 2,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_);   
        is_init = true;
    }

}

void hal_i2c_deinit()
{
    // i2c_del_master_bus(i2c_bus_);
    // is_init = false;
}

i2c_master_dev_handle_t i2c_add_dev(uint8_t addr)
{
    i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100 * 1000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };
    i2c_master_dev_handle_t dev;
    i2c_master_bus_add_device(i2c_bus_, &i2c_dev_cfg, &dev);
    return dev;  
}

void i2c_write_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t value)
{
    uint8_t buffer[2] = {reg, value};
    i2c_master_transmit(i2c_dev, buffer, 2, 100);
}

void i2c_read_reg(i2c_master_dev_handle_t i2c_dev, uint8_t reg, uint8_t* buffer)
{
    i2c_master_transmit_receive(i2c_dev, &reg, 1, buffer, 1, 100);
}



#endif