#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "driver/i2c_master.h"

typedef struct 
{
    i2c_port_t          i2cPort;
    gpio_num_t          gpioSclPin;
    gpio_num_t          gpioSdaPin;
    uint32_t            i2cClockSpeed;
    gpio_pullup_t       gpioPullup;
}I2cDef;

typedef enum{
    QMI8658A = 0,
    QMC5883P,
    SPL06,
    MPU6050,
    SENSOR_TYPE_COUNT,
}SensorType_t;

typedef struct 
{
    const I2cDef *def;
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t devHandle[SENSOR_TYPE_COUNT];
}I2cDrv;

extern  I2cDrv  sensorsBus;

void i2cDrvInit(I2cDrv *i2c);
void i2cDrvDeInit(I2cDrv *i2c);

#endif