#ifndef I2C_DRV_V1_H
#define I2C_DRV_V1_H
#include "mpconfigboard.h"
#if(MICROPY_HW_I2CDEV_V1)
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
#if MICROPY_HW_SPA06
    SPA06,
#else
    SPL06,
#endif
    SENSOR_TYPE_COUNT,
}SensorType_t;

typedef struct 
{
    const I2cDef *def;
    i2c_master_bus_handle_t busHandle;
    i2c_master_dev_handle_t devHandle[SENSOR_TYPE_COUNT];
}I2cDrv;

extern  I2cDrv  sensorsBus;
extern  I2cDrv  deckBus;

void i2cDrvInit(I2cDrv *i2c);
void i2cDrvDeInit(I2cDrv *i2c);

#endif

#endif