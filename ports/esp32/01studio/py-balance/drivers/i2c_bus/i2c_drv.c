#include "mpconfigboard.h"
#include "i2c_drv.h"
#include "esp_log.h"

#define I2C_DEFAULT_SENSORS_CLOCK_SPEED 4000

static const char* TAG = "i2c_drv";
static bool isInit_i2cPort[2] = {0, 0};
static const I2cDef sensorBusDef = {
    .i2cPort        = I2C_NUM_0,
    .gpioSclPin     = MICROPY_HW_SENSOR_I2C_PIN_SCL,
    .gpioSdaPin     = MICROPY_HW_SENSOR_I2C_PIN_SDA,
    .i2cClockSpeed  = I2C_DEFAULT_SENSORS_CLOCK_SPEED,
    .gpioPullup     = GPIO_PULLUP_ENABLE,

};

I2cDrv  sensorsBus  = {
    .def = &sensorBusDef,
};


static void i2cDrvInitBus(I2cDrv *i2c)
{
    if(isInit_i2cPort[i2c->def->i2cPort]){
        return;
    }

    i2c_master_bus_config_t conf = {0};
    conf.i2c_port = i2c->def->i2cPort;
    conf.scl_io_num = i2c->def->gpioSclPin;
    conf.sda_io_num = i2c->def->gpioSdaPin;
    conf.clk_source = I2C_CLK_SRC_DEFAULT;
    conf.glitch_ignore_cnt = 7;
    conf.flags.enable_internal_pullup = i2c->def->gpioPullup;
    ESP_ERROR_CHECK(i2c_new_master_bus(&conf, &i2c->busHandle));

    ESP_LOGI(TAG, "i2c %d driver install", i2c->def->i2cPort);
    isInit_i2cPort[i2c->def->i2cPort] = true;

}

void i2cDrvInit(I2cDrv *i2c)
{
    i2cDrvInitBus(i2c);
}

void i2cDrvDeInit(I2cDrv *i2c)
{
    if(isInit_i2cPort[i2c->def->i2cPort])
    {
        
    }
}