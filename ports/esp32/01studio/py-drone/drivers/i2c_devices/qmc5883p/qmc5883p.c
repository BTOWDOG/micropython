#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "config.h"
#include "mpconfigboard.h"
#if(MICROPY_HW_I2CDEV_V1)
#include "i2cdev_v1.h"

#include "qmc5883p.h"
#include "esp_log.h"

#define TAG "QMC5883P"

static uint8_t devAddr;
static uint8_t buffer[6];
static uint8_t mode;
static I2C_Dev *I2Cx;
static bool isInit;

void qmc5883pInit(I2C_Dev *i2cPort)
{
    if (isInit){
        return;
    }

    I2Cx = i2cPort;
    

    i2c_device_config_t conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = QMC5883P_ADDRESS,
        .scl_speed_hz    = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2Cx->busHandle, &conf, &I2Cx->devHandle[QMC5883P]));
    vTaskDelay(1 / portTICK_PERIOD_MS);
    i2cdevWriteByte(I2Cx->devHandle[QMC5883P], QMC5883P_RA_CONFIG_2, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    isInit = true;
}

void qmc5883pDeInit(void)
{
    if (!isInit) {
        return;
    }

    if (I2Cx != NULL && I2Cx->devHandle[QMC5883P] != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(I2Cx->devHandle[QMC5883P]);

        if (err != ESP_OK) {
            printf("rm qmc5883p device failed: %s",esp_err_to_name(err));
            return;
        }

        I2Cx->devHandle[QMC5883P] = NULL;
    }

    I2Cx = NULL;
    isInit = false;
}

bool qmc5883pTestConnection()
{
    uint8_t read_id = 0;
    if(i2cdevReadByte(I2Cx->devHandle[QMC5883P], QMC5883P_CHIP_ID, &read_id) == ESP_OK){
        ESP_LOGI(TAG, "QMC5883P ID IS 0x%X\n", read_id);
        return  (read_id == QMC5883P_DEFAULT_ADDRESS);
    }
    return false;
}

uint8_t qmc5883pReadByte(uint8_t reg_addr)
{
    i2cdevReadByte(I2Cx->devHandle[QMC5883P], reg_addr, buffer);
    return buffer[0];
}

void qmc5883pRead(uint8_t reg_addr, uint16_t len, uint8_t *data)
{
    i2cdevReadReg8(I2Cx->devHandle[QMC5883P], reg_addr, len, data);
}

void qmc5883pWriteByte(uint8_t reg_addr, uint8_t data)
{
    i2cdevWriteByte(I2Cx->devHandle[QMC5883P], reg_addr, data);
}

#endif
