#include "mpconfigboard.h"
#if(MICROPY_HW_I2CDEV_V1)

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2cdev.h"
#include "qmi8658.h"

#include "esp_log.h"
static const char* TAG = "QMI8658";

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static uint8_t buffer[14];
static bool isInit;


void qmi8658Init(I2C_Dev *i2cPort)
{
    if (isInit) {
        return;
    }

    I2Cx = i2cPort;

    i2c_device_config_t conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = QMI8658_ADDRESS_AD0_LOW,
        .scl_speed_hz    = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2Cx->busHandle, &conf, &I2Cx->devHandle[QMI8658A]));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    isInit = true;
}

void qmi8658DeInit(void)
{
    if (!isInit) {
        return;
    }

    if (I2Cx != NULL && I2Cx->devHandle[QMI8658A] != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(I2Cx->devHandle[QMI8658A]);

        if (err != ESP_OK) {
            printf("rm qmi8658 device failed: %s",esp_err_to_name(err));
            return;
        }

        I2Cx->devHandle[QMI8658A] = NULL;
    }

    I2Cx = NULL;
    isInit = false;
}

bool qmi8658Test(void)
{
    bool testStatus;

    if(!isInit){
        return false;
    }

    testStatus = qmi8658TestConnection();

    return testStatus;
}


//
bool qmi8658TestConnection()
{
    return qmi8658GetDeviceID() == 0x05;
}


uint8_t qmi8658GetDeviceID()
{
    // uint8_t memAddress = QMI8658_WHO_AM_I;
    i2cdevReadByte(I2Cx->devHandle[QMI8658A], QMI8658_WHO_AM_I, buffer);
    // printf("++++++++ %d ++++++++++++++++++++\n",is);
    return buffer[0];
}

uint8_t qmi8658ReadByte(uint8_t reg_addr)
{
    i2cdevReadByte(I2Cx->devHandle[QMI8658A], reg_addr, buffer);
    return buffer[0];
}

void qmi8658Read(uint8_t reg_addr, uint16_t len, uint8_t *data)
{
    bool test = i2cdevReadReg8(I2Cx->devHandle[QMI8658A], reg_addr, len, data);
}


void qmi8658WriteByte(uint8_t reg_addr, uint8_t data)
{
    i2cdevWriteByte(I2Cx->devHandle[QMI8658A], reg_addr, data);
}

#endif
