

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
        .device_address  = 0x6A,
        .scl_speed_hz    = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(I2Cx->busHandle, &conf, &I2Cx->devHandle[QMI8658A]));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // uint8_t memAddress = QMI8658_WHO_AM_I;
    // while (1)
    // {
    //     esp_err_t is = i2c_master_transmit_receive(I2Cx->devHandle[QMI8658A], &memAddress, 1, buffer, 1, 200 / portTICK_PERIOD_MS);
    //     if (is == ESP_OK)
    //     {
    //         break;
    //     }else{
    //         printf("test\n");
    //         printf("new bus ret=%d\n", ret);
    //         vTaskDelay(100 / portTICK_PERIOD_MS);
    //     }
        
    // }
    
    isInit = true;
}

void qmi8658DeInit(void)
{
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


