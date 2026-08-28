#include "i2cdev.h"
#include "string.h"
#include "freertos/FreeRTOS.h"

int i2cdevInit(I2C_Dev *dev)
{
    i2cDrvInit(dev);
    return true;
}

bool i2cdevRead(i2c_master_dev_handle_t devHandle,  uint8_t len, uint8_t *data)
{
    return i2cdevReadReg8(devHandle, I2CDEV_NO_MEM_ADDR, len, data);
}

bool i2cdevReadByte(i2c_master_dev_handle_t devHandle, uint8_t memAddress, 
                    uint8_t *data)
{
    return i2cdevReadReg8(devHandle, memAddress, 1, data);
}

bool i2cdevReadBit(i2c_master_dev_handle_t devHandle, uint8_t memAddress, uint8_t bitNum,
                   uint8_t *data)
{
    uint8_t byte;
    bool status;

    status = i2cdevReadByte(devHandle, memAddress, &byte);
    *data = byte & (1 << bitNum);

    return status;
}

bool i2cdevReadBits(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
    bool status;
    uint8_t byte;

    if((status = i2cdevReadByte(devHandle, memAddress, &byte)) == ESP_OK){
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        byte &= mask;
        byte >>= (bitStart - length + 1);
        *data = byte;
    }

    return status;
}


bool i2cdevReadReg8(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t len, uint8_t *data)
{
    return i2c_master_transmit_receive(devHandle, &memAddress, 1, data, len, 200 / portTICK_PERIOD_MS);
}

bool i2cdevWriteByte(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                     uint8_t data)
{
    return i2cdevWriteReg8(devHandle, memAddress, 1, &data);
}

bool i2cdevWriteBit(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data)
{
    uint8_t byte;
    i2cdevReadByte(devHandle, memAddress, &byte);
    byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));
    return i2cdevWriteByte(devHandle, memAddress, byte);
}

bool i2cdevWriteBits(i2c_master_dev_handle_t devHandle, uint8_t memAddres,
                     uint8_t bitStart, uint8_t length, uint8_t data)
{
    bool status;
    uint8_t byte;

    if((status = i2cdevReadByte(devHandle, memAddres, &byte)) == ESP_OK){
        uint8_t mask = ((1 <<length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1);
        data &= mask;
        byte &= ~(mask);
        byte |= data;
        status = i2cdevWriteByte(devHandle, memAddres, byte);
    }

    return status;
}

bool i2cdevWriteReg8(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                     uint8_t len, uint8_t *data)
{
    uint8_t buf[len + 1];
    buf[0] = memAddress;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(devHandle, buf, len + 1, 200 / portTICK_PERIOD_MS);
}

