#ifndef I2CDEV_H
#define I2CDEV_H

#include <stdint.h>
#include <stdbool.h>

#include "i2c_drv.h"

#define I2CDEV_NO_MEM_ADDR  0xFF

typedef I2cDrv   I2C_Dev;
#define I2C0_DEV &sensorsBus
#define I2C1_DEV &topSensorsBus

int i2cdevInit(I2C_Dev *dev);

bool i2cdevRead(i2c_master_dev_handle_t devHandle,  uint8_t len, uint8_t *data);
bool i2cdevReadByte(i2c_master_dev_handle_t devHandle, uint8_t memAddress, 
                    uint8_t *data);
bool i2cdevReadBit(i2c_master_dev_handle_t devHandle, uint8_t memAddress, uint8_t bitNum,
                   uint8_t *data);
bool i2cdevReadBits(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data);
bool i2cdevReadReg8(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t len, uint8_t *data);

bool i2cdevWriteByte(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                     uint8_t data);
bool i2cdevWriteBit(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                    uint8_t bitNum, uint8_t data);
bool i2cdevWriteBits(i2c_master_dev_handle_t devHandle, uint8_t memAddres,
                     uint8_t bitStart, uint8_t length, uint8_t data);
bool i2cdevWriteReg8(i2c_master_dev_handle_t devHandle, uint8_t memAddress,
                     uint8_t len, uint8_t *data);
#endif