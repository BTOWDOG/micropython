#ifndef QMC5883P_H_
#define QMC5883P_H_

#include "mpconfigboard.h"
#if(MICROPY_HW_I2CDEV_V1)

#include <stdbool.h>
#include "i2cdev.h"

#define QMC5883P_ADDRESS            0x2C
#define QMC5883P_DEFAULT_ADDRESS    0x80

#define QMC5883P_CHIP_ID            0x00
#define QMC5883P_RA_DATAX_L         0x01
#define QMC5883P_RA_DATAX_H         0x02
#define QMC5883P_RA_DATAY_L         0x03
#define QMC5883P_RA_DATAY_H         0x04
#define QMC5883P_RA_DATAZ_L         0x05
#define QMC5883P_RA_DATAZ_H         0x06
#define QMC5883P_RA_STATUS          0x09

#define QMC5883P_RA_CONFIG_1        0x0A
#define QMC5883P_RA_CONFIG_2	    0x0B

#define QMC5883P_STATUS_DRDY_BIT    0x01

#define QMC5883P_MODE_SUSPEND       0x00  //对应standby
#define QMC5883P_MODE_CONTINUOUS    0X03

#define QMC5883P_OUTPUT_10HZ        0x00
#define QMC5883P_OUTPUT_50HZ        0x04
#define QMC5883P_OUTPUT_100HZ       0x08
#define QMC5883P_OUTPUT_200HZ       0x0C


#define QMC5883P_SAMPLE_64          0x00    // 1,8
#define QMC5883P_SAMPLE_128         0x40    // 2,8  
#define QMC5883P_SAMPLE_256         0x80    // 4,8
#define QMC5883P_SAMPLE_512         0xC0    // 8,8 

#define QMC5883P_SET_RESET_ON       0x03
#define QMC5883P_OUTPUT_2G          0x0c
#define QMC5883P_OUTPUT_8G          0x08



void qmc5883pInit(I2C_Dev *i2cPort);
bool qmc5883pTestConnection();


uint8_t qmc5883pReadByte(uint8_t reg_addr);
void qmc5883pWriteByte(uint8_t reg_addr, uint8_t data);
void qmc5883pRead(uint8_t reg_addr, uint16_t len, uint8_t *data);


#endif

#endif