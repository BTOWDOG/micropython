#ifndef SPA06_H
#define SPA06_H

#include <stdbool.h>
#include "mpconfigboard.h"
#if MICROPY_HW_SPA06 || MICROPY_HW_SPA06_V1

#if MICROPY_HW_I2CDEV_V1
#include "i2cdev_v1.h"
#else
#include "i2cdev.h"
#endif

#if MICROPY_HW_SPA06_V1
#define SPA06_I2C_ADDR					(0x77)
#else
#define SPA06_I2C_ADDR					(0x76)
#endif

#define SPA06_DEFAULT_CHIP_ID			(0x11)

#define SPA06_PRESSURE_MSB_REG			(0x00)  /* Pressure MSB Register */
#define SPA06_PRESSURE_LSB_REG			(0x01)  /* Pressure LSB Register */
#define SPA06_PRESSURE_XLSB_REG			(0x02)  /* Pressure XLSB Register */
#define SPA06_TEMPERATURE_MSB_REG		(0x03)  /* Temperature MSB Reg */
#define SPA06_TEMPERATURE_LSB_REG		(0x04)  /* Temperature LSB Reg */
#define SPA06_TEMPERATURE_XLSB_REG		(0x05)  /* Temperature XLSB Reg */
#define SPA06_PRESSURE_CFG_REG			(0x06)	/* Pressure configuration Reg */
#define SPA06_TEMPERATURE_CFG_REG		(0x07)	/* Temperature configuration Reg */
#define SPA06_MODE_CFG_REG				(0x08)  /* Mode and Status Configuration */
#define SPA06_INT_FIFO_CFG_REG			(0x09)	/* Interrupt and FIFO Configuration */
#define SPA06_INT_STATUS_REG			(0x0A)	/* Interrupt Status Reg */
#define SPA06_FIFO_STATUS_REG			(0x0B)	/* FIFO Status Reg */
#define SPA06_RST_REG					(0x0C)  /* Softreset Register */
#define SPA06_CHIP_ID					(0x0D)  /* Chip ID Register */
#define SPA06_COEFFICIENT_CALIB_REG		(0x10)  /* Coeffcient calibraion Register */

#define SPA06_CALIB_COEFFICIENT_LENGTH	(21)  //21
#define SPA06_DATA_FRAME_SIZE			(6)

#define SPA06_CONTINUOUS_MODE			(0x07)

#define TEMPERATURE_INTERNAL_SENSOR		(0)
#define TEMPERATURE_EXTERNAL_SENSOR		(1)

//测量次数 times / S
#define SPA06_MWASURE_1					(0x00)
#define SPA06_MWASURE_2					(0x01)
#define SPA06_MWASURE_4					(0x02)
#define SPA06_MWASURE_8					(0x03)
#define SPA06_MWASURE_16				(0x04)
#define SPA06_MWASURE_32				(0x05)
#define SPA06_MWASURE_64				(0x06)
#define SPA06_MWASURE_128				(0x07)
#define SPA06_RATE_25_16                (0x08)
#define SPA06_RATE_25_8                 (0x09)
#define SPA06_RATE_25_4                 (0x0A)
#define SPA06_RATE_25_2                 (0x0B)
#define SPA06_RATE_25                   (0x0C)
#define SPA06_RATE_50                   (0x0D)
#define SPA06_RATE_100                  (0x0E)
#define SPA06_RATE_200                  (0x0F)

//过采样率
#define SPA06_OVERSAMP_1				(0x00)
#define SPA06_OVERSAMP_2				(0x01)
#define SPA06_OVERSAMP_4				(0x02)
#define SPA06_OVERSAMP_8				(0x03)
#define SPA06_OVERSAMP_16				(0x04)
#define SPA06_OVERSAMP_32				(0x05)
#define SPA06_OVERSAMP_64				(0x06)
#define SPA06_OVERSAMP_128				(0x07)

float spa06_get_temperature(int32_t rawTemperature);
float spa06_get_pressure(int32_t rawPressure, int32_t rawTemperature);
      
bool SPA06Init(I2C_Dev *i2cPort);
float SPA06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/);
void SPA06DeInit(void);

void spa06Read(uint8_t memAddress, uint8_t len, uint8_t *data);



#endif

#endif