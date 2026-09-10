/**
 *
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
 * Copyright (C) 2011 Fabio Varesano <fvaresano@yahoo.it>
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "spa06.h"

#if MICROPY_HW_SPA06

#include "math.h"

#include "py/obj.h"
#include "esp_log.h"

#define P_MEASURE_RATE 			SPA06_MWASURE_16 	//每秒测量次数
#define P_OVERSAMP_RATE 		SPA06_OVERSAMP_64	//过采样率
#define SPA06_PRESSURE_CFG		(P_MEASURE_RATE<<4 | P_OVERSAMP_RATE)

#define T_MEASURE_RATE 			SPA06_MWASURE_16 	//每秒测量次数
#define T_OVERSAMP_RATE 		SPA06_OVERSAMP_8	//过采样率
#define SPA06_TEMPERATURE_CFG	(TEMPERATURE_EXTERNAL_SENSOR<<7 | T_MEASURE_RATE<<4 | T_OVERSAMP_RATE)

#define SPA06_MODE				(SPA06_CONTINUOUS_MODE)

const uint32_t scaleFactor[8] = {524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960};
static const char* TAG = "SPA06";

typedef enum 
{
	PRESURE_SENSOR, 
	TEMPERATURE_SENSOR
}spa06Sensor_e;

typedef struct 
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    int16_t c31;
    int16_t c40;
} spa06CalibCoefficient_t;

spa06CalibCoefficient_t  spa06Calib;

static uint8_t devAddr;
static I2C_Dev *I2Cx;
static bool isInit = false;
static uint32_t lastConv = 0;

int32_t kp = 0;
int32_t kt = 0;
int32_t SPL06RawPressure = 0;
int32_t SPL06RawTemperature = 0;

static void SPL06GetPressure(void);


static bool spl06i2cdevRead(I2C_Dev *dev, uint8_t memAddress, uint8_t len, uint8_t *data)
{
	return i2cdevReadReg8(dev->devHandle[SPA06], memAddress, len, data);
}


void spa06Read(uint8_t memAddress, uint8_t len, uint8_t *data)
{
	i2cdevReadReg8(I2Cx->devHandle[SPA06],memAddress, len, data);
}



void spa06_get_calib_param(void)
{
	uint8_t buffer[SPA06_CALIB_COEFFICIENT_LENGTH] = {0};
	
	spl06i2cdevRead(I2Cx, SPA06_COEFFICIENT_CALIB_REG, SPA06_CALIB_COEFFICIENT_LENGTH, buffer);


	spa06Calib.c0 = (int16_t)buffer[0]<<4 | buffer[1]>>4;
	spa06Calib.c0 = (spa06Calib.c0 & 0x0800) ? (spa06Calib.c0 | 0xF000) : spa06Calib.c0;
	
	spa06Calib.c1 = (int16_t)(buffer[1] & 0x0F)<<8 | buffer[2];
	spa06Calib.c1 = (spa06Calib.c1 & 0x0800) ? (spa06Calib.c1 | 0xF000) : spa06Calib.c1;
	
	spa06Calib.c00 = (int32_t)buffer[3]<<12 | (int32_t)buffer[4]<<4 | (int32_t)buffer[5]>>4;
	spa06Calib.c00 = (spa06Calib.c00 & 0x080000) ? (spa06Calib.c00 | 0xFFF00000) : spa06Calib.c00;
	
	spa06Calib.c10 = (int32_t)(buffer[5] & 0x0F)<<16 | (int32_t)buffer[6]<<8 | (int32_t)buffer[7];
	spa06Calib.c10 = (spa06Calib.c10 & 0x080000) ? (spa06Calib.c10 | 0xFFF00000) : spa06Calib.c10;
	
	spa06Calib.c01 = (int16_t)buffer[8]<<8 | buffer[9];
	spa06Calib.c11 = (int16_t)buffer[10]<<8 | buffer[11];
	spa06Calib.c20 = (int16_t)buffer[12]<<8 | buffer[13];
	spa06Calib.c21 = (int16_t)buffer[14]<<8 | buffer[15];
	spa06Calib.c30 = (int16_t)buffer[16]<<8 | buffer[17];

    spa06Calib.c31 = (int16_t)buffer[18]<<4 | buffer[19]>>4;
    spa06Calib.c31 = (spa06Calib.c31 & 0x0800) ? (spa06Calib.c31 | 0xF000) : spa06Calib.c31;

    spa06Calib.c40 = (int16_t)(buffer[19] & 0x0F) << 8 | buffer[20];
    spa06Calib.c40 = (spa06Calib.c40 & 0x0800) ? (spa06Calib.c40 | 0xF000) :spa06Calib.c40;
}

void spa06_rateset(spa06Sensor_e sensor, uint8_t measureRate, uint8_t oversamplRate)
{
	uint8_t reg;
	if (sensor == PRESURE_SENSOR)
	{
		kp = scaleFactor[oversamplRate];
		i2cdevWriteByte(I2Cx->devHandle[SPA06], SPA06_PRESSURE_CFG_REG, measureRate<<4 |oversamplRate);
		if (oversamplRate > SPA06_OVERSAMP_8)
		{
			i2cdevReadByte(I2Cx->devHandle[SPA06], SPA06_INT_FIFO_CFG_REG, &reg);
			i2cdevWriteByte(I2Cx->devHandle[SPA06], SPA06_INT_FIFO_CFG_REG, reg | 0x04);

		}
	}
	else if (sensor == TEMPERATURE_SENSOR)
	{
		kt = scaleFactor[oversamplRate];
		i2cdevWriteByte(I2Cx->devHandle[SPA06], SPA06_TEMPERATURE_CFG_REG, measureRate<<4 | oversamplRate | 0x80);
		if (oversamplRate > SPA06_OVERSAMP_8)
		{
			i2cdevReadByte(I2Cx->devHandle[SPA06], SPA06_INT_FIFO_CFG_REG, &reg);
			i2cdevWriteByte(I2Cx->devHandle[SPA06], SPA06_INT_FIFO_CFG_REG, reg | 0x08);
		}
	}
}

bool SPA06Init(I2C_Dev *i2cPort)
{
	uint8_t SPA06ID = 0;
    if (isInit){
		return true;
	}
        
	I2Cx = i2cPort;

	i2c_device_config_t conf = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = SPA06_I2C_ADDR,
		.scl_speed_hz	 = 400000,
	};

	ESP_ERROR_CHECK(i2c_master_bus_add_device(I2Cx->busHandle, &conf, &I2Cx->devHandle[SPA06]));

	vTaskDelay(10 / portTICK_RATE_MS);

	i2cdevReadByte(I2Cx->devHandle[SPA06], SPA06_CHIP_ID, &SPA06ID);	/* 读取SPL06 ID*/

    if(SPA06ID == SPA06_DEFAULT_CHIP_ID)
		ESP_LOGI(TAG,"SPA06 ID IS: 0x%X\n",SPA06ID);
    else
        return false;

    //读取校准数据
	spa06_get_calib_param();
	spa06_rateset(PRESURE_SENSOR, SPA06_MWASURE_16, SPA06_OVERSAMP_32);
	spa06_rateset(TEMPERATURE_SENSOR, SPA06_MWASURE_16, SPA06_OVERSAMP_1);
	
	i2cdevWriteByte(I2Cx->devHandle[SPA06], SPA06_MODE_CFG_REG, SPA06_MODE);


    isInit = true;
    return true;
}
void SPA06DeInit(void)
{
    if (!isInit) {
        return;
    }

    if (I2Cx != NULL && I2Cx->devHandle[SPA06] != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(I2Cx->devHandle[SPA06]);

        if (err != ESP_OK) {
            printf("rm SPA06 device failed: %s",esp_err_to_name(err));
            return;
        }

        I2Cx->devHandle[SPA06] = NULL;
    }

    I2Cx = NULL;
    isInit = false;
}

float spa06_get_temperature(int32_t rawTemperature)
{
    float fTCompensate;
    float fTsc;

    fTsc = rawTemperature / (float)kt;
    fTCompensate =  spa06Calib.c0 * 0.5 + spa06Calib.c1 * fTsc;
    return fTCompensate;
}

// SPL06 Pcomp​=c00+c10Psc​+c20Psc2​+c30Psc3​+Tsc​(c01+c11Psc​+c21Psc2​)
// SPA06 Pcomp​=c00+c10Psc​+c20Psc2​+c30Psc3​+c40Psc4​+Tsc​(c01+c11Psc​+c21Psc2​+c31Psc3​)
float spa06_get_pressure(int32_t rawPressure, int32_t rawTemperature)
{
    float Tsc = rawTemperature / (float)kt;
    float Psc = rawPressure / (float)kp;

    float P2 = Psc * Psc;
    float P3 = P2 * Psc;
    float P4 = P3 * Psc;

    float Pcomp =
        spa06Calib.c00 +
        spa06Calib.c10 * Psc +
        spa06Calib.c20 * P2 +
        spa06Calib.c30 * P3 +
        spa06Calib.c40 * P4 +
        Tsc * (spa06Calib.c01 +
               spa06Calib.c11 * Psc +
               spa06Calib.c21 * P2 +
               spa06Calib.c31 * P3);

    return Pcomp;   // Pa
}

/**
 * Converts pressure to altitude above sea level (ASL) in meters
 */
 

float SPA06PressureToAltitude(float pressure/*, float* groundPressure, float* groundTemp*/)
{	
    if(pressure)
    {
		return 44330.f * (powf((1015.7f / pressure), 0.190295f) - 1.0f);
    }
    else
    {
        return 0;
    }
}

#endif