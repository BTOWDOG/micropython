/**
 * ESP-Drone Firmware
 *
 * Copyright 2019-2020  Espressif Systems (Shanghai)
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
 * adc.c - Analog Digital Conversion
 *
 *
 */


#include "driver/gpio.h"
#include "adc.h"
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include "py/mphal.h"


#include "adc_esp32.h"
#include "config.h"
#include "pm_esplane.h"

#include "esp_log.h"

#define TAG "ADC"

static bool isInit;

// static esp_adc_cal_characteristics_t *adc_chars;
static adc_cali_handle_t adc_cali_handle;

static const adc_channel_t channel = ADC_CHANNEL_1;     // GPIO2 if ADC1
static const adc_bitwidth_t width = ADC_WIDTH_MIN;

static const adc_atten_t atten = ADC_ATTEN_DB_0;   //11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF		1100		//Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES		30			//Multisampling

static machine_adc_obj_t *madc;


float analogReadVoltage(uint32_t pin)
{
    int adc_reading = 0;
    int adc_read = 0;
    int count = 0;
    for (int i = 0; i < NO_OF_SAMPLES; i++) {

        adc_oneshot_read(madc->block->handle, channel, &adc_read);
        if(adc_read > 0){
            adc_reading = adc_reading + adc_read;
            count++;
        }
    }
    adc_reading /= count;

    int voltage_mv;
    adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
    
    
    return voltage_mv / 1000.0;
}

void adcInit(void)
{

    if (isInit) {
        return;
    }

    madc = (machine_adc_obj_t*)madc_search_helper(NULL, channel, -1);

    if(madc == NULL){
        return;
    }


    //Configure ADC
    if (unit == ADC_UNIT_1) {

        if(!madc->block->handle){
            adc_oneshot_unit_init_cfg_t init_config = {
                .unit_id = madc->block->unit_id
            };
            adc_oneshot_new_unit(&init_config, &madc->block->handle);
            adc_oneshot_chan_cfg_t chan_config = {
                .atten = atten,
                .bitwidth = width,
            };
            adc_oneshot_config_channel(madc->block->handle, channel, &chan_config);

            adc_cali_curve_fitting_config_t cali_config = {
                .unit_id = unit,
                .chan = channel,
                .atten = atten,
                .bitwidth = width,
            };
            adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
        }

    }
    
    isInit = true;
}

bool adcTest(void)
{
    return isInit;
}
