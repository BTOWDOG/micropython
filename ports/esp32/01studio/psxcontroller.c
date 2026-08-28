// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
	******************************************************************************
	* Copyright (C), 2021 -2023, 01studio Tech. Co., Ltd.https://www.01studio.cc/
	* File Name					:	psxcontroller.c
	* Author					:	Folktale
	* Version					:	v1.0
	* date						:	2021/7/1
	* Description				:	
	******************************************************************************
**/

#include "mpconfigboard.h"
#include "esp_system.h"
#include "py/obj.h"
#include <math.h>
#include "py/builtin.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "py/binary.h"
#include "py/objarray.h"
#include "py/mperrno.h"

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "psxcontroller.h"
#include "sdkconfig.h"


#if MICROPY_ENABLE_PSXCONTROLLER

#include "adc.h"
#include "esp_log.h"

#include "py/runtime.h"
#include "py/mphal.h"
#include "modmachine.h"


#define GPIO_PULL_DOWN (1)
#define GPIO_PULL_UP   (2)
#define GPIO_PULL_HOLD (4)

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_0

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

#define NO_OF_SAMPLES	10

static bool is_initpsx = 0;

static machine_adc_obj_t *madc;
static uint8_t adc_bit_width;

//esp_err_t adc1_config_width(adc_bits_width_t width_bit)
//esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel, const adc_oneshot_chan_cfg_t *config)
//esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *init_config, adc_oneshot_unit_handle_t *ret_unit)
//esp_err_t adc1_config_channel_atten(adc1_channel_t channel, adc_atten_t atten)
//esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t chan, int *out_raw)
//int adc1_get_raw(adc1_channel_t channel)
static void joy_gpio_init(void)
{
	esp_err_t err;
	
	// adc_oneshot_unit_init_cfg_t init_cfg = {
	// 	.unit_id = ADC_UNIT_1,
	// };

	// adc_oneshot_new_unit(&init_cfg,&adc1);

	#if CONFIG_IDF_TARGET_ESP32S2
	// adc_oneshot_new_unit(ADC_WIDTH_BIT_13); 
	adc_bit_width = 13;
	#elif CONFIG_IDF_TARGET_ESP32C3
	// adc_oneshot_new_unit(ADC_WIDTH_BIT_12);
	adc_bit_width = 12;
	#else
	// adc_oneshot_new_unit(ADC_WIDTH_BIT_12);
	adc_bit_width = 12;
	#endif
	
	adc_oneshot_chan_cfg_t chan_cfg = {
		.atten = ADC_EXAMPLE_ATTEN,
		.bitwidth = adc_bit_width,
	};

	madc = (machine_adc_obj_t*)madc_search_helper(NULL, PSX_ADCL_LEFTRIGHT_CH, -1);
	madc = (machine_adc_obj_t*)madc_search_helper(NULL, PSX_ADCL_UPDOWN_CH, -1);
	madc = (machine_adc_obj_t*)madc_search_helper(NULL, PSX_ADCR_LEFTRIGHT_CH, -1);
	madc = (machine_adc_obj_t*)madc_search_helper(NULL, PSX_ADCR_UPDOWN_CH, -1);

    if (!madc) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid ADC_CHANNEL"));
    }

	if (!madc->block->handle)
	{
		adc_oneshot_unit_init_cfg_t	init_config = {
			.unit_id = madc->block->unit_id
		};
		check_esp_err(adc_oneshot_new_unit(&init_config, &madc->block->handle));
	}
	
	err = adc_oneshot_config_channel(madc->block->handle, PSX_ADCL_LEFTRIGHT_CH, &chan_cfg);
    if (err != ESP_OK) {
        printf("PSX_ADCL_LEFTRIGHT_CH init error\r\n");
    }
	err = adc_oneshot_config_channel(madc->block->handle, PSX_ADCL_UPDOWN_CH, &chan_cfg);
    if (err != ESP_OK) {
        printf("PSX_ADCL_UPDOWN_CH init error\r\n");
    }

	err = adc_oneshot_config_channel(madc->block->handle, PSX_ADCR_LEFTRIGHT_CH, &chan_cfg);
    if (err != ESP_OK) {
        printf("PSX_ADCR_LEFTRIGHT_CH init error\r\n");
    }
	
	err = adc_oneshot_config_channel(madc->block->handle, PSX_ADCR_UPDOWN_CH, &chan_cfg);
    if (err != ESP_OK) {
        printf("PSX_ADCR_UPDOWN_CH init error\r\n");
    }
}
static void gpio_global_init(gpio_num_t gpio,gpio_mode_t io_mode,uint8_t mode)
{
	if (rtc_gpio_is_valid_gpio(gpio)) {
		#if !CONFIG_IDF_TARGET_ESP32C3
		rtc_gpio_deinit(gpio);
		#endif
	}
	esp_rom_gpio_pad_select_gpio(gpio);
	gpio_set_direction(gpio, io_mode);
	
	if (mode & GPIO_PULL_DOWN) {
		gpio_pulldown_en(gpio);
	} else {
		gpio_pulldown_dis(gpio);
	}
	if (mode & GPIO_PULL_UP) {
		gpio_pullup_en(gpio);
	} else {
		gpio_pullup_dis(gpio);
	}
	if (mode & GPIO_PULL_HOLD) {
		gpio_hold_en(gpio);
	} else if (GPIO_IS_VALID_OUTPUT_GPIO(gpio)) {
		gpio_hold_dis(gpio);
	}
}
static void input_gpio_init(void)
{
	gpio_global_init(PSX_GPIO_UP,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_DOWN,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_LEFT,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_RIGHT,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_X,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_Y,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_A,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_B,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_BACK,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_START,GPIO_MODE_INPUT,GPIO_PULL_UP);
	
	gpio_global_init(PSX_GPIO_A_OK,GPIO_MODE_INPUT,GPIO_PULL_UP);
	gpio_global_init(PSX_GPIO_B_OK,GPIO_MODE_INPUT,GPIO_PULL_UP);
}

static void joy_adc_read(uint16_t *joy_leftright,uint16_t *joy_updown)
{	
	int adc_reading1 = 0;
	int adc_reading2 = 0;
	int reading1 = 0;
	int reading2 = 0;
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		adc_oneshot_read(madc->block->handle, PSX_ADCL_LEFTRIGHT_CH, &reading1);
		adc_oneshot_read(madc->block->handle, PSX_ADCL_UPDOWN_CH, &reading2);
		adc_reading1 += reading1;
		adc_reading2 += reading2;
	}
	adc_reading1 /= NO_OF_SAMPLES;
	adc_reading2 /= NO_OF_SAMPLES;
	
	*joy_leftright = (uint16_t)adc_reading1;
	*joy_updown = (uint16_t)adc_reading2;
}

void gamepad_get_adc(uint16_t *leftX ,uint16_t *leftY,uint16_t *rightX,uint16_t *rightY)
{
	int adc_reading1 = 0;
	int adc_reading2 = 0;
	int adc_reading3 = 0;
	int adc_reading4 = 0;
	int reading1 = 0;
	int reading2 = 0;
	int reading3 = 0;
	int reading4 = 0;
	//Multisampling
	for (int i = 0; i < NO_OF_SAMPLES; i++) {
		adc_oneshot_read(madc->block->handle, PSX_ADCL_LEFTRIGHT_CH, &reading1);
		adc_oneshot_read(madc->block->handle, PSX_ADCL_UPDOWN_CH, &reading2);
		adc_oneshot_read(madc->block->handle, PSX_ADCR_LEFTRIGHT_CH, &reading3);
		adc_oneshot_read(madc->block->handle, PSX_ADCR_UPDOWN_CH, &reading4);
		adc_reading1 += reading1;
		adc_reading2 += reading2;
		adc_reading3 += reading3;
	adc_reading4 += reading4;
	}
	adc_reading1 /= NO_OF_SAMPLES;
	adc_reading2 /= NO_OF_SAMPLES;
	adc_reading3 /= NO_OF_SAMPLES;
	adc_reading4 /= NO_OF_SAMPLES;
	
	*leftX = (uint16_t)adc_reading1;
	*leftY = (uint16_t)adc_reading2;
	
	*rightX = (uint16_t)adc_reading3;
	*rightY = (uint16_t)adc_reading4;

	// mp_printf("%d %d %d %d\n",*leftX,*leftY,*rightX,*rightY);
}
/****************************************************************
Byte5:
buttons - right cluster buttons
bit7	X
bit6	A
bit5	B
bit4	Y

VALUE	KEY
0	up
1	right&up

2	right
3	right&down
4	down

5	left&down
6	left
7	left&up
8	no action

Byte6:
button
BIT		KEY
bit7	AOK
bit6	BOK
bit5	START
bit4	BACK
bit3	RT
bit2	LT
bit1	RB
bit0	LB
****************************************************************/
void gamepad_get_key(uint8_t *keyByte5,uint8_t * keyByte6)
{
	uint8_t key5 = 0,key6 = 0;
	bool up = 0,down=0,left=0,right=0;

	if(PSX_UP_PRESSED == gpio_get_level(PSX_GPIO_UP)){
		up = 1;
	}
	if(PSX_RIGHT_PRESSED == gpio_get_level(PSX_GPIO_RIGHT)){
		right = 1;
	}
	if(PSX_DOWN_PRESSED == gpio_get_level(PSX_GPIO_DOWN)){
		down = 1;
	}
	if(PSX_LEFT_PRESSED == gpio_get_level(PSX_GPIO_LEFT)){
		left = 1;
	}
	if(up){
		if(right)		key5 = 1;
		else if(left)	key5 = 7;
		else			key5 = 0;
	}else if(down){
		if(right)		key5 = 3;
		else if(left)	key5 = 5;
		else			key5 = 4;
	}else if(right){
						key5 = 2;
	}else if(left){
						key5 = 6;
	}else{
						key5 = 8;
	}

	if(PSX_A_PRESSED == gpio_get_level(PSX_GPIO_A)){
		key5 |= (1<<6);
	}
	if(PSX_B_PRESSED == gpio_get_level(PSX_GPIO_B)){
		key5 |= (1<<5);
	}
	if(PSX_X_PRESSED == gpio_get_level(PSX_GPIO_X)){
		key5 |= (1<<7);
	}
	if(PSX_Y_PRESSED == gpio_get_level(PSX_GPIO_Y)){
		key5 |= (1<<4);
	}
	if(PSX_BACK_PRESSED == gpio_get_level(PSX_GPIO_BACK)){
		key6 |= (1<<4);
	}
	if(PSX_START_PRESSED == gpio_get_level(PSX_GPIO_START)){
		key6 |= (1<<5);
	}
	if(PSX_AOK_PRESSED == gpio_get_level(PSX_GPIO_A_OK)){
		key6 |= (1<<7);
	}
	if(PSX_BOK_PRESSED == gpio_get_level(PSX_GPIO_B_OK)){
		key6 |= (1<<6);
	}
	
	*keyByte5 = key5;
	*keyByte6 = key6;
}
/*********************************************************************
 ________________________________________________________________________________________________________
|  15 |   14  |   13  |   12   | 11| 10| 9 | 8 |   7  |   6   |  5  |   4   |   3   |   2    |  1  |  0  |
|-----|-------|-------|--------|---|---|---|---|------|-------|-----|-------|-------|--------|-----|-----|
| UP1 | DOWN1 | LEFT1 | RIGHT1 | A | B | X | Y | BACK | START | UP2 | DOWN2 | LEFT2 | RIGHT2 | TDB | TDB |
|-----|-------|-------|--------|---|---|---|---|------|-------|-----|-------|-------|--------|-----|-----|

*********************************************************************/
static int joy_read_data(void)
{
	int pxsInput = 0xffff;
	uint16_t leftright_voltage=0,updown_voltage=0;

	uint8_t lr_dir = 0,ud_dir = 0;
	
	joy_adc_read(&leftright_voltage,&updown_voltage);
	// printf("leftright_voltage: %dmV,updown_voltage:%dmV\n", leftright_voltage, updown_voltage);

	if(leftright_voltage >= PSX_JOY_RIGHT_VALUE){
		lr_dir = 1;  //right
	}else if(leftright_voltage <= PSX_JOY_LEFT_VALUE){
		lr_dir = 2; //left左
	}
	
	if(updown_voltage >= PSX_JOY_UP_VALUE){
		ud_dir = 1; //up
	}else if(updown_voltage <= PSX_JOY_DOWN_VALUE){
		ud_dir = 2; //down
	}
	//up1
	if(PSX_UP_PRESSED == gpio_get_level(PSX_GPIO_UP) || ud_dir == 1){
		pxsInput &= ~(1<<15);
	}
	//down1
	if(PSX_DOWN_PRESSED == gpio_get_level(PSX_GPIO_DOWN) || ud_dir == 2){
		pxsInput &= ~(1<<14);
	}
	//right1
	if(PSX_RIGHT_PRESSED == gpio_get_level(PSX_GPIO_RIGHT) || lr_dir == 1){
		pxsInput &= ~(1<<12);
	}
	//left1
	if(PSX_LEFT_PRESSED == gpio_get_level(PSX_GPIO_LEFT) || lr_dir == 2){
		pxsInput &= ~(1<<13);
	}
	if(PSX_A_PRESSED == gpio_get_level(PSX_GPIO_A)){
		pxsInput &= ~(1<<11);
	}
	if(PSX_B_PRESSED == gpio_get_level(PSX_GPIO_B)){
		pxsInput &= ~(1<<10);
	}
	if(PSX_X_PRESSED == gpio_get_level(PSX_GPIO_X)){
		pxsInput &= ~(1<<9);
	}
	if(PSX_Y_PRESSED == gpio_get_level(PSX_GPIO_Y)){
		pxsInput &= ~(1<<8);
	}
	if(PSX_BACK_PRESSED == gpio_get_level(PSX_GPIO_BACK)){
		pxsInput &= ~(1<<7);
	}
	if(PSX_START_PRESSED == gpio_get_level(PSX_GPIO_START)){
		pxsInput &= ~(1<<6);
	}

	return pxsInput;

}

int psxReadInput() {
	return joy_read_data();
}

void psxcontrollerInit() {
	if(!is_initpsx){
		input_gpio_init();
		joy_gpio_init();
		is_initpsx = 1;
	}
}

#else
int psxReadInput() {
	return 0xFFFF;
}
void psxcontrollerInit() {

}

#endif