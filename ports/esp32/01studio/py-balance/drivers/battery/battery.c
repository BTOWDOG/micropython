#include "driver/gpio.h"
#include "adc.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "py/mphal.h"

#include "esp_log.h"

#define TAG "ADC"

static bool isInit;


static adc_cali_handle_t adc_cali_handle;

static const adc_channel_t channel = ADC_CHANNEL_9;     // GPIO2 if ADC1
static const adc_bitwidth_t width = ADC_WIDTH_MIN;

static const adc_atten_t atten = ADC_ATTEN_DB_11;   //11dB attenuation (ADC_ATTEN_DB_11) gives full-scale voltage 3.9V
static const adc_unit_t unit = ADC_UNIT_1;
#define DEFAULT_VREF		1100		//Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES		30			//Multisampling

static machine_adc_obj_t *madc;



float analogReadVoltage()
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

    if(count == 0) return 0;


    adc_reading /= count;
    //Convert adc_reading to voltage in mV
    // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    
    int voltage_mv;
    adc_cali_raw_to_voltage(adc_cali_handle, adc_reading, &voltage_mv);
    
    return (voltage_mv / 1000.0) * 4.4f ;
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

