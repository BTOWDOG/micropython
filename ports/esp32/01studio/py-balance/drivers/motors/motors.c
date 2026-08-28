#include "mpconfigboard.h"
#include "motors.h"

static bool isInit = false;

EncoderMotor motor[2];
static void encoderMotorInit(EncoderMotor *motorArr)
{


    for (int i = 0; i < 2; i++) {

        ledc_timer_config_t timerConfig = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .timer_num = i,
            .duty_resolution = LEDC_TIMER_7_BIT,
            .freq_hz = 20000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ledc_timer_config(&timerConfig);

        ledc_channel_config_t channelConfig = {
            .gpio_num = motorArr[i].pwmPin,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = motorArr[i].pwmChannel,
            .timer_sel = i,
            .duty = 0,
            .hpoint = 0,
        };
        ledc_channel_config(&channelConfig);

        gpio_config_t ioConfig = {
            .pin_bit_mask = (1ULL << motorArr[i].dir1Pin) | (1ULL << motorArr[i].dir2Pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&ioConfig);

        gpio_set_level(motorArr[i].dir1Pin, 0);
        gpio_set_level(motorArr[i].dir2Pin, 0);

        pcnt_unit_config_t unitConfig = {
            .high_limit = 32767,
            .low_limit = -32767,
        };
        pcnt_new_unit(&unitConfig, &motorArr[i].pcntUnit);

        // channel A: A 做 edge，B 做 level
        pcnt_chan_config_t chanAConfig = {
            .edge_gpio_num = motorArr[i].encAPin,
            .level_gpio_num = motorArr[i].encBPin,
        };
        ESP_ERROR_CHECK(pcnt_new_channel(motorArr[i].pcntUnit, &chanAConfig, &motorArr[i].pcntChanA));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            motorArr[i].pcntChanA,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,   
            PCNT_CHANNEL_EDGE_ACTION_DECREASE    
        ));

        ESP_ERROR_CHECK(pcnt_channel_set_level_action(
            motorArr[i].pcntChanA,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,      
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE   
        ));

        pcnt_chan_config_t chanBConfig = {
            .edge_gpio_num = motorArr[i].encBPin,
            .level_gpio_num = motorArr[i].encAPin,
        };
        ESP_ERROR_CHECK(pcnt_new_channel(motorArr[i].pcntUnit, &chanBConfig, &motorArr[i].pcntChanB));

        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            motorArr[i].pcntChanB,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,   
            PCNT_CHANNEL_EDGE_ACTION_INCREASE    
        ));

        ESP_ERROR_CHECK(pcnt_channel_set_level_action(
            motorArr[i].pcntChanB,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,      
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE    
        ));

        pcnt_unit_enable(motorArr[i].pcntUnit);
        pcnt_unit_clear_count(motorArr[i].pcntUnit);
        pcnt_unit_start(motorArr[i].pcntUnit);

        motorArr[i].lastCount = 0;
        motorArr[i].speed = 0.0f;
        motorArr[i].count = 0;
    }
}

#define MAX_DUTY 127
#define MIN_DUTY -127

static int limitDuty(int duty)
{
    if (duty > MAX_DUTY) return MAX_DUTY;
    if (duty < MIN_DUTY ) return MIN_DUTY;
    return duty;
}

void motorSetSpeed(int id, int duty)
{
    duty = limitDuty(duty);

    if (duty > 0) {
        gpio_set_level(motor[id].dir1Pin, 1);
        gpio_set_level(motor[id].dir2Pin, 0);
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor[id].pwmChannel, duty);
    } else if (duty < 0) {
        gpio_set_level(motor[id].dir1Pin, 0);
        gpio_set_level(motor[id].dir2Pin, 1);
        
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor[id].pwmChannel, -duty);
    } else {
        gpio_set_level(motor[id].dir1Pin, 0);
        gpio_set_level(motor[id].dir2Pin, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor[id].pwmChannel, 0);
    }

    ledc_update_duty(LEDC_LOW_SPEED_MODE, motor[id].pwmChannel);
}



int encoderGetCount(int id)
{
    int count = 0;
    pcnt_unit_get_count(motor[id].pcntUnit, &count);
    pcnt_unit_clear_count(motor[id].pcntUnit);
    motor[id].count = count;
    return count;
}

void motorsInit(void)
{
    if (isInit) {
        return;
    }

    motor[0].pwmPin = MICROPY_MOTOR01_PWM_PIN;
    motor[0].pwmChannel = MICROPY_MOTOR01_PWM_CHANNEL;
    motor[0].dir1Pin = MICROPY_MOTOR01_DIR1_PIN;
    motor[0].dir2Pin = MICROPY_MOTOR01_DIR2_PIN;
    motor[0].encAPin = MICROPY_MOTOR01_ENC_A_PIN;
    motor[0].encBPin = MICROPY_MOTOR01_ENC_B_PIN;
    motor[0].lastCount = 0;
    motor[0].speed = 0.0f;
    motor[0].count = 0;

    motor[1].pwmPin = MICROPY_MOTOR02_PWM_PIN;
    motor[1].pwmChannel = MICROPY_MOTOR02_PWM_CHANNEL;
    motor[1].dir1Pin = MICROPY_MOTOR02_DIR2_PIN;
    motor[1].dir2Pin = MICROPY_MOTOR02_DIR1_PIN;
    motor[1].encAPin = MICROPY_MOTOR02_ENC_B_PIN;
    motor[1].encBPin = MICROPY_MOTOR02_ENC_A_PIN;
    motor[1].lastCount = 0;
    motor[1].speed = 0.0f;
    motor[1].count = 0;

    encoderMotorInit(motor);
    
    isInit = true;

}