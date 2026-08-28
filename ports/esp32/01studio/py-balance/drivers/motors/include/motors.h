#ifndef MOTORS_H
#define MOTORS_H


#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

typedef struct
{
    int8_t pwmPin;
    int8_t pwmChannel;

    int8_t dir1Pin;
    int8_t dir2Pin;

    int8_t encAPin;
    int8_t encBPin;

    int32_t lastCount;
    float speed;              // count/s

    volatile int32_t count;

    pcnt_unit_handle_t pcntUnit;
    pcnt_channel_handle_t pcntChanA;
    pcnt_channel_handle_t pcntChanB;
} EncoderMotor;

extern EncoderMotor motor[2];


void motorSetSpeed(int id, int duty);
int encoderGetCount(int id);
void motorsInit(void);


#endif