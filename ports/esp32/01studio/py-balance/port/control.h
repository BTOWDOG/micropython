#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>

typedef struct 
{
    float acc;
    float gyro;
    float fusion;
}balance_angle_t;

typedef struct 
{
    float leftSpeed;
    float rightSpeed;	
    float aveSpeed; 
    float difSpeed;
    float showSpeed;

}balance_speed_t;


typedef struct 
{
    int16_t leftPwm;
    int16_t rightPwm;	
    int16_t avePwm;
    int16_t difPwm;	
}balance_motor_t;


typedef struct 
{

    uint8_t runFlag;
    uint8_t sportFlag;
    uint16_t angleTick;
    uint16_t speedTick;
    balance_angle_t angle;
    balance_speed_t speed;
    balance_motor_t motor;

}balance_ctrl_t;

extern balance_ctrl_t balCtrl;
void control(void);

#endif