#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
 #include "pid.h"
#include "sensors_qmi8658.h"
#include "motors.h"
#include "battery.h"
#include "control.h"

#define ANGLE_LOOP_TICK_COUNT 10
#define SPEED_LOOP_TICK_COUNT 50 

gptimer_handle_t gptimer = NULL;
TaskHandle_t control_task_handle = NULL; 
static bool isInit = false;
balance_ctrl_t balCtrl;

static bool IRAM_ATTR timer_callback(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *edata,
    void *user_ctx)
{
    BaseType_t high_task_wakeup = pdFALSE;

    if (control_task_handle != NULL) {
        vTaskNotifyGiveFromISR(control_task_handle, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

void balance_timer_init(void)   
{
    gptimer_config_t config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1MHz = 1us
    };

    gptimer_new_timer(&config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback,
    };

    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    gptimer_enable(gptimer);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 1000,   // 1000us = 1ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };

    gptimer_set_alarm_action(gptimer, &alarm_config);

    gptimer_start(gptimer);
}

balance_pid_t anglePid = {					
	.Kp = 4.0,						
	.Ki = 0.1,						
	.Kd = 4.0,						
	
	.OutMax = 127,					
	.OutMin = -127,					
	
	.OutOffset = 2,					
	
	.ErrorIntMax = 600,				
	.ErrorIntMin = -600,			
};

balance_pid_t speedPid = {					
	.Kp = 2.0,						
	.Ki = 0.05,						
	.Kd = 0,						
	
	.OutMax = 20,					
	.OutMin = -20,					
	
	.ErrorIntMax = 150,				
	.ErrorIntMin = -150,			
};

balance_pid_t turnPid = {					
	.Kp = 4,						
	.Ki = 3,						
	.Kd = 0,						
	
	.OutMax = 50,					
	.OutMin = -50,					
	
	.ErrorIntMax = 20,				
	.ErrorIntMin = -20,				
};

float alpha = 0.01;

void control_task(void *arg)
{
    pidInit(&anglePid);
    pidInit(&speedPid);
    pidInit(&turnPid);

	balCtrl.runFlag = 0;
	balCtrl.angleTick = 0;
	balCtrl.speedTick = 0;
	balCtrl.sportFlag = 1;

	int16_t AX, AY, AZ, GX, GY, GZ;
    
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);


        if (++balCtrl.angleTick >= ANGLE_LOOP_TICK_COUNT)
        {
            balCtrl.angleTick = 0;

            qmi8658GetDate(&AX, &AY, &AZ, &GX, &GY, &GZ);


            // GX -=  qmi8658CalData.gyroOffset;
        	// balCtrl.angle.acc = -atan2(AY, AZ) * 57.3f;

			GY -= qmi8658CalData.gyroOffset;
			balCtrl.angle.acc = -atan2(AX, AZ) * 57.3f;
			
			balCtrl.angle.acc -= qmi8658CalData.middleAngle;

            // balCtrl.angle.gyro = balCtrl.angle.fusion + GX / 16.0 * 0.01;
		    balCtrl.angle.gyro = balCtrl.angle.fusion + GY / 16.0 * 0.01;

			balCtrl.angle.fusion = alpha * balCtrl.angle.acc + (1 - alpha) * balCtrl.angle.gyro;		

			if (fabsf(balCtrl.angle.fusion) > 60.0f)	
			{
				balCtrl.runFlag = 0;				
			}
			
			if (balCtrl.runFlag)					
			{
				anglePid.Actual = balCtrl.angle.fusion;	
				pidUpdate(&anglePid);		
				balCtrl.motor.avePwm = -anglePid.Out;

				balCtrl.motor.leftPwm =  balCtrl.motor.avePwm + balCtrl.motor.difPwm ;		
				balCtrl.motor.rightPwm = balCtrl.motor.avePwm - balCtrl.motor.difPwm;		

				motorSetSpeed(0, balCtrl.motor.leftPwm);		
				motorSetSpeed(1, balCtrl.motor.rightPwm);		
			}
			else		
			{

				motorSetSpeed(0, 0);
		        motorSetSpeed(1, 0);
			}
        
        }


		balCtrl.speedTick++;
        if(balCtrl.speedTick >= SPEED_LOOP_TICK_COUNT)
        {
			balCtrl.speedTick = 0;			

			balCtrl.speed.leftSpeed = encoderGetCount(0) / 20.24; // 20.24 减少比*4分频*时间
			balCtrl.speed.rightSpeed = encoderGetCount(1) / 20.24;

			balCtrl.speed.showSpeed = (fabsf(balCtrl.speed.leftSpeed) + fabsf(balCtrl.speed.rightSpeed)) * 105.25; //mm/s
			
			balCtrl.speed.aveSpeed = (balCtrl.speed.leftSpeed + balCtrl.speed.rightSpeed) / 2.0;
			balCtrl.speed.difSpeed = balCtrl.speed.leftSpeed - balCtrl.speed.rightSpeed;			

			if (balCtrl.runFlag)					
			{
				if(balCtrl.sportFlag){
					speedPid.Actual = balCtrl.speed.aveSpeed;			
					pidUpdate(&speedPid);				
					anglePid.Target = speedPid.Out;	
				}	

				turnPid.Actual =  balCtrl.speed.difSpeed;			
				pidUpdate(&turnPid);				
				balCtrl.motor.difPwm = turnPid.Out;				
			}

        }

    }
}



void control(void)
{
    balance_timer_init();

    xTaskCreatePinnedToCore(
    	control_task,
    	"control_task",
    	4096,
    	NULL,
    	10, 
    	&control_task_handle,
    	1    
    );

    isInit = true;
}