#include "motors.h"
#include "sensors_qmi8658.h"
#include "control.h"
#include "battery.h"
#include "param_storage.h"
#include "hcsr04.h"

static bool isInit = false;


void systemInit(void)
{
    if(isInit) return;
    
    storageInit();
    motorsInit();
    sensorsQmi8658Init();
    adcInit();
    ultrasonicInit();
    control();
    
    isInit = true;

}
