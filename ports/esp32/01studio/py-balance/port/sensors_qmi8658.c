#include "i2cdev.h"
#include "qmi8658.h"
#include "freertos/FreeRTOS.h"
#include <math.h>
#include "sensors_qmi8658.h"
#include "param_storage.h"
#include "control.h"
#include "py/mpprint.h"
sensor_cal_data_t qmi8658CalData;

bool confirmSensorCalibration(void)
{

    if(saveConfig()){
        balCtrl.angle.fusion = 0;
        return 1;
    }

    return 0;
}


void qmi8658Calibration(void)
{
    const int16_t count = 20;
    int16_t AX, AY, AZ, GX, GY, GZ;
    float gyroSum = 0, pitchSum = 0;

    for (int i = 0; i < count; i++)
    {
        qmi8658GetDate(&AX, &AY, &AZ, &GX, &GY, &GZ);
        
        gyroSum += GY;

        pitchSum += -atan2(AX, AZ) / 3.14159 * 180;

        // gyroSum += GX;
        // pitchSum += -atan2(AY, AZ) * 57.3;
        
        vTaskDelay(pdMS_TO_TICKS(2));

    }

    float gyroAvg = gyroSum / 20.0;
    float pitchAvg = pitchSum / 20.0;
    
    qmi8658CalData.gyroOffset = gyroAvg;
    qmi8658CalData.middleAngle = pitchAvg;

}


// void qmi8658Calibration(void)
// {
//     const int16_t count = 20;
//     int16_t AX, AY, AZ, GX, GY, GZ;
// 	float GX_Array[20] = {0};
// 	float AngleAcc_Array[20] = {0};
// 	uint8_t p = 0;
//     float gyroSum, pitchSum ;
//     float Anglacc;

//     qmi8658GetDate(&AX, &AY, &AZ, &GX, &GY, &GZ);
    
//     Anglacc =  -atan2(AX, AZ) / 3.14159 * 180

//     GX_Array[p] = GY;
//     AngleAcc_Array[p] =  Anglacc;
//     p++;
//     p%=20;

//     for(uint8_t i = 0; i < count; i++)
//     {
//         gyroSum += GY_Array[i];
//         pitchSum += AngleAcc_Array[i]; 
//     }


//     gyroAvg = gyroSum / 30.0;
//     pitchAvg = pitchSum / 30.0;

// }



void qmi8658GetDate(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t buffer[12];
    
    qmi8658Read(0x35, 12, buffer);

    *AccX = (((int16_t)buffer[1]) << 8) | buffer[0];
    *AccY = (((int16_t)buffer[3]) << 8) | buffer[2];
    *AccZ = (((int16_t)buffer[5]) << 8) | buffer[4];

    *GyroX = (((int16_t)buffer[7]) << 8) | buffer[6];
    *GyroY = (((int16_t)buffer[9]) << 8) | buffer[8];
    *GyroZ = (((int16_t)buffer[11]) << 8) | buffer[10];

}


void sensorsQmi8658Init(void)
{
    i2cdevInit(I2C0_DEV);
    qmi8658Init(I2C0_DEV);
    printf("qmi8658 init\n");
    if(qmi8658TestConnection() == true){
        printf("QMI8658 I2C connection [OK].\n");
    }else{
        printf("QMI8658 I2C connection [FAIL].\n");
        while (1){
            vTaskDelay(500 / portTICK_PERIOD_MS);
            if (qmi8658TestConnection() == true){
                printf("QMI8658 I2C connection [OK].\n");
                break;
            }
            
        }
    }
    
    qmi8658WriteByte(QMI8658_RESET, 0xB0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    uint8_t read = qmi8658ReadByte(0x4D);
    if(read == 0x80) { printf("Reset [OK]"); }

    qmi8658WriteByte(0x02, 0x50);  

    qmi8658WriteByte(0x03, 0x03);

    qmi8658WriteByte(0x04, 0x73);

    qmi8658WriteByte(0x06, 0x77);

    qmi8658WriteByte(0x08, 0x83);

    vTaskDelay(100 / portTICK_PERIOD_MS);
}