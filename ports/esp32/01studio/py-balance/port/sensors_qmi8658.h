#ifndef SENSORS_QMI8658_H
#define SENSORS_QMI8658_H
#include <stdint.h>

typedef struct
{
    float middleAngle;
    float gyroOffset;

} sensor_cal_data_t;

extern sensor_cal_data_t qmi8658CalData;

void qmi8658Calibration(void);
bool confirmSensorCalibration();
void sensorsQmi8658Init(void);
void qmi8658GetDate(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
#endif