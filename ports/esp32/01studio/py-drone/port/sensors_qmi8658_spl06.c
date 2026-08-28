#include "mpconfigboard.h"
#if MICROPY_HW_QMI8658

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "soc/gpio_sig_map.h"

#include "i2cdev.h"
#include "qmi8658.h"
#include "qmc5883p.h"

#if MICROPY_HW_SPA06 || MICROPY_HW_SPA06_V1
#include "spa06.h"
#else
#include "spl06.h"
#endif

#include "stabilizer_types.h"
#include "filter.h"

#include "config.h"


#define GYRO_LPF_CUTOFF_FREQ 80
#define ACCEL_LPF_CUTOFF_FREQ 30
static lpf2pData accLpf[3];
static lpf2pData gyroLpf[3];

static bool isBarometerPresent = false;
static bool isMagnetometerPresent = false;

static bool isInit = false;

static const char* TAG = "qmi8658_spl06";


static xSemaphoreHandle sensorsDataReady;
static xSemaphoreHandle dataReady;

static xQueueHandle accelerometerDataQueue;
static xQueueHandle gyroDataQueue;
static xQueueHandle magnetometerDataQueue;
static xQueueHandle barometerDataQueue;

static volatile uint64_t imuIntTimestamp;
static sensorData_t sensorData;


static uint8_t isprintf = 0;

#define SENSORS_ENABLE_PRESSURE_SPL06
#define SENSORS_ENABLE_MAG_HM5883L 
#define MICROPY_MPU_PIN_IRQ				(9)


// Number of samples used in variance calculation. Changing this effects the threshold
#define SENSORS_NBR_OF_BIAS_SAMPLES 1024 /* 计算方差的采样样本个数 */
#define MAG_GAUSS_PER_LSB 12000//666.7f


typedef struct {
    Axis3f bias;
    Axis3f variance;//
    Axis3f mean;//
    bool isBiasValueFound;
    bool isBufferFilled;
    Axis3i16 *bufHead;
    Axis3i16 buffer[SENSORS_NBR_OF_BIAS_SAMPLES];
} BiasObj;
            
static BiasObj gyroBiasRunning;
static Axis3f gyroBias;

static Axis3i16 gyroRaw;
static Axis3i16 accelRaw;
static Axis3i16 magRaw;

static float TempRaw;
static float PressureRaw;
static float AslRaw;

static Axis3f readvariance;

/*传感器偏置初始化*/
static void sensorsBiasObjInit(BiasObj *bias)
{
    bias->isBufferFilled = false;
    bias->bufHead = bias->buffer;  
}




static bool gyroBiasFound = false;
static float accScaleSum = 0;
static float accScale = 1;


/*传感器数据校准*/
bool sensorsAreCalibrated(void)	
{
	return gyroBiasFound;
}

/*从队列读取陀螺数据*/
bool sensorsReadGyro(Axis3f *gyro)
{
    return (pdTRUE == xQueueReceive(gyroDataQueue, gyro, 0));
}

/*从队列读取加速计数据*/
bool sensorsReadAcc(Axis3f *acc)
{
    return (pdTRUE == xQueueReceive(accelerometerDataQueue, acc, 0));
}

/*从队列读取磁力计数据*/
bool sensorsReadMag(Axis3f *mag)
{
    return (pdTRUE == xQueueReceive(magnetometerDataQueue, mag, 0));
}

/*从队列读取气压数据*/
bool sensorsReadBaro(baro_t *baro)
{
	return (pdTRUE == xQueueReceive(barometerDataQueue, baro, 0));
}

/*获取传感器数据*/
void sensorsAcquire(sensorData_t *sensors, const uint32_t tick)	
{
    sensorsReadGyro(&sensors->gyro);
    sensorsReadAcc(&sensors->acc);
    sensorsReadMag(&sensors->mag);
    sensorsReadBaro(&sensors->baro);
    sensors->interruptTimestamp = sensorData.interruptTimestamp;
}


static void sensorsDeviceInit(void)
{
	isMagnetometerPresent = false;
	isBarometerPresent = false;
lab1:
    i2cdevInit(I2C0_DEV);
#if MICROPY_HW_SPL06_V1 || MICROPY_HW_SPA06_V1
    i2cdevInit(I2C1_DEV);
#endif
    printf("i2c init\n");
    qmi8658Init(I2C0_DEV);
    printf("qmi8658 init\n");
    if(qmi8658TestConnection() == true){
        ESP_LOGI(TAG, "QMI8658 I2C connection [OK].\n");
    }else{
        ESP_LOGI(TAG, "QMI8658 I2C connection [FAIL].\n");
        while (1){
            vTaskDelay(100 / portTICK_PERIOD_MS);
            if (qmi8658TestConnection() == true){
                ESP_LOGI(TAG, "QMI8658 I2C connection [OK].\n");
                break;
            }
            
        }
    }

    printf("Init i2cdev sensors\n");
    ESP_LOGD(TAG, "start Reset QMI8658\n");
    //reset
    qmi8658WriteByte(QMI8658_RESET, 0xB0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    

    uint8_t read = qmi8658ReadByte(0x4D);
    if(read == 0x80) { ESP_LOGI(TAG, "Reset [OK]"); }
    else goto lab1;
    
    //auto i2c addre  int1_en 
    qmi8658WriteByte(0x02, 0x58);  

    //set ACC hz  8658：16g  896.8          6050： acc fs_16     16g 1000hz 
    qmi8658WriteByte(0x03, 0x32);
    //set gyr hz  8658:2048  896.8          6050： gyro fs_2000 2000dps 1000hz  
    qmi8658WriteByte(0x04, 0x72);


    //enable  ACC gry  
    qmi8658WriteByte(0x08, 0x03);//1010

    // qmi8658WriteByte(0x02, 0x50);
    
    // qmi8658WriteByte(0x03, 0x33);

    // qmi8658WriteByte(0x04, 0x63);

    // vTaskDelay(100 / portTICK_PERIOD_MS);
    
    qmi8658WriteByte(0x06, 0x33);  

    for (uint8_t i = 0; i < 3; i++) {
        lpf2pInit(&gyroLpf[i], 1793.6, GYRO_LPF_CUTOFF_FREQ);
        lpf2pInit(&accLpf[i], 1793.6, ACCEL_LPF_CUTOFF_FREQ);
    }
    
#ifdef SENSORS_ENABLE_MAG_HM5883L
    qmc5883pInit(I2C0_DEV);

    if (qmc5883pTestConnection() == true) {
        isMagnetometerPresent = true;
        //hmc5883lSetMode(QMC5883L_MODE_CONTINUOUS); // 16bit 100Hz Continuous 
        //QMC5883L_OUTPUT_10HZ | QMC5883L_OUTPUT_2G | QMC5883L_SAMPLE_128
        uint8_t tempMode = QMC5883P_MODE_CONTINUOUS || QMC5883P_OUTPUT_10HZ || QMC5883P_SAMPLE_128;
        qmc5883pWriteByte(QMC5883P_RA_CONFIG_1, tempMode);
        ESP_LOGI(TAG,"qmc5883p I2C connection [OK].\n");
		// debugpeintf("hmc5883l I2C connection [OK]\r\n");
    } else {
        ESP_LOGE(TAG,"qmc5883p I2C connection [FAIL].\n");
		// debugpeintf("hq5883l I2C connection [FAIL]\r\n");
        goto lab1;
    }

#endif
#if MICROPY_HW_SPL06_V1 || MICROPY_HW_SPA06_V1
    #if MICROPY_HW_SPA06_V1
    
    if (SPA06Init(I2C1_DEV)) {
        isBarometerPresent = true;
        ESP_LOGI(TAG,"SPA06 I2C v1 connection [OK].\n");
        printf("SPA06 I2C V1 \n");
    } else {
        //TODO: Should sensor test fail hard if no connection
       ESP_LOGE(TAG,"SPA06 I2C v1 connection [FAIL].\n");
	   goto lab1;
    }

    #else
    if (SPL06Init(I2C1_DEV)) {
        isBarometerPresent = true;
        ESP_LOGI(TAG,"SPL06 I2C v1 connection [OK].\n");
        printf("SPL06 I2C V1 \n");
    } else {
        //TODO: Should sensor test fail hard if no connection
       ESP_LOGE(TAG,"SPL06 I2C v1 connection [FAIL].\n");
	   goto lab1;
    }

    #endif
#else
    #if MICROPY_HW_SPA06
    
    if (SPA06Init(I2C0_DEV)) {
        isBarometerPresent = true;
        ESP_LOGI(TAG,"SPA06 I2C connection [OK].\n");
        printf("SPL06 I2C  \n");
    } else {
        //TODO: Should sensor test fail hard if no connection
       ESP_LOGE(TAG,"SPA06 I2C connection [FAIL].\n");
	   goto lab1;
    }

    #else
    if (SPL06Init(I2C0_DEV)) {
        isBarometerPresent = true;
        ESP_LOGI(TAG,"SPL06 I2C connection [OK].\n");
    } else {
        //TODO: Should sensor test fail hard if no connection
       ESP_LOGE(TAG,"SPL06 I2C connection [FAIL].\n");
	   goto lab1;
    }
    #endif
#endif

}

#define ESP_INTR_FLAG_DEFAULT 0

static void IRAM_ATTR sensors_inta_isr_handler(void *arg)
{

    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    imuIntTimestamp = usecTimestamp(); //This function returns the number of microseconds since esp_timer was initialized
    xSemaphoreGiveFromISR(sensorsDataReady, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


static void sensorsInterruptInit(void)
{

    gpio_config_t io_conf;
    //interrupt of rising edge
#if ESP_IDF_VERSION_MAJOR > 4
    io_conf.intr_type = GPIO_INTR_POSEDGE;
#else
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
#endif
    //bit mask of the pins
    io_conf.pin_bit_mask = (1ULL << MICROPY_MPU_PIN_IRQ);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    sensorsDataReady = xSemaphoreCreateBinary();
    dataReady = xSemaphoreCreateBinary();
    gpio_config(&io_conf);
    //install gpio isr service
    //portDISABLE_INTERRUPTS();
    gpio_set_intr_type(MICROPY_MPU_PIN_IRQ, GPIO_INTR_POSEDGE);
    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(MICROPY_MPU_PIN_IRQ, sensors_inta_isr_handler, (void *)MICROPY_MPU_PIN_IRQ);
    //portENABLE_INTERRUPTS();

    //   FSYNC "shall not be floating, must be set high or low by the MCU"
}

#define SENSORS_GYRO_FS_CFG MPU6050_GYRO_FS_2000
#define SENSORS_DEG_PER_LSB_CFG (float)((2 * 2048.0) / 65536.0)

#define SENSORS_ACCEL_FS_CFG MPU6050_ACCEL_FS_16
#define SENSORS_G_PER_LSB_CFG (float)((2 * 16) / 65536.0)


// /*处理磁力计数据*/
void processMagnetometerMeasurements(const uint8_t *buffer)
{
	#ifdef SENSORS_ENABLE_MAG_HM5883L
    //TODO: replace it to hmc5883l
    if (buffer[0] & QMC5883P_STATUS_DRDY_BIT) {
        int16_t headingx = (((int16_t)buffer[2]) << 8) | buffer[1];
        int16_t headingy = (((int16_t)buffer[4]) << 8) | buffer[3];
        int16_t headingz = (((int16_t)buffer[6]) << 8) | buffer[5];

        sensorData.mag.x = (float)headingx / MAG_GAUSS_PER_LSB; //to gauss
        sensorData.mag.y = (float)headingy / MAG_GAUSS_PER_LSB;
        sensorData.mag.z = (float)headingz / MAG_GAUSS_PER_LSB;
		
		magRaw.x = headingx;/*用于上传到上位机*/
		magRaw.y = headingy;
		magRaw.z = headingz;
		
    }
	#endif
}

/*处理气压计数据*/
void processBarometerMeasurements(const uint8_t *buffer)
{
	static float temp;
	static float pressure;

	// Check if there is a new data update
	if(!isBarometerPresent){
		return;
	}

	int32_t Pressure = (int32_t)buffer[1]<<16 | (int32_t)buffer[2]<<8 | (int32_t)buffer[3];
	Pressure = (Pressure & 0x800000) ? (0xFF000000 | Pressure) : Pressure;

	int32_t rawTemp = (int32_t)buffer[4]<<16 | (int32_t)buffer[5]<<8 | (int32_t)buffer[6];
	rawTemp = (rawTemp & 0x800000) ? (0xFF000000 | rawTemp) : rawTemp;

    // printf("Pressure %ld rawTemp %ld\n", Pressure, rawTemp);
    
    #if MICROPY_HW_SPA06 || MICROPY_HW_SPA06_V1
	temp = spa06_get_temperature(rawTemp);
	pressure = spa06_get_pressure(Pressure, rawTemp);
	sensorData.baro.pressure = pressure / 100.0f;
	sensorData.baro.temperature = (float)temp; /*单位度*/
	sensorData.baro.asl = SPA06PressureToAltitude(sensorData.baro.pressure) * 100.f; //cm

    #else
	temp = spl0601_get_temperature(rawTemp);
	pressure = spl0601_get_pressure(Pressure, rawTemp);
	sensorData.baro.pressure = pressure / 100.0f;
	sensorData.baro.temperature = (float)temp; /*单位度*/
	sensorData.baro.asl = SPL06PressureToAltitude(sensorData.baro.pressure) * 100.f; //cm

    #endif
}


#define GYRO_NBR_OF_AXES 3
#define GYRO_MIN_BIAS_TIMEOUT_MS M2T(1 * 1000)


#define GYRO_VARIANCE_BASE 5000  /* 陀螺仪零偏方差阈值 */

#define GYRO_VARIANCE_THRESHOLD_X (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Y (GYRO_VARIANCE_BASE)
#define GYRO_VARIANCE_THRESHOLD_Z (GYRO_VARIANCE_BASE)

/*计算方差和平均值*/
static void sensorsCalculateVarianceAndMean(BiasObj *bias, Axis3f *varOut, Axis3f *meanOut)
{
    uint32_t i;
    int32_t sum[GYRO_NBR_OF_AXES] = {0};
    int32_t sumSq[GYRO_NBR_OF_AXES] = {0};

    for (i = 0; i < SENSORS_NBR_OF_BIAS_SAMPLES; i++) {
        sum[0] += bias->buffer[i].x;
        sum[1] += bias->buffer[i].y;
        sum[2] += bias->buffer[i].z;
        sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
        sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
        sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
        // printf("[%ld] sum[2] = %ld buffer[2] = %d sumSq[2] = %ld \n", 
        //         i + 1,
        //         sum[2],
        //         bias->buffer[i].z, sumSq[2]);
        // vTaskDelay(1);
        
    }


    varOut->x = (sumSq[0] - ((int64_t) sum[0] * sum[0]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->y = (sumSq[1] - ((int64_t) sum[1] * sum[1]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    varOut->z = (sumSq[2] - ((int64_t) sum[2] * sum[2]) / SENSORS_NBR_OF_BIAS_SAMPLES);
    //printf("%ld ++++++++++++%lld+++++++++++++++++++++++++++ %ld\n", sum[2] ,(int64_t) sum[2] *sum[2] , sumSq[2]);
    meanOut->x = (float)sum[0] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->y = (float)sum[1] / SENSORS_NBR_OF_BIAS_SAMPLES;
    meanOut->z = (float)sum[2] / SENSORS_NBR_OF_BIAS_SAMPLES;
    // printf("%f %f %f -----------------------------\n", meanOut->x, meanOut->y, meanOut->z);
    // printf("agv x %f %f %f \n", meanOut->x, meanOut->y , meanOut->z);
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
 /*传感器查找偏置值*/
static bool sensorsFindBiasValue(BiasObj *bias)
{
    static int32_t varianceSampleTime;
    bool foundBias = false;

    if (bias->isBufferFilled) {
        sensorsCalculateVarianceAndMean(bias, &bias->variance, &bias->mean);
        // printf("variance: %f %f %f\n",
        // bias->variance.x,
        // bias->variance.y,
        // bias->variance.z);
        if (bias->variance.x < GYRO_VARIANCE_THRESHOLD_X &&
			bias->variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
			bias->variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
			
			(varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < xTaskGetTickCount())) {
				
            varianceSampleTime = xTaskGetTickCount();
            bias->bias.x = bias->mean.x;
            bias->bias.y = bias->mean.y;
            bias->bias.z = bias->mean.z;
            foundBias = true;
            bias->isBiasValueFound = true;
			
			isprintf = 0;
        }
		readvariance.x = bias->variance.x;
		readvariance.y = bias->variance.y;
		readvariance.z = bias->variance.z;
		//printf("x:%0.2f,y:%0.2f,z:%0.2f\r\n",bias->variance.x,bias->variance.y,bias->variance.z);
		
		// ESP_LOGE(TAG,"x:%0.2f,y:%0.2f,z:%0.2f\r\n",bias->variance.x,bias->variance.y,bias->variance.z);
		
		// if(isprintf)
		// {
			// memset(peintf_buf, '\0', 100);
			// sprintf(peintf_buf,"x:%0.2f,y:%0.2f,z:%0.2f\r\n",bias->variance.x,bias->variance.y,bias->variance.z);
			// debugpeintf(peintf_buf);
		// }
    }

    return foundBias;
}




/**
 * 往方差缓冲区（循环缓冲区）添加一个新值，缓冲区满后，替换旧的的值
 */
static void sensorsAddBiasValue(BiasObj *bias, int16_t x, int16_t y, int16_t z)
{
    bias->bufHead->x = x;
    bias->bufHead->y = y;
    bias->bufHead->z = z;
    bias->bufHead++;

    if (bias->bufHead >= &bias->buffer[SENSORS_NBR_OF_BIAS_SAMPLES]) {
        bias->bufHead = bias->buffer;
        bias->isBufferFilled = true;
    }
}

#ifdef GYRO_BIAS_LIGHT_WEIGHT
//计算陀螺方差
/**
 * Calculates the bias out of the first SENSORS_BIAS_SAMPLES gathered. Requires no buffer
 * but needs platform to be stable during startup.
 */
static bool processGyroBiasNoBuffer(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    static uint32_t gyroBiasSampleCount = 0;
    static bool gyroBiasNoBuffFound = false;
    static Axis3i64 gyroBiasSampleSum;
    static Axis3i64 gyroBiasSampleSumSquares;

    if (!gyroBiasNoBuffFound) {
        // If the gyro has not yet been calibrated:
        // Add the current sample to the running mean and variance
        gyroBiasSampleSum.x += gx;
        gyroBiasSampleSum.y += gy;
        gyroBiasSampleSum.z += gz;
#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
        gyroBiasSampleSumSquares.x += gx * gx;
        gyroBiasSampleSumSquares.y += gy * gy;
        gyroBiasSampleSumSquares.z += gz * gz;
#endif
        gyroBiasSampleCount += 1;

        // If we then have enough samples, calculate the mean and standard deviation
        if (gyroBiasSampleCount == SENSORS_BIAS_SAMPLES) {
            gyroBiasOut->x = (float)(gyroBiasSampleSum.x) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->y = (float)(gyroBiasSampleSum.y) / SENSORS_BIAS_SAMPLES;
            gyroBiasOut->z = (float)(gyroBiasSampleSum.z) / SENSORS_BIAS_SAMPLES;

#ifdef SENSORS_GYRO_BIAS_CALCULATE_STDDEV
            gyroBiasStdDev.x = sqrtf((float)(gyroBiasSampleSumSquares.x) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->x * gyroBiasOut->x));
            gyroBiasStdDev.y = sqrtf((float)(gyroBiasSampleSumSquares.y) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->y * gyroBiasOut->y));
            gyroBiasStdDev.z = sqrtf((float)(gyroBiasSampleSumSquares.z) / SENSORS_BIAS_SAMPLES - (gyroBiasOut->z * gyroBiasOut->z));
#endif
            gyroBiasNoBuffFound = true;
        }
    }

    return gyroBiasNoBuffFound;
}
#else
/**
 * Calculates the bias first when the gyro variance is below threshold. Requires a buffer
 * but calibrates platform first when it is stable.
 */
static bool processGyroBias(int16_t gx, int16_t gy, int16_t gz, Axis3f *gyroBiasOut)
{
    sensorsAddBiasValue(&gyroBiasRunning, gx, gy, gz);

    if (!gyroBiasRunning.isBiasValueFound) {
        sensorsFindBiasValue(&gyroBiasRunning);

        // if (gyroBiasRunning.isBiasValueFound) {
            // soundSetEffect(SND_CALIB);
            // ledseqRun(&seq_calibrated);
            // DEBUG_PRINTI("isBiasValueFound!");
        // }
    }

    gyroBiasOut->x = gyroBiasRunning.bias.x;
    gyroBiasOut->y = gyroBiasRunning.bias.y;
    gyroBiasOut->z = gyroBiasRunning.bias.z;

    return gyroBiasRunning.isBiasValueFound;
}
#endif

#define SENSORS_ACC_SCALE_SAMPLES 200 /* 加速计采样个数 */
/**
 * Calculates accelerometer scale out of SENSORS_ACC_SCALE_SAMPLES samples. Should be called when
 * platform is stable.
 */
 //根据样本计算重力加速度缩放因子
static bool processAccScale(int16_t ax, int16_t ay, int16_t az)
{
    static bool accBiasFound = false;
    static uint32_t accScaleSumCount = 0;

    if (!accBiasFound) {
        accScaleSum += sqrtf(powf(ax * SENSORS_G_PER_LSB_CFG, 2) + powf(ay * SENSORS_G_PER_LSB_CFG, 2) + powf(az * SENSORS_G_PER_LSB_CFG, 2));
        accScaleSumCount++;

        if (accScaleSumCount == SENSORS_ACC_SCALE_SAMPLES) {
            accScale = accScaleSum / SENSORS_ACC_SCALE_SAMPLES;
            accBiasFound = true;
        }
    }

    return accBiasFound;
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
 #if 0
static void sensorsAccAlignToGravity(Axis3f *in, Axis3f *out)
{
    Axis3f rx;
    Axis3f ry;

    // Rotate around x-axis
    rx.x = in->x;
    rx.y = in->y * cosRoll - in->z * sinRoll;
    rx.z = in->y * sinRoll + in->z * cosRoll;

    // Rotate around y-axis
    ry.x = rx.x * cosPitch - rx.z * sinPitch;
    ry.y = rx.y;
    ry.z = -rx.x * sinPitch + rx.z * cosPitch;

    out->x = ry.x;
    out->y = ry.y;
    out->z = ry.z;
}
#endif


static void applyAxis3fLpf(lpf2pData *data, Axis3f *in)
{
    for (uint8_t i = 0; i < 3; i++) {
        in->axis[i] = lpf2pApply(&data[i], in->axis[i]);
    }
}


int flat = 0;
/*处理加速计和陀螺仪数据*/
void processAccGyroMeasurements(const uint8_t *buffer)
{
    /*  Note the ordering to correct the rotated 90ยบ IMU coordinate system */

    Axis3f accScaled;

//8658: L H  6050:H L  
#ifdef CONFIG_TARGET_ESPLANE_V1
    /* sensors step 2.1 read from buffer */
    accelRaw.x = (((int16_t)buffer[0]) << 8) | buffer[1];
    accelRaw.y = (((int16_t)buffer[2]) << 8) | buffer[3];
    accelRaw.z = (((int16_t)buffer[4]) << 8) | buffer[5];
    gyroRaw.x = (((int16_t)buffer[8]) << 8) | buffer[9];
    gyroRaw.y = (((int16_t)buffer[10]) << 8) | buffer[11];
    gyroRaw.z = (((int16_t)buffer[12]) << 8) | buffer[13];
#else
    /* sensors step 2.1 read from buffer */
    accelRaw.y = (((int16_t)buffer[1]) << 8) | buffer[0];
    accelRaw.x = (((int16_t)buffer[3]) << 8) | buffer[2];
    accelRaw.z = (((int16_t)buffer[5]) << 8) | buffer[4];
    gyroRaw.y = (((int16_t)buffer[7]) << 8) | buffer[6];
    gyroRaw.x = (((int16_t)buffer[9]) << 8) | buffer[8];
    gyroRaw.z = (((int16_t)buffer[11]) << 8) | buffer[10];
#endif

#ifdef GYRO_BIAS_LIGHT_WEIGHT
    gyroBiasFound = processGyroBiasNoBuffer(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#else
    /* sensors step 2.2 Calculates the gyro bias first when the  variance is below threshold */
    gyroBiasFound = processGyroBias(gyroRaw.x, gyroRaw.y, gyroRaw.z, &gyroBias);
#endif

    /*sensors step 2.3 Calculates the acc scale when platform is steady */
    if (gyroBiasFound) {
        processAccScale(accelRaw.x, accelRaw.y, accelRaw.z);
    }

    /* sensors step 2.4 convert  digtal value to physical angle */
#ifdef CONFIG_TARGET_ESPLANE_V1
    sensorData.gyro.x = (gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#else
    sensorData.gyro.x = -(gyroRaw.x - gyroBias.x) * SENSORS_DEG_PER_LSB_CFG;
#endif

    sensorData.gyro.y = (gyroRaw.y - gyroBias.y) * SENSORS_DEG_PER_LSB_CFG;/*单位 °/s */
    sensorData.gyro.z = (gyroRaw.z - gyroBias.z) * SENSORS_DEG_PER_LSB_CFG;

    /* sensors step 2.5 low pass filter */
    applyAxis3fLpf((lpf2pData *)(&gyroLpf), &sensorData.gyro);

#ifdef CONFIG_TARGET_ESPLANE_V1
    accScaled.x = (accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale; /*单位 g(9.8m/s^2)*/
#else
    accScaled.x = -(accelRaw.x) * SENSORS_G_PER_LSB_CFG / accScale;   
#endif

    accScaled.y = (accelRaw.y) * SENSORS_G_PER_LSB_CFG / accScale;
    accScaled.z = (accelRaw.z) * SENSORS_G_PER_LSB_CFG / accScale;

    /* sensors step 2.6 Compensate for a miss-aligned accelerometer. */
	#if 0
    sensorsAccAlignToGravity(&accScaled, &sensorData.acc);
	#else
	sensorData.acc.x = accScaled.x;	/*单位 g(9.8m/s^2)*/
	sensorData.acc.y = accScaled.y;	/*重力加速度缩放因子accScale 根据样本计算得出*/
	sensorData.acc.z = accScaled.z;
	#endif

    applyAxis3fLpf((lpf2pData *)(&accLpf), &sensorData.acc);

}


static TaskHandle_t sensors_handle = NULL;
#include "debug.h"
static void sensorsTask(void *param)
{
    vTaskDelay(M2T(200));

    ESP_LOGI(TAG,"xTaskCreate sensorsTask SetupSlave done\n");
    uint8_t buf1[12] = {0};
    uint8_t buf2[7] ={0};
    uint8_t buf3[7] = {0};
    static uint32_t last = 0;
    // uint8_t buffer[512] = {0};
    // //FIFO MODE
    // qmi8658WriteByte(0x13, 0x0c);
    // qmi8658WriteByte(0x14, 0x01);  //1000 0010
    static int16_t conut3 = 0;
    while (1)
    {

        uint32_t now = xTaskGetTickCount();

        // printf("imu dt=%lu\n", now - last);

        last = now;
        if(pdTRUE == xSemaphoreTake(sensorsDataReady, portMAX_DELAY)  ){
            
            sensorData.interruptTimestamp = imuIntTimestamp;
            
            // uint8_t read = qmi8658ReadByte(QMI8658_STATUS0);


 
            // uint16_t fifo_count;
            // qmi8658WriteByte(0x14, 0x82); // Bit7 = 1 (RD_MODE)
            // uint8_t cnt[2];
            // qmi8658Read(0x15,2,cnt);
            // fifo_count = (cnt[0] << 8) | cnt[1];

            // uint16_t read_bytes = fifo_count * 12;

            // qmi8658Read(0x17, read_bytes, buffer);

            // printf("test\n");
            // for(int i = 0; i < 6; i++)
            // {
            //     int  temp = i * 12; //0 ~ 11 :12 ~ 23 : 24 ~ 35: 36 ~ 47 : 48 ~ 59 : 60 ~ 71
            //     accelRaw.y = (((int16_t)buffer[temp + 1]) << 8) | buffer[temp + 0];
            //     accelRaw.x = (((int16_t)buffer[temp + 3]) << 8) | buffer[temp + 2];
            //     accelRaw.z = (((int16_t)buffer[temp + 5]) << 8) | buffer[temp + 4];
            //     gyroRaw.y = (((int16_t)buffer[temp + 7]) << 8)  | buffer[temp + 6];
            //     gyroRaw.x = (((int16_t)buffer[temp + 9]) << 8)  | buffer[temp + 8];
            //     gyroRaw.z = (((int16_t)buffer[temp + 11]) << 8) | buffer[temp + 10];

            //     printf("gyro:[%d] %d %d %d\n",++flat, gyroRaw.x, gyroRaw.y, gyroRaw.z);
            // }
            // qmc5883pRead(0x09, 1, buf2);
            // qmc5883pRead(0x01, 6, buf2 + 1);

            // spl06Read(SPL06_MODE_CFG_REG, 1 ,buf3);
            // spl06Read(SPL06_PRESSURE_MSB_REG, 6, buf3);

            qmi8658Read(0x35, 12, buf1);
            processAccGyroMeasurements(buf1);


            // DBG_EVERY(qmi_raw_log, 400, "RAW acc[%d %d %d] gyro[%d %d %d] scaled_acc[%.3f %.3f %.3f] scaled_gyro[%.2f %.2f %.2f]",
            //     accelRaw.x, accelRaw.y, accelRaw.z,
            //     gyroRaw.x, gyroRaw.y, gyroRaw.z,
            //     sensorData.acc.x, sensorData.acc.y, sensorData.acc.z,
            //     sensorData.gyro.x, sensorData.gyro.y, sensorData.gyro.z);

            if(isMagnetometerPresent && gyroBiasRunning.isBiasValueFound){

                qmc5883pRead(0x09, 1, buf2);
                qmc5883pRead(0x01, 6, buf2 + 1);
                processMagnetometerMeasurements(buf2);
            }

            if(isBarometerPresent  && gyroBiasRunning.isBiasValueFound){
                
                #if MICROPY_HW_SPA06 || MICROPY_HW_SPA06_V1
                spa06Read(SPA06_MODE_CFG_REG, 1 ,buf3);
                // spl06Read(SPL06_PRESSURE_MSB_REG, 6, buf3 + 1);
                // processBarometerMeasurements(buf3);
                if((*buf3 & 0x30) == 0x30 ){
                    spa06Read(SPA06_PRESSURE_MSB_REG, 6, buf3 + 1);
                    processBarometerMeasurements(buf3);
                    // xQueueOverwrite(barometerDataQueue, &sensorData.baro);
                }
                #else
                spl06Read(SPL06_MODE_CFG_REG, 1 ,buf3);
                // spl06Read(SPL06_PRESSURE_MSB_REG, 6, buf3 + 1);
                // processBarometerMeasurements(buf3);
                if((*buf3 & 0x30) == 0x30 ){
                    spl06Read(SPL06_PRESSURE_MSB_REG, 6, buf3 + 1);
                    processBarometerMeasurements(buf3);
                    // xQueueOverwrite(barometerDataQueue, &sensorData.baro);
                }
                #endif
            }

            // DBG_EVERY(baro_log, 500,
            // "BARO pressure=%.2f temp=%.2f asl=%.2f",
            // sensorData.baro.pressure,
            // sensorData.baro.temperature,
            // sensorData.baro.asl);

            vTaskSuspendAll();

            xQueueOverwrite(accelerometerDataQueue, &sensorData.acc);
            xQueueOverwrite(gyroDataQueue, &sensorData.gyro);

            if(isMagnetometerPresent){
                xQueueOverwrite(magnetometerDataQueue, &sensorData.mag);
            }

            if(isBarometerPresent){
                xQueueOverwrite(barometerDataQueue, &sensorData.baro);
            }

            xTaskResumeAll();
            
            xSemaphoreGive(dataReady);

            // qmi8658WriteByte(0x14, 0x03);
            // qmi8658WriteByte(0x14, 0x01);
            // printf("sensors ready\n");
            // printf("g: %f %f %f  a: %f %f %f\n",
            // sensorData.gyro.x,
            // sensorData.gyro.y,
            // sensorData.gyro.z,
            // sensorData.acc.x,
            // sensorData.acc.y,
            // sensorData.acc.z);

        }


    }
    
}

static void sensorsTaskInit(void)
{
	accelerometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	gyroDataQueue = xQueueCreate(1, sizeof(Axis3f));
	magnetometerDataQueue = xQueueCreate(1, sizeof(Axis3f));
	barometerDataQueue = xQueueCreate(1, sizeof(baro_t));

    xTaskCreate(sensorsTask, SENSORS_TASK_NAME, SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, &sensors_handle);

    ESP_LOGI(TAG, "xTaskCreate sensorsTask");
}


//初始化传感器
void sensorsQmi8658Spl06Init(void)
{
    if(isInit){
        return;
    }

    sensorsBiasObjInit(&gyroBiasRunning);

    sensorsDeviceInit();

    sensorsInterruptInit();
    sensorsTaskInit(); //传感器任务
    isInit = true;

}

void readBiasVlue(Axis3f *variance)
{
	*variance = readvariance;
}

void getPressureRawData(float* temp, float* press, float*asl )
{
	*temp = TempRaw;
	*press = PressureRaw;
	*asl = AslRaw;
}

void getSensorRawData(Axis3i16* acc, Axis3i16* gyro, Axis3i16*mag )
{
	*acc = accelRaw;
	*gyro = gyroRaw;
	*mag = magRaw;
}
void setPrintf(uint8_t set)
{
	isprintf = set;
}
#endif 