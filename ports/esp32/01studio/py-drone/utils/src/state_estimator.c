#include "state_estimator.h"
#include "attitude_pid.h"
#include "maths.h"
#include "stabilizer.h"
#include "sensfusion6.h"
#include "axis.h"

#define ACC_LIMIT			(1000.f)/*加速度限幅 单位cm/s/s*/
#define ACC_LIMIT_MAX		(1800.f)/*最大加速度限幅 单位cm/s/s*/
#define VELOCITY_LIMIT		(130.f)	/*速度限幅 单位cm/s*/
#define VELOCITY_LIMIT_MAX	(500.f)	/*最大速度限幅 单位cm/s*/

#define GRAVITY_CMSS 		(980.f)	/*重力加速度 单位cm/s/s*/
#define INAV_ACC_BIAS_ACCEPTANCE_VALUE	(GRAVITY_CMSS * 0.25f)   // Max accepted bias correction of 0.25G - unlikely we are going to be that much off anyway

static float wBaro = 0.38f;			/*气压校正权重*/

static float wAccBias = 0.01f;		/*加速度校正权重*/

static bool isRstHeight = false;	/*复位高度*/
static bool isRstAll = false;		/*复位估测*/

static float fusedHeight;			/*融合高度，起飞点为0*/
static float fusedHeightLpf = 0.f;	/*融合高度，低通*/
static float startBaroAsl = 0.f;	/*起飞点海拔*/


typedef enum {
    BARO_STATE_INIT = 0,    // 刚上电，丢弃前几帧不稳定数据
    BARO_STATE_CONVERGING,  // 正在观察波动，等待数据收敛
	BARO_STATE_LOCKED,
    BARO_STATE_READY,        // 正常工作模式，输出相对高度
} BaroStatus_e;
static BaroStatus_e BaroAslStatus =  BARO_STATE_INIT;
static float lastBaroAsl = 0.0f;
static float diffBaroAsl  = 0.0f;
static int8_t diffCnt = 0;

/*估测系统*/
static estimator_t estimator = 
{
	.vAccDeadband = 4.0f,
	.accBias[0] =  0.0f,
	.accBias[1] =  0.0f,
	.accBias[2] =  0.0f,
	.acc[0] = 0.0f,
	.acc[1] = 0.0f,
	.acc[2] = 0.0f,
	.vel[0] = 0.0f,
	.vel[1] = 0.0f,
	.vel[2] = 0.0f,
	.pos[0] = 0.0f,
	.pos[1] = 0.0f,
	.pos[2] = 0.0f,
};

/* Inertial filter, implementation taken from PX4 implementation by Anton Babushkin <rk3dov@gmail.com> */
static void inavFilterPredict(int axis, float dt, float acc)
{
    estimator.pos[axis] += estimator.vel[axis] * dt + acc * dt * dt / 2.0f; //位移 =  s = Vot + 1/2 a * t 
    estimator.vel[axis] += acc * dt;   //新速度 = Vo + a *t
}
/*位置校正*/
static void inavFilterCorrectPos(int axis, float dt, float e, float w)
{
    float ewdt = e * w * dt;
    estimator.pos[axis] += ewdt;
    estimator.vel[axis] += w * ewdt;
}
#include "debug.h"
void positionEstimate(sensorData_t* sensorData, state_t* state, float dt) 
{	

	static float accLpf[3] = {0.f};		/*加速度低通*/	
	float weight = wBaro;

	// static float lastValidRelZ = 0.0f;
	// static uint16_t baroProtectCnt = 0;

	// float relateHight = sensorData->baro.asl - startBaroAsl;	/*气压相对高度*/

	// float rawRelZ = sensorData->baro.asl - startBaroAsl;
	// float relateHight = rawRelZ;
	// float relStep = rawRelZ - lastValidRelZ;


	// if (baroProtectCnt > 0)
	// {
	// 	baroProtectCnt--;

	// 	// 保护期内：拒绝明显不合理的 baro 跳变
	// 	if (rawRelZ < -20.0f || fabsf(relStep) > 15.0f)
	// 	{
	// 		relateHight = lastValidRelZ;
	// 	}
	// 	else
	// 	{
	// 		lastValidRelZ = rawRelZ;
	// 	}
	// }
	// else
	// {
	// 	lastValidRelZ = rawRelZ;
	// }

	// if (baroProtectCnt > 0 && (rawRelZ < -20.0f || fabsf(rawRelZ - lastValidRelZ) > 15.0f)) {
	// 	printf("BARO_REJECT rawRel=%.2f lastRel=%.2f step=%.2f\r\n",
	// 		rawRelZ, lastValidRelZ, rawRelZ - lastValidRelZ);
	// }
	
	if(BaroAslStatus != BARO_STATE_READY)
	{
		switch (BaroAslStatus)
		{
			case BARO_STATE_INIT:
				lastBaroAsl = sensorData->baro.asl;
				BaroAslStatus = BARO_STATE_CONVERGING;
				diffCnt = 0;
				break;
			case BARO_STATE_CONVERGING:
				diffBaroAsl = sensorData->baro.asl - lastBaroAsl;
				if(ABS(diffBaroAsl) < 10){
					diffCnt++;
				}else{
					BaroAslStatus = BARO_STATE_INIT;
				}
				
				if(diffCnt >= 100){
					BaroAslStatus = BARO_STATE_LOCKED;
				}
				break;
			case BARO_STATE_LOCKED:
				estRstAll();
				break;
			default:
				break;
		}
	}

	float relateHight = sensorData->baro.asl - startBaroAsl;	/*气压相对高度*/

	{
		fusedHeight = relateHight;	/*融合高度*/
	}
	fusedHeightLpf += (fusedHeight - fusedHeightLpf) * 0.1f;	/*融合高度 低通*/

	
	if(isRstHeight)
	{	
		isRstHeight = false;
		
		weight = 0.95f;		/*增加权重，快速调整*/	
		
		startBaroAsl = sensorData->baro.asl;

		estimator.pos[Z] = fusedHeight;

		// baroProtectCnt = 40;
		// lastValidRelZ = 0.0f;

		printf("isRstHeight\n");
	}
	else if(isRstAll)
	{
		isRstAll = false;
		
		accLpf[Z] = 0.f;	
		fusedHeight  = 0.f;
		fusedHeightLpf = 0.f;
		if((getCommanderKeyFlight()==true)||(getCommanderKeyland()==true) || BaroAslStatus == BARO_STATE_LOCKED)
		{
			BaroAslStatus = BARO_STATE_READY;
			startBaroAsl = sensorData->baro.asl;
		}
		estimator.vel[Z] = 0.f;
		estimator.pos[Z] = fusedHeight;
		
		// baroProtectCnt = 80;
		// lastValidRelZ = 0.0f;

	}	

	Axis3f accelBF;

	accelBF.x = sensorData->acc.x * GRAVITY_CMSS - estimator.accBias[X];
	accelBF.y = sensorData->acc.y * GRAVITY_CMSS - estimator.accBias[Y];
	accelBF.z = sensorData->acc.z * GRAVITY_CMSS - estimator.accBias[Z];	
	// DBG_EVERY(board, 250, "saveStartBaroAsl = %f startBaroAsl = %f baro.asl = %f   relateHight = %f fusedHeightLpf = %f ", 
	// 	saveStartBaroAsl, startBaroAsl, sensorData->baro.asl, relateHight, fusedHeightLpf);

	/* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
	imuTransformVectorBodyToEarth(&accelBF);

	float raw_bfz = accelBF.z;
	bool bfz_bad = fabsf(raw_bfz) > 40.0f;   // 先试20
	static int  test = 0;
	test++;

	// if (bfz_bad) {
	// 	accelBF.z = 0.0f;
	// 	printf("[%d]bfz is bad raw=%.2f\r\n", test, raw_bfz);
	// }

	estimator.acc[X] = applyDeadbandf(accelBF.x, estimator.vAccDeadband);
	estimator.acc[Y] = applyDeadbandf(accelBF.y, estimator.vAccDeadband);
	estimator.acc[Z] = applyDeadbandf(accelBF.z, estimator.vAccDeadband);

	// DBG_EVERY(acc_earth_log, 50,
	// 	"ACC_EARTH raw_bfz=%.2f used_bfz=%.2f est_acc_z=%.2f state_acc_z=%.2f",
	// 	raw_bfz,
	// 	accelBF.z,
	// 	estimator.acc[Z],
	// 	state->acc.z);
		

	for(uint8_t i=0; i<3; i++)
		accLpf[i] += (estimator.acc[i] - accLpf[i]) * 0.1f;	/*加速度低通*/
		
	bool isKeyFlightLand = ((getCommanderKeyFlight()==true)||(getCommanderKeyland()==true));	/*定高飞或者降落状态*/
	
	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
	{
		state->acc.x = constrainf(accLpf[X], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		state->acc.y = constrainf(accLpf[Y], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
		state->acc.z = constrainf(accLpf[Z], -ACC_LIMIT, ACC_LIMIT);	/*加速度限幅*/
	}else
	{
		state->acc.x = constrainf(estimator.acc[X], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		state->acc.y = constrainf(estimator.acc[Y], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
		state->acc.z = constrainf(estimator.acc[Z], -ACC_LIMIT_MAX, ACC_LIMIT_MAX);	/*最大加速度限幅*/
	}		
	
	// float errPosZ = fusedHeight - estimator.pos[Z];

	float errPosZ = fusedHeightLpf - estimator.pos[Z];

	/* 位置预估: Z-axis */
	inavFilterPredict(Z, dt, estimator.acc[Z]);
	// inavFilterPredict(Z, dt, 0.0);
	/* 位置校正: Z-axis */
	inavFilterCorrectPos(Z, dt, errPosZ, weight);	

	// static int conut = 30;

	// float eweight = 0;
	// conut--;
	// if(conut == 0){
	// 	eweight = 0.05;
	// }

	// inavFilterCorrectPos(Z, dt, errPosZ, weight);

	/*加速度偏置校正*/
	Axis3f accelBiasCorr = {{ 0, 0, 0}};
	
	accelBiasCorr.z -= errPosZ  * sq(wBaro);
	float accelBiasCorrMagnitudeSq = sq(accelBiasCorr.x) + sq(accelBiasCorr.y) + sq(accelBiasCorr.z);
	if (accelBiasCorrMagnitudeSq < sq(INAV_ACC_BIAS_ACCEPTANCE_VALUE)) 
	{
		/* transform error vector from NEU frame to body frame */
		imuTransformVectorEarthToBody(&accelBiasCorr);

		/* Correct accel bias */
		estimator.accBias[X] += accelBiasCorr.x * wAccBias * dt;
		estimator.accBias[Y] += accelBiasCorr.y * wAccBias * dt;
		estimator.accBias[Z] += accelBiasCorr.z * wAccBias * dt;
	}	

	if(isKeyFlightLand == true)		/*定高飞或者降落状态*/
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT, VELOCITY_LIMIT);	/*速度限幅 VELOCITY_LIMIT*/
	}else
	{
		state->velocity.x = constrainf(estimator.vel[X], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
		state->velocity.y = constrainf(estimator.vel[Y], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
		state->velocity.z = constrainf(estimator.vel[Z], -VELOCITY_LIMIT_MAX, VELOCITY_LIMIT_MAX);	/*最大速度限幅 VELOCITY_LIMIT_MAX*/
	}
	
	state->position.x = estimator.pos[X];
	state->position.y = estimator.pos[Y];
	state->position.z = estimator.pos[Z];	


	// DBG_EVERY(est_z_log, 50,
	// 	"EST baro=%.2f start=%.2f relate=%.2f fused=%.2f posZ=%.2f velZ=%.2f accZ=%.2f errZ=%.2f biasZ=%.2f rstH=%d rstAll=%d",
	// 	sensorData->baro.asl,
	// 	startBaroAsl,
	// 	relateHight,
	// 	fusedHeightLpf,
	// 	estimator.pos[Z],
	// 	estimator.vel[Z],
	// 	estimator.acc[Z],
	// 	errPosZ,
	// 	estimator.accBias[Z],
	// 	isRstHeight,
	// 	isRstAll);

}

/*读取融合高度 单位cm*/	
float getFusedHeight(void)
{
	return fusedHeightLpf;
}

/*复位估测高度*/
void estRstHeight(void)
{
	isRstHeight = true;
}

/*复位所有估测*/
void estRstAll(void)
{
	isRstAll = true;
}


