#ifndef _START_INIT_TASK_H
#define _START_INIT_TASK_H

#include "tim.h"

#include "PID.h"
#include "CANDrive.h"
#include "MWmotor.h"
#include "kalman.h"
#include "kalmanII.h"

#include "leg_pos.h"
#include "lqr_k.h"
#include "leg_conv.h"
#include "leg_spd.h"
#include "ahrs.h"
#include "arm_math.h"

#include "BMI088driver.h"
#include "ist8310driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 获取系统时钟频率，单位是 MHz */
#define SystemCoreClockMHz (SystemCoreClock / 1000000U)
#define PI_2		(PI / 2)

/* 电机输出力矩限幅 */
#define TORQUE_LIMIT 13.0f

//#define UP_DEBUG
#ifdef UP_DEBUG
    #define LEG_INITLENGTH 0.36f
#else
    #define LEG_INITLENGTH 0.15f 
#endif
#define LEG_LENGTHMAX 0.30f

/* 电机总线ID */
#define ROBOT_BUS_ID 1

/* 电机算法结构体 */
typedef struct {
	float speed;			   // rad/s
	float angle, offsetAngle;  // rad
	float dir;				   // 1 or -1
} MotorCali; 

/* 目标量结构体 */
typedef struct {
	float position;	 // m
	float speedCmd;	 // m/s
	float speed;    // m/s
	float yawSpeedCmd; // rad/s
	float yawAngle;	 // rad
	float rollAngle; // rad
	float legLength; // m
    uint8_t EstopFlag;
} Target;

/* 状态变量结构体 */
typedef struct {
	float theta, dTheta;
	float x, dx;
	float phi, dPhi;
} StateVar;

/* 腿部姿态结构体 */
typedef struct {
	float angle, length;   // rad, m
	float dAngle, dLength; // rad/s, m/s
	float ddLength;		   // m/s^2
} LegPos; 

/* 电机目标量结构体 */
typedef struct {
	float vel;
	float pos;
	float torque;
} MotorTarget;

/* 陀螺仪结构体 */
typedef struct IMUData
{
	float yaw, pitch, roll, yawOffset;	// rad
	float yawSpd, pitchSpd, rollSpd;    // rad/s
    int16_t yawRound;
	float zAccel;                       // m/s^2
} IMUData;

extern PID imuTempPID;
extern PID legAnglePID; 
extern PID legAngleSPID;
extern PID legLengthPID;
extern PID legLengthSPID;
extern PID yawPID;
extern PID yawSPID;
extern PID rollPID;
extern PID rollSPID;
    
extern bmi088_real_data_t bmi088_real_data;
extern ist8310_real_data_t ist8310_real_data;
extern IMUData imuData;

extern kalman_filterII_t yawKalmanIIFilter;
extern kalman_filterII_t rollKalmanIIFilter;
extern kalman_filterII_t pitchKalmanIIFilter;
    
extern MW_MOTOR_ACCESS_INFO MWjoint1;
extern MW_MOTOR_ACCESS_INFO MWjoint2;
extern MW_MOTOR_ACCESS_INFO MWwheel3;
extern MW_MOTOR_ACCESS_INFO MWjoint4;
extern MW_MOTOR_ACCESS_INFO MWjoint5;
extern MW_MOTOR_ACCESS_INFO MWwheel6;

extern MotorCali leftJoint[2], rightJoint[2], leftWheel, rightWheel;
extern Target target;
extern StateVar stateVar;
extern LegPos leftLegPos, rightLegPos;
extern MotorTarget rightAhead, rightBack, rightwheel, leftAhead, leftBack, leftwheel;

extern SemaphoreHandle_t xCanRxSemaphore;
extern uint8_t recAllFlag;

void timer_time_set(void);
float timer_milliseconds_elapsed_since(void);
uint32_t timer_microseconds_elapsed_since(void);
void DWT_Delay_us(uint32_t us);
    
#ifdef __cplusplus
}
#endif
    
#endif
