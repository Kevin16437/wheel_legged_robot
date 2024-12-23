#ifndef _MOTOR_CONTROL_TASK
#define _MOTOR_CONTROL_TASK

#ifdef __cplusplus
extern "C" {
#endif
    
#include "start_init_task.h"

/* 电机模式状态标志位 */
typedef enum {
	INITIAL_IDLE = 0,
	DIRECT_TOQUECONTROL = 1,
	DIRECT_POSCONTROL = 2,
	DIRECT_VELCONTROL = 3
} MOTOR_STATUS;    

extern TaskHandle_t Motor_Task_handler;
extern MOTOR_STATUS motorStatus;

void Motor_Update(MotorCali *motor, float pos, float vel);
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize);
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId);
void Motor_InitAll(void);
void Motor_Task(void *arg);

#ifdef __cplusplus
}
#endif

#endif
