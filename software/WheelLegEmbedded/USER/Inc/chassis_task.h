#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H
#include "start_init_task.h"
#include "arm_math.h"

extern TaskHandle_t Ctrl_TargetUpdateTask_handler;
extern TaskHandle_t LegPos_UpdateTask_handler;
extern TaskHandle_t Chassis_Control_Task_handler;

void Ctrl_TargetUpdateTask(void const * argument);
void LegPos_UpdateTask(void const * argument);
void Chassis_Control_Task(void const * argument);

#endif
