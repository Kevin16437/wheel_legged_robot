#include "motor_control_task.h"
#include "chassis_task.h"

MOTOR_STATUS motorStatus = INITIAL_IDLE;
TaskHandle_t Motor_Task_handler;
/* 从CAN总线接收到的数据中解析出算法中电机角度和速度 */
void Motor_Update(MotorCali *motor, float pos, float vel) {
	motor->angle = (((pos / 9.67f) * 2.0f * (float)PI) - motor->offsetAngle) * motor->dir;
	motor->speed = ((vel / 9.67f) * 2.0f * (float)PI) * motor->dir;
}

/* 用户自创建总线发送函数 */
void MotorBusSend(uint8_t busId, uint8_t can_id, uint8_t *data, uint8_t dataSize) {
    if(busId == ROBOT_BUS_ID) {
        while(!((hcan1.Instance->TSR & CAN_TSR_TME0) && (hcan1.Instance->TSR & CAN_TSR_TME1) && (hcan1.Instance->TSR & CAN_TSR_TME2)));
        CAN_Send_StdDataFrame(&hcan1, can_id, data);
    }
}

/* 用户自创建总线消息函数 */
void MotorNotice(uint8_t busId, uint8_t nodeId, MW_CMD_ID cmdId) {
    if(busId == ROBOT_BUS_ID) {
        
    }
}  

/* 电机算法数据初始化 */
void Motor_Init(MotorCali *motor, float offsetAngle, float dir) {
	motor->offsetAngle = offsetAngle;
	motor->dir = dir;
}

/* 电机初始化 */
void Motor_InitAll(void) {
    /* 进入闭环控制状态 */
    for (uint8_t i = 0; i < 3; i++) {
        MWSetAxisState(MWjoint1.busId, MWjoint1.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        MWSetAxisState(MWjoint2.busId, MWjoint2.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        MWSetAxisState(MWwheel3.busId, MWwheel3.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        MWSetAxisState(MWjoint4.busId, MWjoint4.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        MWSetAxisState(MWjoint5.busId, MWjoint5.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        MWSetAxisState(MWwheel6.busId, MWwheel6.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
    }
    
    /* 直接力矩模式 */
    motorStatus = DIRECT_TOQUECONTROL;
#ifdef UP_DEBUG
    /* 算法电机数据初始化 */
    Motor_Init(&leftJoint[0], 1.746f, -1);
	Motor_Init(&leftJoint[1], 1.650f, -1);
	Motor_Init(&leftWheel, 0, -1);
    Motor_Init(&rightJoint[0], -1.274f, 1);
    Motor_Init(&rightJoint[1], -1.141f, 1);
	Motor_Init(&rightWheel, 0, 1);
#else 
    /* 算法初始位置标定 */
    Motor_Init(&leftJoint[0], -0.1079f, -1);
	Motor_Init(&leftJoint[1], 3.5498f, -1);
	Motor_Init(&leftWheel, 0, -1);
	Motor_Init(&rightJoint[0], -0.0609f, 1);
	Motor_Init(&rightJoint[1], -3.0200f, 1);
	Motor_Init(&rightWheel, 0, 1);

#endif
    /* 腿部VMC解算更新任务 */
    xTaskCreate((TaskFunction_t)LegPos_UpdateTask, 
                (const char *)"LegPos_UpdateTask", 
                (uint16_t)512, 
                (void *)NULL, 
                (UBaseType_t)osPriorityHigh, 
                (TaskHandle_t *)&LegPos_UpdateTask_handler);   
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
        if((MWjoint1.motorData->currentState == 8) && (MWjoint2.motorData->currentState == 8) &&
           (MWjoint4.motorData->currentState == 8) && (MWjoint5.motorData->currentState == 8) &&
           (MWwheel3.motorData->currentState == 8) && (MWwheel6.motorData->currentState == 8) && (recAllFlag == 1)) {
            /* 主控制程序更新任务 */
            xTaskCreate((TaskFunction_t)Chassis_Control_Task, 
                        (const char *)"Chassis_Control_Task", 
                        (uint16_t)1024, 
                        (void *)NULL, 
                        (UBaseType_t)osPriorityAboveNormal, 
                        (TaskHandle_t *)&Chassis_Control_Task_handler);
            vTaskDelete(NULL);
        }
        vTaskDelayUntil(&xLastWakeTime, 1); 
    }
}





