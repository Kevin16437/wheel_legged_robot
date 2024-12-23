#include "start_init_task.h"
#include "IMU_task.h"
#include "motor_control_task.h"
#include "chassis_task.h"
#include "remote_task.h"

/* 陀螺仪温度控制PID */
PID imuTempPID = {.Kp = 1500.0f, .Ki = 0.2f, .Kd = 0.0f, .limit = 4400.0f, .maxOut = 4500.0f};

/* 腿部角度PID */
PID legAnglePID = {.Kp = 10.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 0.0f, .maxOut = 5.0f};
PID legAngleSPID = {.Kp = 3.0f, .Ki = 0.01f, .Kd = 0.0f, .limit = 0.0f, .maxOut = 2.0f};

/* Yaw轴PID */
PID yawPID = {.Kp = 15.0f, .Ki = 0.1f, .Kd = 0.0f, .limit = 0.0f, .maxOut = 2.0f};
PID yawSPID = {.Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 0.0f, .maxOut = 0.8f};

/* 腿部长度PID */
PID legLengthPID = {.Kp = 20.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 50.0f, .maxOut = 30.0f};
PID legLengthSPID = {.Kp = 30.0f, .Ki = 0.5f, .Kd = 0.0f, .limit = 50.0f, .maxOut = 50.0f};

/* Roll轴PID */
PID rollPID = {.Kp = 15.0f, .Ki = 0.0f, .Kd = 0.0f, .limit = 0.0f, .maxOut = 3.0f};
PID rollSPID = {.Kp = 20.0f, .Ki = 0.2f, .Kd = 0.0f, .limit = 20.0f, .maxOut = 10.0f};

/* 陀螺仪实际数据 */
bmi088_real_data_t bmi088_real_data;

/* 磁力计实际数据 */
ist8310_real_data_t ist8310_real_data;

/* 陀螺仪解算数据 */
IMUData imuData;

/* 电机数据接收 */
MW_MOTOR_DATA MWjointData1;
MW_MOTOR_DATA MWjointData2;
MW_MOTOR_DATA MWwheelData3;
MW_MOTOR_DATA MWjointData4;
MW_MOTOR_DATA MWjointData5;
MW_MOTOR_DATA MWwheelData6;

/* 电机登记数据 */
MW_MOTOR_ACCESS_INFO MWjoint1 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 1, 
                               .motorData = &MWjointData1, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint2 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 2, 
                               .motorData = &MWjointData2, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWwheel3 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 3, 
                               .motorData = &MWwheelData3, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint4 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 4, 
                               .motorData = &MWjointData4, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWjoint5 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 5, 
                               .motorData = &MWjointData5, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
MW_MOTOR_ACCESS_INFO MWwheel6 = {.busId = ROBOT_BUS_ID, 
                               .nodeId = 6, 
                               .motorData = &MWwheelData6, 
                               .sender = MotorBusSend, 
                               .notifier = MotorNotice};
/**
 * @brief 电机结构体
 * @note leftJoint[0]:左前关节电机, leftJoint[1]:左后关节电机, leftWheel:左车轮电机
 *       rightJoint[0]:右前关节电机, rightJoint[1]:右后关节电机, rightWheel:右车轮电机
 */           
MotorCali leftJoint[2], rightJoint[2], leftWheel, rightWheel;

/* 电机目标量 */
MotorTarget rightAhead, rightBack, rightwheel, leftAhead, leftBack, leftwheel;

/* 控制目标量*/
Target target = {0, 0, 0, 0, 0, 0, LEG_INITLENGTH, 0};

/* 算法状态量变量 */
StateVar stateVar;

/* 左右腿部姿态 */
LegPos leftLegPos, rightLegPos;

/* 陀螺仪卡尔曼滤波器 */
kalman_filterII_t yawKalmanIIFilter = {
     .P_data = {2, 0, 0, 2},        
     .A_data = {1, 0.001, 0, 1},    
     .H_data = {1, 0, 0, 1},        
     .Q_data = {5, 0, 0, 1},    
     .R_data = {100,0,0,1000},
     };
kalman_filterII_t rollKalmanIIFilter = {
     .P_data = {2, 0, 0, 2},        
     .A_data = {1, 0.001, 0, 1},    
     .H_data = {1, 0, 0, 1},        
     .Q_data = {5, 0, 0, 1},    
     .R_data = {100,0,0,1000},
     };
kalman_filterII_t pitchKalmanIIFilter = {
     .P_data = {2, 0, 0, 2},        
     .A_data = {1, 0.001, 0, 1},    
     .H_data = {1, 0, 0, 1},        
     .Q_data = {5, 0, 0, 1},    
     .R_data = {100,0,0,300},
     };    

/* CAN接收二值信号量 */
SemaphoreHandle_t xCanRxSemaphore;
     
/* 初始化任务 */
void StartInitTask(void const * argument) {
    vTaskDelay(500);
    xCanRxSemaphore = xSemaphoreCreateBinary();
    /* 进入临界区 */
    vPortEnterCritical();
    /* 注册电机信息 */
    MWRegisterMotor(MWjoint1);
	MWRegisterMotor(MWjoint2);
	MWRegisterMotor(MWwheel3);
	MWRegisterMotor(MWjoint4);
	MWRegisterMotor(MWjoint5);
	MWRegisterMotor(MWwheel6);
    /* CAN初始化 */
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    /* 计数定时器初始化，测试程序运行时间 */
    HAL_TIM_Base_Start(&htim2);
    /* 陀螺仪加热丝定时器初始化 */
    HAL_TIM_Base_Start(&htim10);
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    /* 遥控器输入捕获 */
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
    /* 卡尔曼滤波初始化 */
    kalmanII_Init(&yawKalmanIIFilter);
    kalmanII_Init(&rollKalmanIIFilter);
    kalmanII_Init(&pitchKalmanIIFilter);
    /* 退出临界区 */
    vPortExitCritical(); 

    vPortEnterCritical();
    /* 陀螺仪任务 */
    xTaskCreate((TaskFunction_t)IMU_task, 
                (const char *)"IMU_task", 
                (uint16_t)1024, 
                (void *)NULL, 
                (UBaseType_t)osPriorityRealtime, 
                (TaskHandle_t *)&IMU_task_handler);
    /* 目标量更新任务 */
    xTaskCreate((TaskFunction_t)Remote_Task, 
                (const char *)"Remote_Task", 
                (uint16_t)128, 
                (void *)NULL, 
                (UBaseType_t)osPriorityHigh, 
                (TaskHandle_t *)&Remote_Task_handler);                 
    vPortExitCritical(); 
    
    /* 电机初始化 */
    Motor_InitAll();
              
}

/* 定义双缓冲区和标志位 */
struct {
    struct {
        float pos;
        float vel;
    } motorCali[6];                 
    uint8_t flags;                  
} buffer[2];

/* 当前使用的缓冲区索引 */
uint8_t currentBufferIndex = 0;     
/* 判断电机是否全部收到数据 */
uint8_t recAllFlag = 0;
/* CAN接收中断回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(hcan->Instance == CAN1) {
        uint32_t canId = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
        MWReceiver(ROBOT_BUS_ID, canId, CAN1_buff);
        uint8_t nodeId = canId >> 5;
        if (motorStatus != INITIAL_IDLE) {
            switch (nodeId) //根据CAN ID更新各电机数据
            {
                case 0x01: 
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWjointData1.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWjointData1.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
                case 0x02:
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWjointData2.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWjointData2.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
                case 0x03:
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWwheelData3.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWwheelData3.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
                case 0x04:
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWjointData4.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWjointData4.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
                case 0x05:
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWjointData5.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWjointData5.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
                case 0x06:
                    buffer[currentBufferIndex].motorCali[nodeId - 1].pos = MWwheelData6.encoderPosEstimate;
                    buffer[currentBufferIndex].motorCali[nodeId - 1].vel = MWwheelData6.encoderVelEstimate;
                    buffer[currentBufferIndex].flags |= (1 << (nodeId - 1)); // 设置对应电机标志位
                    break;
            }
            /* 检查是否已接收到六个电机的数据 */
            if (buffer[currentBufferIndex].flags == 0x3F) { // 0x3F == 6 bits all set (111111)
                Motor_Update(&rightJoint[0], buffer[currentBufferIndex].motorCali[0].pos, buffer[currentBufferIndex].motorCali[0].vel);
                Motor_Update(&rightJoint[1], buffer[currentBufferIndex].motorCali[1].pos, buffer[currentBufferIndex].motorCali[1].vel);
                Motor_Update(&rightWheel, buffer[currentBufferIndex].motorCali[2].pos, buffer[currentBufferIndex].motorCali[2].vel);
                Motor_Update(&leftJoint[1], buffer[currentBufferIndex].motorCali[3].pos, buffer[currentBufferIndex].motorCali[3].vel);
                Motor_Update(&leftJoint[0], buffer[currentBufferIndex].motorCali[4].pos, buffer[currentBufferIndex].motorCali[4].vel);
                Motor_Update(&leftWheel, buffer[currentBufferIndex].motorCali[5].pos, buffer[currentBufferIndex].motorCali[5].vel);
                
                /* 切换到另一个缓冲区，并清空标志位 */
                currentBufferIndex = (currentBufferIndex + 1) % 2;
                buffer[currentBufferIndex].flags = 0;
                
                /* 全部电机收到数据 */
                recAllFlag = 1;
                
                /* 通知主任务进行数据处理 */
                xSemaphoreGiveFromISR(xCanRxSemaphore, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

/******************************** 计算代码段消耗时间所用的时间函数 ********************************/
/* 定时器计数清零函数 */
void timer_time_set(void) {
    __HAL_TIM_SET_COUNTER(&htim2, 0); //PSC 83 ARR 0XFFFFF
}

/* 获取从上次调用timer_time_set()到当前时间之间的毫秒数 */
float timer_milliseconds_elapsed_since(void) {
    uint32_t diff = __HAL_TIM_GET_COUNTER(&htim2);
    return (float)diff / (float)1000.0f; //ms
}

/* 获取从上次调用timer_time_set()到当前时间之间的微秒数 */
uint32_t timer_microseconds_elapsed_since(void) {
    uint32_t diff = __HAL_TIM_GET_COUNTER(&htim2);
    return diff; //us
}

/* DWT微秒延时函数 */
void DWT_Delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * SystemCoreClockMHz;
    while ((DWT->CYCCNT - startTick) < delayTicks);
}


