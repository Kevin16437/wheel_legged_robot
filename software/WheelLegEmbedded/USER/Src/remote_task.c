#include "remote_task.h"

RemoteRec remoteData = {3000, 3000, 3000, 3000, 3000};

TaskHandle_t Remote_Task_handler;
void Remote_Task(void const * argument) {
    float speedSlopeStep = 0.030f;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;) {
//        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS);
        
        target.speedCmd = (((float)remoteData.channels[1] - 2000.0f) / 1000.0f - 1.0f) * 1.5f;
        target.yawSpeedCmd = (((float)remoteData.channels[0] - 2000.0f) / 1000.0f - 1.0f) * 1.5f;
        target.legLength += (((float)remoteData.channels[2] - 2000.0f) / 1000.0f - 1.0f) * 0.0005f;
        if(remoteData.channels[3] < 3000) target.EstopFlag = 1;
        else target.EstopFlag = 0;
        #ifdef UP_DEBUG
            if (target.legLength < LEG_INITLENGTH - 0.1f) target.legLength  = LEG_INITLENGTH - 0.1f;
            else if (target.legLength > LEG_INITLENGTH + 0.1f) target.legLength  = LEG_INITLENGTH + 0.1f;
        #else 
            if (target.legLength < LEG_INITLENGTH) target.legLength  = LEG_INITLENGTH;
            else if (target.legLength > LEG_LENGTHMAX) target.legLength  = LEG_LENGTHMAX;
        #endif
        
        //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		speedSlopeStep = -(legLength - LEG_INITLENGTH) * 0.1f + 0.03f;

		//计算速度斜坡，斜坡值更新到target.speed
		if(fabs(target.speedCmd - target.speed) < speedSlopeStep)
			target.speed = target.speedCmd;
		else
		{
			if(target.speedCmd - target.speed > 0)
				target.speed += speedSlopeStep;
			else
				target.speed -= speedSlopeStep;
		}

		//计算位置目标，并限制在当前位置的±0.1m内
		target.position += target.speed * 0.004f;
		if(target.position - stateVar.x > 0.1f)
			target.position = stateVar.x + 0.1f; 
		else if(target.position - stateVar.x < -0.1f)
			target.position = stateVar.x - 0.1f;

		//限制速度目标在当前速度的±0.5m/s内
		if(target.speed - stateVar.dx > 0.5f)
			target.speed = stateVar.dx + 0.5f;
		else if(target.speed - stateVar.dx < -0.5f)
			target.speed = stateVar.dx - 0.5f;

		//计算yaw方位角目标
		target.yawAngle -= target.yawSpeedCmd * 0.004f;
        
        vTaskDelayUntil(&xLastWakeTime, 4);
    }
}

// 定义全局变量，用于存储捕获值和计算结果
volatile uint32_t timUp[4] = {0};
volatile uint32_t timDown[4] = {0};
void CaptureAndCalculate(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t chIndex)
{
    // 检查当前极性是否为上升沿
    if ((htim->Instance->CCER & (TIM_CCER_CC1P << (channel >> 2) * 4)) == 0) {
        // 捕获上升沿时间
        timUp[chIndex] = HAL_TIM_ReadCapturedValue(htim, channel);

        // 将通道极性设置为下降沿
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    // 检查当前极性是否为下降沿
    else {
        // 捕获下降沿时间
        timDown[chIndex] = HAL_TIM_ReadCapturedValue(htim, channel);

        // 计算高电平持续时间
        if (timDown[chIndex] > timUp[chIndex])
            remoteData.channels[chIndex] = timDown[chIndex] - timUp[chIndex];
        else
            remoteData.channels[chIndex] = (htim->Instance->ARR - timUp[chIndex] + timDown[chIndex] + 1);
        
        if (remoteData.channels[chIndex] == 3001 || remoteData.channels[chIndex] == 2999)
            remoteData.channels[chIndex] = 3000;
        // 重新设置通道极性为上升沿，以便捕获下一个周期
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
//        
//        static BaseType_t xHigherPriorityTaskWoken;
//        vTaskNotifyGiveFromISR(Remote_Task_handler, &xHigherPriorityTaskWoken);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    // 判断是否为 TIM8 通道 1 或 TIM1 通道 2、3、4 的中断
    if (htim->Instance == TIM8) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_1:
                CaptureAndCalculate(htim, TIM_CHANNEL_1, 0);
                break;
            case HAL_TIM_ACTIVE_CHANNEL_2:
                CaptureAndCalculate(htim, TIM_CHANNEL_2, 4); 
                break;
            default:
                break;
        }
    } 
    else if (htim->Instance == TIM1) {
        switch (htim->Channel) {
            case HAL_TIM_ACTIVE_CHANNEL_2:
                CaptureAndCalculate(htim, TIM_CHANNEL_2, 1); 
                break;
            case HAL_TIM_ACTIVE_CHANNEL_3:
                CaptureAndCalculate(htim, TIM_CHANNEL_3, 2); 
                break;
            case HAL_TIM_ACTIVE_CHANNEL_4:
                CaptureAndCalculate(htim, TIM_CHANNEL_4, 3); 
                break;
            default:
                break;
        }
    }
    else {
        remoteData.channels[0] = 3000;
        remoteData.channels[1] = 3000;
        remoteData.channels[2] = 3000;
        remoteData.channels[3] = 4000;
    }
}



