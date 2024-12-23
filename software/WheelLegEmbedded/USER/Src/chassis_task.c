#include "chassis_task.h"
#include "motor_control_task.h"

TaskHandle_t Ctrl_TargetUpdateTask_handler;
TaskHandle_t LegPos_UpdateTask_handler;
TaskHandle_t Chassis_Control_Task_handler;
void LegPos_UpdateTask(void const * argument) {
	const float lpfRatio = 0.5f; //低通滤波系数(新值的权重)
	float lastLeftDLength = 0, lastRightDLength = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for(;;)	{
        while (xSemaphoreTake(xCanRxSemaphore, portMAX_DELAY) != pdTRUE); 
        float legPos[2], legSpd[2];
        
        /* 计算左腿位置 */
        leg_pos(leftJoint[1].angle, leftJoint[0].angle, legPos);
        leftLegPos.length = legPos[0];
        leftLegPos.angle = legPos[1];

        /* 计算左腿速度 */
        leg_spd(leftJoint[1].speed, leftJoint[0].speed, leftJoint[1].angle, leftJoint[0].angle, legSpd);
        leftLegPos.dLength = legSpd[0];
        leftLegPos.dAngle = legSpd[1];

        /* 计算左腿腿长加速度 */
        leftLegPos.ddLength = ((leftLegPos.dLength - lastLeftDLength) * 1000 / 4) * lpfRatio + leftLegPos.ddLength * (1 - lpfRatio);
        lastLeftDLength = leftLegPos.dLength;

        /* 计算右腿位置 */
        leg_pos(rightJoint[1].angle, rightJoint[0].angle, legPos);
        rightLegPos.length = legPos[0];
        rightLegPos.angle = legPos[1];

        /* 计算右腿速度 */
        leg_spd(rightJoint[1].speed, rightJoint[0].speed, rightJoint[1].angle, rightJoint[0].angle, legSpd);
        rightLegPos.dLength = legSpd[0];
        rightLegPos.dAngle = legSpd[1];

        /* 计算右腿腿长加速度 */
        rightLegPos.ddLength = ((rightLegPos.dLength - lastRightDLength) * 1000 / 4) * lpfRatio + rightLegPos.ddLength * (1 - lpfRatio);
        lastRightDLength = rightLegPos.dLength;
//        if (Chassis_Control_Task_handler != NULL)
//            xTaskNotifyGive(Chassis_Control_Task_handler);
	}
}

float test_now_bodyAngle = 0.0f;
float test_now_bodySpd = 0.0f;
float test_now_legAngle = 0.0f;
float test_now_legSpd = 0.0f;
float test_now_wheelPos = 0.0f;
float test_now_wheelSpd = 0.0f;
float test_target_wheelPos = 0.0f;
float test_target_wheelSpd = 0.0f;

float test_target_pos = 0.0f;
float test_target_spd = 0.0f;
float test_real_pos = 0.0f;
float test_real_spd = 0.0f;
float test_legLength = 0.0f;
float test_target_legLen = 0.0f;

/* 手动为反馈矩阵和输出叠加一个系数，用于手动优化控制效果 */
float kRatio[2][6] = {{1.0f, 1.0f, 2.5f, 1.0f, 1.0f, 1.0f},
                    {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}};
void Chassis_Control_Task(void const * argument) {
	const float wheelRadius = 0.0625f;  //m，车轮半径
//	const float legMass = 1.491f;       //kg，腿部质量

	TickType_t xLastWakeTime = xTaskGetTickCount();

	float lqrTpRatio = 1.0f, lqrTRatio = 1.0f;

	/* 设定初始目标值 */
	target.rollAngle = 0.0f;
	target.legLength = LEG_INITLENGTH;
	target.speed = 0.0f;
	target.position = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
	for(;;)	{
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		/* 计算状态变量 */
		stateVar.phi = imuData.pitch;
		stateVar.dPhi = imuData.pitchSpd;
		stateVar.x = (leftWheel.angle + rightWheel.angle) / 2 * wheelRadius;
		stateVar.dx = (leftWheel.speed + rightWheel.speed) / 2 * wheelRadius;
		stateVar.theta = (leftLegPos.angle + rightLegPos.angle) / 2 - PI_2 - imuData.pitch;
		stateVar.dTheta = (leftLegPos.dAngle + rightLegPos.dAngle) / 2 - imuData.pitchSpd;
		float legLength = (leftLegPos.length + rightLegPos.length) / 2;
		float dLegLength = (leftLegPos.dLength + rightLegPos.dLength) / 2;
        
        test_now_bodyAngle = stateVar.phi*100.0f;
        test_now_bodySpd   = stateVar.dPhi*100.0f;
        test_now_legAngle  = stateVar.theta*100.0f;
        test_now_legSpd    = stateVar.dTheta*100.0f;
        test_now_wheelPos  = stateVar.x*100.0f;
        test_now_wheelSpd  = stateVar.dx*100.0f;
        test_target_wheelPos = target.position*100.0f;
        test_target_wheelSpd = target.speed*100.0f;
        
        test_legLength = legLength*100.0f;
        test_target_legLen = target.legLength*100.0f;
        
		/* 计算LQR反馈矩阵 */
		float kRes[12] = {0}, k[2][6] = {0};
		lqr_k(legLength, kRes);
		
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 2; j++)
                k[j][i] = kRes[i * 2 + j] * kRatio[j][i];
        }

		/* 准备状态变量 */
		float x[6] = {stateVar.theta, stateVar.dTheta, stateVar.x, stateVar.dx, stateVar.phi, stateVar.dPhi};
		/* 与给定量作差 */
		x[2] -= target.position;
		x[3] -= target.speed;

		/* 矩阵相乘，计算LQR输出 */
		float lqrOutT = k[0][0] * x[0] + k[0][1] * x[1] + k[0][2] * x[2] + k[0][3] * x[3] + k[0][4] * x[4] + k[0][5] * x[5];
		float lqrOutTp = k[1][0] * x[0] + k[1][1] * x[1] + k[1][2] * x[2] + k[1][3] * x[3] + k[1][4] * x[4] + k[1][5] * x[5];
		
		/* 计算yaw轴PID输出 */
		PID_Control(&yawPID, imuData.yaw, target.yawAngle);
        PID_Control(&yawSPID, imuData.yawSpd, yawPID.pid_out);
		
		/* 根据离地状态修改目标腿长，并计算腿长PID输出 */
		PID_Control(&legLengthPID, legLength, target.legLength);
        PID_Control(&legLengthSPID, dLegLength, legLengthPID.pid_out);
        
		/* 计算roll轴PID输出 */
		PID_Control(&rollPID, imuData.roll, target.rollAngle);
        PID_Control(&rollSPID, imuData.rollSpd, rollPID.pid_out);
        
		/* 根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量 */
		float leftForce = legLengthSPID.pid_out + 5.0f - rollSPID.pid_out ;
		float rightForce = legLengthSPID.pid_out + 5.0f + rollSPID.pid_out;
        
		/* 计算左右腿角度差PID输出 */
		PID_Control(&legAnglePID, leftLegPos.angle - rightLegPos.angle, 0);
        PID_Control(&legAngleSPID, leftLegPos.dAngle - rightLegPos.dAngle, legAnglePID.pid_out);
        
        test_target_pos = 0;
        test_real_pos = (leftLegPos.angle - rightLegPos.angle) * 100.0f;
        test_target_spd = legAnglePID.pid_out*100.0f;
        test_real_spd = (leftLegPos.dAngle - rightLegPos.dAngle)*100.0f;
		
		/* 计算髋关节扭矩输出，为LQR输出和左右腿角度差PID输出的叠加 */
 		float leftTp = lqrOutTp * lqrTpRatio - legAngleSPID.pid_out * (leftLegPos.length / LEG_INITLENGTH);
		float rightTp = lqrOutTp * lqrTpRatio + legAngleSPID.pid_out * (rightLegPos.length / LEG_INITLENGTH);

		/* 使用VMC计算各关节电机输出扭矩 */
		float leftJointTorque[2]={0};
		leg_conv(leftForce, leftTp, leftJoint[1].angle, leftJoint[0].angle, leftJointTorque);
		float rightJointTorque[2]={0};
		leg_conv(rightForce, rightTp, rightJoint[1].angle, rightJoint[0].angle, rightJointTorque);

        /* 轮子力矩输出 */
        leftwheel.torque  = -(-lqrOutT * lqrTRatio - yawSPID.pid_out);
        rightwheel.torque = (-lqrOutT * lqrTRatio + yawSPID.pid_out);

        /* 腿部力矩输出 */
		leftBack.torque   = leftJointTorque[1];
		leftAhead.torque  = leftJointTorque[0];
		rightAhead.torque = -rightJointTorque[0];
		rightBack.torque  = -rightJointTorque[1];
		
        /* 输出力矩限幅 */
		limit(leftBack.torque,   TORQUE_LIMIT, -TORQUE_LIMIT);
        limit(leftAhead.torque,  TORQUE_LIMIT, -TORQUE_LIMIT);
        limit(leftwheel.torque,  TORQUE_LIMIT, -TORQUE_LIMIT);
        limit(rightAhead.torque, TORQUE_LIMIT, -TORQUE_LIMIT);
        limit(rightBack.torque,  TORQUE_LIMIT, -TORQUE_LIMIT);
        limit(rightwheel.torque, TORQUE_LIMIT, -TORQUE_LIMIT);
        
        if (target.EstopFlag == 1) {
			leftwheel.torque  = 0;
            rightwheel.torque = 0;
            leftBack.torque   = 0;
            leftAhead.torque  = 0;
            rightAhead.torque = 0;
            rightBack.torque  = 0;
		}
        
        if (MWwheel3.motorData->ErrorStatus.controllerErrorFlag == 4) {
            MWClearErrors(ROBOT_BUS_ID, MWwheel3.nodeId);
            MWSetAxisState(MWwheel3.busId, MWwheel3.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        } 
        else if ( MWwheel6.motorData->ErrorStatus.controllerErrorFlag == 4) {
            MWClearErrors(ROBOT_BUS_ID, MWwheel6.nodeId);
            MWSetAxisState(MWwheel6.busId, MWwheel6.nodeId, MW_AXIS_STATE_CLOSED_LOOP_CONTROL); 
        }
//        #define TEST_RIGHTLEG_NOOUTPUT 
//        #define TEST_LEFTLEG_NOOUTPUT
//        #define TEST_WHEEL_NOOUTPUT 
        
        switch (motorStatus) {
            case DIRECT_VELCONTROL:     // 直接速度控制
                
                break;
            case DIRECT_POSCONTROL:     // 直接位置控制
                
                break;    
            case DIRECT_TOQUECONTROL:   // 直接力矩控制           
            #ifndef TEST_RIGHTLEG_NOOUTPUT
                MWTorqueControl(MWjoint1.busId, MWjoint1.nodeId, rightAhead.torque); 
                MWTorqueControl(MWjoint2.busId, MWjoint2.nodeId, rightBack.torque);  
            #endif

            #ifndef TEST_LEFTLEG_NOOUTPUT
                MWTorqueControl(MWjoint4.busId, MWjoint4.nodeId, leftBack.torque);   
                MWTorqueControl(MWjoint5.busId, MWjoint5.nodeId, leftAhead.torque);  
            #endif
            
            #ifndef TEST_WHEEL_NOOUTPUT
                MWTorqueControl(MWwheel3.busId, MWwheel3.nodeId, rightwheel.torque); 
                MWTorqueControl(MWwheel6.busId, MWwheel6.nodeId, leftwheel.torque);  
            #endif
            
                break;
            default:
                break;
        }
                
		vTaskDelayUntil(&xLastWakeTime, 4); //4ms控制周期
	}
}
