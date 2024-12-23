/**
 * @file    PID.c
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块
 */

#include "PID.h"

void PID_Control(PID *parameter, float current, float expected) {
    parameter->error_last = parameter->error_now;
    parameter->error_now = expected - current;
    parameter->error_inter += parameter->error_now;
    limit(parameter->error_inter, parameter->limit, -parameter->limit);
    
    parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                         parameter->Kd * (parameter->error_now - parameter->error_last);
    limit(parameter->pid_out, parameter->maxOut, -parameter->maxOut);
}

float PID_Increment(PID_ADD *parameter, float current, float expect) {
    parameter->error_now = expect - current;

    parameter->increament =
            parameter->Kp * (parameter->error_now - parameter->error_next) + parameter->Ki * (parameter->error_now) +
            parameter->Kd * (parameter->error_now - 2 * parameter->error_next + parameter->error_last);

    parameter->error_last = parameter->error_next;
    parameter->error_next = parameter->error_now;

    return parameter->increament;
}

void PID_Control_Smis(PID_Smis *parameter, float current, float expected, float speed) {
    parameter->error_now = expected - current;
    parameter->error_inter += parameter->error_now;
    
    limit(parameter->error_inter, parameter->limit, -parameter->limit);
    
    parameter->pid_out = parameter->Kp * parameter->error_now + parameter->Ki * parameter->error_inter +
                         parameter->Kd * speed;
    limit(parameter->pid_out, parameter->maxOut, -parameter->maxOut);
}
