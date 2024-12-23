/**
 * @file    PID.h
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块头文件
 */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 限幅宏函数
 * @param IN 限幅变量
 * @param MAX 最大值
 * @param MIN 最小值
 */
#define limit(IN, MAX, MIN) \
    if (IN < MIN)           \
        IN = MIN;           \
    if (IN > MAX)           \
        IN = MAX

/**
 * @brief 标准位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float maxOut;       //!<@brief 输出限幅
    float error_now;    //!<@brief 当前误差
    float error_last;   //!<@brief 上一次误差
    float error_inter;  //!<@brief 误差积分
    float pid_out;      //!<@brief PID输出
} PID;

/**
 * @brief 带史密斯预估器的位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float maxOut;       //!<@brief 输出限幅
    float error_now;    //!<@brief 当前误差
    float error_inter;  //!<@brief 误差积分
    float pid_out;      //!<@brief PID输出
} PID_Smis;

/**
 * @brief 增量式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float error_now;    //!<@brief 当前误差
    float error_next;   //!<@brief 上一次误差
    float error_last;   //!<@brief 上上次误差
    float increament;   //!<@brief PID增量
} PID_ADD;

/**
 * @brief 标准位置式PID
 * @param[in] parameter PID参数
 * @param[in] current 实际值
 * @param[in] expected 期望值
 */
void PID_Control(PID *parameter, float current, float expected);

/**
 * @brief 带史密斯预估器的位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 * @param[in] speed 实际速度
 */
void PID_Control_Smis(PID_Smis *parameter, float current, float expected, float speed);

/**
 * @brief 增量式PID
 * @param[in] current 实际值
 * @param[in] expect 期望值
 * @param[in] parameter PID参数
 * @return PID增量
 */
float PID_Increment(PID_ADD *parameter, float current, float expect);

#ifdef __cplusplus
}
#endif

#endif
