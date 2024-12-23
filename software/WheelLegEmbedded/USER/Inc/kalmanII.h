/**
 * @file    kalmanII.h
 * @author  yao
 * @date    1-May-2020
 * @brief  二阶卡尔曼滤波器模块
 * @note 此模块依赖DSP库
 * @details
 *   初始化示例
 *
 *          kalman_filterII_t kalman_filter = {
 *              .P_data = {2, 0, 0, 2},                       // 协方差矩阵
 *              .A_data = {1, 0.001, 0, 1},                   // 预测矩阵 （采样时间）
 *              .H_data = {2, 0, 0, 1},                       // 传感器测量数据矩阵
 *              .Q_data = {0.1, 0, 0, 0.1},                   // 外部的不确定性（不确定性）
 *              .R_data = {0.1, 0.1, 0.1, 0.1},               // 传感器测量方差（采集数据方差）
 *              };
 */


#ifndef _KALMANII_H
#define _KALMANII_H

#include "RMLibHead.h"

RMLIB_CPP_BEGIN

#if !defined(ARM_MATH_CM4) && !defined(ARM_MATH_CM3) && !defined(ARM_MATH_CM7)
#warning "No DSP library support, kalmanII not enable"
#else

#include "arm_math.h"

/**
 * @brief 二阶卡尔曼滤波器
 */
typedef struct {
    float raw_value;
    float filtered_value[2];
    float xhat_data[2], xhatminus_data[2], z_data[2], Pminus_data[4], K_data[4];
    float P_data[4];
    float AT_data[4], HT_data[4];
    float A_data[4];
    float H_data[4];
    float Q_data[4];
    float R_data[4];

    /**
     * @brief 滤波器核心
     */
    struct kalman_filtercore {
        float raw_value;
        float filtered_value[2];
        arm_matrix_instance_f32 xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K, E;
    } kalman;
} kalman_filterII_t;

/**
 * @brief 初始化二阶卡尔曼滤波器
 * @param[in] I 二阶卡尔曼滤波器
 */
void kalmanII_Init(kalman_filterII_t *I);

/**
 * @brief 滤波器
 * @param[in] I 二阶卡尔曼滤波器
 * @param[in] signal1 信号1
 * @param[in] signal2 信号2
 * @return 滤波后的信号float数组,长度为2
 */
float *KalmanII_Filter(kalman_filterII_t *I, float signal1, float signal2);

RMLIB_CPP_END

#endif

#endif
