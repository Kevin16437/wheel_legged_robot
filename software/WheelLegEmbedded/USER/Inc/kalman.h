/**
 * @file    kalman.h
 * @author  Liu heng
 * @date    27-August-2013
 * @brief   一阶卡尔曼滤波器模块
 * @details
 *   实现过程完全与硬件无关，可直接调用，任意移植。
 *   使用时先定义一个kalman对象，然后调用kalmanCreate()创建一个滤波器
 *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波。
 *
 *          kalman p;
 *          float SersorData;
 *          kalmanCreate(&p,20,200);
 *          while(1)
 *          {
 *             SersorData = sersor();
 *             SersorData = KalmanFilter(&p,SersorData);
 *             printf("%2.2f",SersorData);
 *          }
 *
 *          MPU6050的卡尔曼滤波器参考参数 Q：10 R：400
 */

#ifndef _KALMAN_H
#define _KALMAN_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶卡尔曼滤波器
 */
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
} kalman_filter_t;

/**
  * @brief 初始化一个卡尔曼滤波器
  * @param[out] p 滤波器
  * @param[in] T_Q 系统噪声协方差
  * @param[in] T_R 测量噪声协方差
  */
void kalman_Init(kalman_filter_t *p, float T_Q, float T_R);

/**
  * @brief 卡尔曼滤波器
  * @param[in] p 滤波器
  * @param[in] dat 待滤波信号
  * @retval 滤波后的信号
  */
float Kalman_Filter(kalman_filter_t *p, float dat);

#ifdef __cplusplus
}
#endif

#endif
