/**
 * @file    RMLibHead.h
 * @author  Yao
 * @date    2020年11月4日
 * @brief   RMLib头文件，引用库文件并定义基本返回值，支持实时系统临界区和动态内存函数
 */

/**@mainpage
 * <table>
 * <tr><th>Project  <td>RMLib
 * <tr><th>Author   <td>yao
 * </table>
 * @section Robomaster整合库
 *
 * <h2><a href="http://www.kdrobot.top/git/KDRobot_RM/RM_LIB">Git页面</a></h2>
 *
 * 注意使用的HAL库的版本大于1.19.0,推荐1.24.x
 * Version: 20220508
 *
 * ## 配置方法
 *
 * 添加全局宏定义`WatchDoglength`，并赋值看门狗最大数量启用看门狗</p>
 *
 * <table>
 * <tr><th>CAN底层驱动模块  <td>@see CANDrive.h @see CANDrive.c
 * <tr><th>底盘驱动模块  <td>@see Chassis.h @see Chassis.c
 * <tr><th>任意阶IIR直接II型，二阶节滤波器模块  <td>@see IIRFilter.h @see IIRFilter.c
 * <tr><th>二阶卡尔曼滤波器模块  <td>@see kalmanII.h @see kalmanII.c
 * <tr><th>一阶卡尔曼滤波器模块  <td>@see kalman.h @see kalman.c
 * <tr><th>电机驱动模块  <td>@see motor.h @see motor.c
 * <tr><th>PID模块  <td>@see PID.h @see PID.c
 * <tr><th>斜坡函数模块  <td>@see ramp.h @see ramp.c
 * <tr><th>遥控器模块  <td>@see remote.h @see remote.c
 * <tr><th>队列模块  <td>@see RMQueue.h @see RMQueue.c
 * <tr><th>软件看门狗模块头  <td>@see WatchDog.h @see WatchDog.c
 * <tr><th>虚拟串口通信模块  <td>@see VCOMCOMM.h @see VCOMCOMM.c
 * <tr><th>CRC校验模块  <td>@see CRC.h @see CRC.c
 * <tr><th>串口printf缓冲区模块  <td>@see PrintfDMABuffer.h @see PrintfDMABuffer.c
 * <tr><th>RoboModule直流电机驱动器驱动模块  <td>@see RoboModule_DRV.h @see RoboModule_DRV.c
 * </table>
 */

#ifndef _RMLIBHEAD_H
#define _RMLIBHEAD_H

#ifdef __cplusplus
#define RMLIB_CPP_BEGIN extern "C" {
#define RMLIB_CPP_END }
#else
#define RMLIB_CPP_BEGIN
#define RMLIB_CPP_END
#endif

#define RM_LIB_VERSION 20221007

#include <string.h>
#include <stdlib.h>

RMLIB_CPP_BEGIN

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
    defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F412Rx) || \
    defined(STM32F412Vx) || defined(STM32F413xx) || defined(STM32F423xx)
#include <stm32f4xx.h>
#elif defined(STM32F030x6)|| defined(STM32F030x8)|| defined(STM32F031x6)|| defined(STM32F038xx)|| \
      defined(STM32F042x6)|| defined(STM32F048xx)|| defined(STM32F051x8)|| defined(STM32F058xx)|| \
      defined(STM32F070x6)|| defined(STM32F070xB)|| defined(STM32F071xB)|| defined(STM32F072xB)|| \
      defined(STM32F078xx)|| defined(STM32F091xC)|| defined(STM32F098xx)|| defined(STM32F030xC)
#include <stm32f0xx.h>
#elif defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
      defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) || \
      defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) || \
      defined(STM32F105xC) || defined(STM32F107xC)
#include <stm32f1xx.h>
#elif defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
      defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) || \
      defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) || \
      defined(STM32F378xx) || defined(STM32F398xx)
#include <stm32f3xx.h>
#elif defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
      defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) || \
      defined(STM32H7B0xx) || defined(STM32H7B0xxQ)|| defined(STM32H7A3xx) || defined(STM32H7B3xx) || \
      defined(STM32H7A3xxQ)|| defined(STM32H7B3xxQ)|| defined(STM32H735xx) || defined(STM32H733xx) || \
      defined(STM32H730xx) || defined(STM32H730xxQ)|| defined(STM32H725xx) || defined(STM32H723xx)
#include <stm32h7xx.h>
#endif

#ifdef __USE_RTOS
#include "FreeRTOS.h"

#if __USE_RTOS == 1
#include "cmsis_os.h"
#elif __USE_RTOS == 2
#include "cmsis_os2.h"
#else
#error "__USE_RTOS must be 1 or 2"
#endif

#include "task.h"
#define RMLIB_ENTER_CRITICAL() vPortEnterCritical()
#define RMLIB_EXIT_CRITICAL() vPortExitCritical()
#define RMLIB_MALLOC(SIZE) pvPortMalloc(SIZE)
#define RMLIB_FREE(P) vPortFree(P)

#else //__USE_RTOS

#define RMLIB_ENTER_CRITICAL()
#define RMLIB_EXIT_CRITICAL()
#define RMLIB_MALLOC(SIZE) malloc(SIZE)
#define RMLIB_FREE(P) free(P)

#endif //__USE_RTOS

#else //__UVISION_VERSION
#include "stdint.h"
#define RMLIB_ENTER_CRITICAL()
#define RMLIB_EXIT_CRITICAL()
#define RMLIB_MALLOC(SIZE) malloc(SIZE)
#define RMLIB_FREE(P) free(P)

#endif //__UVISION_VERSION

typedef enum {
    RM_SUCCESS = 1,
    RM_YES = RM_SUCCESS,
    RM_ENABLE = RM_SUCCESS,
    RM_ERROR = 0,
    RM_NO = RM_ERROR,
    RM_DISABLE = RM_ERROR,
} RM_Status;

RMLIB_CPP_END

#endif //_RMLIBHEAD_H
