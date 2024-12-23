#include "CANDrive.h"

#ifdef HAL_CAN_MODULE_ENABLED

#if !defined(CAN1) && defined(CAN)
#define CAN1 CAN
#endif

uint8_t CAN1_buff[8];

#if defined(CAN2)
uint8_t CAN2_buff[8];
#endif

void CanFilter_Init(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef canfilter;

    canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilter.FilterScale = CAN_FILTERSCALE_32BIT;

    canfilter.FilterIdHigh = 0x0000;
    canfilter.FilterIdLow = 0x0000;
    canfilter.FilterMaskIdHigh = 0x0000;
    canfilter.FilterMaskIdLow = 0x0000;

    /*! 从can的过滤器起始编号 只有当设置两个can时 该参数才有意义 */
    canfilter.SlaveStartFilterBank = 14;

    /*! can1和CAN2使用不同的滤波器*/
    if (hcan->Instance == CAN1) {
        /*! 主can的过滤器编号 */
        canfilter.FilterBank = 0;

        /*! CAN_FilterFIFO0 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    }

#if defined(CAN2)
    if (hcan->Instance == CAN2) {
        /*! 从can的过滤器编号 */
        canfilter.FilterBank = 14;

        /*! CAN_FilterFIFO1 */
        canfilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    }
#endif

    /*! 激活过滤器 */
    canfilter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(hcan, &canfilter);
}

HAL_StatusTypeDef CAN_Send_StdDataFrame(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *msg) {
    RMLIB_ENTER_CRITICAL();
    CAN_TxHeaderTypeDef CAN_Tx = {
            .StdId = StdId,                 //标准标识符
            .ExtId = 0,
            .IDE = CAN_ID_STD,              //使用标准帧
            .RTR = CAN_RTR_DATA,            //数据帧
            .DLC = 8,
            .TransmitGlobalTime = DISABLE,
            };

    uint32_t TxMailbox = 0;
    HAL_StatusTypeDef err = HAL_CAN_AddTxMessage(hcan, &CAN_Tx, msg, &TxMailbox);
    RMLIB_EXIT_CRITICAL();
    return err;
}

uint32_t CAN_Receive_DataFrame(CAN_HandleTypeDef *hcan, uint8_t *buf) {
    CAN_RxHeaderTypeDef CAN_Rx = { 0 };
    HAL_CAN_GetRxMessage(hcan, (hcan->Instance == CAN1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1, &CAN_Rx, buf);

    if(CAN_Rx.IDE == CAN_ID_STD)
        return CAN_Rx.StdId;
    else
        return CAN_Rx.ExtId;
}

#endif
