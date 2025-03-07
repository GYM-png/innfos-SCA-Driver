#include "my_fdcan.h"

#include "innfos_ne30.h"
#include "global.h"



/****************************************************************************************************************************** */

/**
 * @brief can bus initialization
 * @param can can bus instance
 * @param hcan hal library can handle
 * @return OK or ERROR
 */
uint8_t can_init(Can_t *can, FDCAN_HandleTypeDef* hcan)
{
	if(can == NULL || hcan == NULL)
		return ERROR; 
	can->hcan_t = hcan;
	can->mutex = xSemaphoreCreateMutex();
	can->rx_sema = xSemaphoreCreateBinary();
	HAL_FDCAN_Start(can->hcan_t);
	HAL_FDCAN_ActivateNotification(can->hcan_t,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
	HAL_FDCAN_ActivateNotification(can->hcan_t,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
    FDCAN_FilterTypeDef * can_filter_config = (FDCAN_FilterTypeDef*)pvPortMalloc(sizeof(FDCAN_FilterTypeDef));
	if (can_filter_config == NULL)		
	{
		vPortFree(can_filter_config);
		log_e("内存分配失败 CAN初始化失败 ");
		return ERROR;
	}
	
	can_filter_config->IdType = FDCAN_STANDARD_ID;                      //ID mode  FDCAN_STANDARD_ID
    can_filter_config->FilterIndex = 1;                  				//filter index                   
    can_filter_config->FilterType = FDCAN_FILTER_RANGE;        			//filter type :range, mask
    can_filter_config->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;          //FIFOx   FDCAN_FILTER_TO_RXFIFOx
    can_filter_config->FilterID1 = 0x001;                  				//start id
    can_filter_config->FilterID2 = 0xFFF;                  				//end id
    if(HAL_FDCAN_ConfigFilter(can->hcan_t, can_filter_config)!=HAL_OK) 			//can filter config
	{
		log_e("CAN滤波器配置失败 ");
		Error_Handler();
	}
	vPortFree(can_filter_config);
	return OK;
}

/**
 * @brief can bus send message
 * @param can can bus instance
 * @param id message id
 * @param data message data
 * @param dlc message length
 * @return OK or ERROR
 */
uint8_t can_send(Can_t *can, uint32_t id, uint8_t * data, uint8_t dlc)
{
	uint8_t result = ERROR;
	if(can == NULL || data == NULL)
	    return ERROR;
	FDCAN_TxHeaderTypeDef  tx_header;

	tx_header.Identifier = id;                           	//id
	tx_header.IdType = FDCAN_STANDARD_ID;                  	//??ID
	tx_header.TxFrameType = FDCAN_DATA_FRAME;              	//???
	tx_header.DataLength = dlc;                         	//???? FDCAN_DLC_BYTES_8
	tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;            
	tx_header.BitRateSwitch = FDCAN_BRS_OFF;               //??????
	tx_header.FDFormat = FDCAN_CLASSIC_CAN;                //???CAN??FDCAN_CLASSIC_CAN
	tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;     //?????
	tx_header.MessageMarker = 0;                           
	if(xSemaphoreTake(can->mutex, 100) == pdTRUE)
	{
		result = HAL_FDCAN_AddMessageToTxFifoQ(can->hcan_t, &tx_header, data);
		xSemaphoreGive(can->mutex);
		return result;
	}
	return result;	
}

/**
 * @brief can bus receive message
 * @param can can bus instance
 * @return OK or ERROR 
 */
uint8_t can_receive(Can_t *can)
{
	if(can == NULL)
        return ERROR;

	uint8_t result = ERROR;
	result = HAL_FDCAN_GetRxMessage(can->hcan_t, FDCAN_RX_FIFO0, can->rx_header_t, can->rx_data);
	return result;
}

/********************************IRQ********************************************************* */
/**
 * @brief can bus fifo0 irq callback
 * @param hcan can instacne
 */
 void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan->Instance == FDCAN1)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		can_receive(&fdcan1);
		xSemaphoreGiveFromISR(fdcan1.rx_sema, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}

}
