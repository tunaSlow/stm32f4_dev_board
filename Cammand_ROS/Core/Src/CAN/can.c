/*
 * can.c
 *
 *  Created on: May 7, 2025
 *      Author: Vannak
 */

#include "can.h"
#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;
FDCAN_RxHeaderTypeDef RX_CAN_2;
FDCAN_TxHeaderTypeDef TX_CAN_2;

uint8_t tx_data_CAN2_buf[8];
uint8_t rx_data_CAN2_buf[8];

extern int16_t Speed_receiving[3];
extern int16_t Speed_measure[8];

extern int16_t Set_speed_M[8];


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RX_CAN_2, rx_data_CAN2_buf);

    switch (RX_CAN_2.Identifier) {
//Robomaster_data_receving
    case 0x201:
      		Speed_measure[0]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];
      		break;
    case 0x202:
      		Speed_measure[1]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];
      		break;
    case 0x203:
      		Speed_measure[2]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];
      		break;
    case 0x204:
      		Speed_measure[3]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];

      		break;

    case 0x205:
      		Speed_measure[4]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];
      		break;
    case 0x206:
      		Speed_measure[5]= rx_data_CAN2_buf[2]<<8 | rx_data_CAN2_buf[3];
      		break;

//////////////////////////////////

   	default :
   		break;
    };
}


void CAN_tran()
{

    tx_data_CAN2_buf[0] = Set_speed_M[0] >> 8;
    tx_data_CAN2_buf[1] = Set_speed_M[0]; // Motor 1
    tx_data_CAN2_buf[2] = Set_speed_M[1] >> 8;
    tx_data_CAN2_buf[3] = Set_speed_M[1]; // Motor 2
    tx_data_CAN2_buf[4] = Set_speed_M[2] >> 8;
    tx_data_CAN2_buf[5] = Set_speed_M[2]; // Motor 3
    tx_data_CAN2_buf[6] = Set_speed_M[3] >> 8;
    tx_data_CAN2_buf[7] = Set_speed_M[3]; // Motor 4

    TX_CAN_2.Identifier = 0x200;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TX_CAN_2, tx_data_CAN2_buf);

    tx_data_CAN2_buf[0] = Set_speed_M[4] >> 8;
    tx_data_CAN2_buf[1] = Set_speed_M[4]; // Motor 5
    tx_data_CAN2_buf[2] = Set_speed_M[5] >> 8;
    tx_data_CAN2_buf[3] = Set_speed_M[5]; // Motor 6
    tx_data_CAN2_buf[4] = 0;
    tx_data_CAN2_buf[5] = 0;
    tx_data_CAN2_buf[6] = 0;
    tx_data_CAN2_buf[7] = 0;

    TX_CAN_2.Identifier = 0x1FF;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TX_CAN_2, tx_data_CAN2_buf);

    HAL_Delay(2);

}

void CAN_INIT(void){

	   HAL_FDCAN_Start(&hfdcan2);

	   TX_CAN_2.Identifier=0x200;
	   TX_CAN_2.IdType=FDCAN_STANDARD_ID;
	   TX_CAN_2.TxFrameType=FDCAN_DATA_FRAME;
	   TX_CAN_2.DataLength= FDCAN_DLC_BYTES_8;
	   TX_CAN_2.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
	   TX_CAN_2.BitRateSwitch=FDCAN_BRS_OFF;
	   TX_CAN_2.FDFormat=FDCAN_CLASSIC_CAN;
	   TX_CAN_2.TxEventFifoControl= FDCAN_NO_TX_EVENTS;
	   TX_CAN_2.MessageMarker=0;

	   FDCAN_FilterTypeDef sFilterConfig;
	   sFilterConfig.IdType = FDCAN_STANDARD_ID;
	   sFilterConfig.FilterIndex = 0;
	   sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	   sFilterConfig.FilterID1 = 0x200;
	   sFilterConfig.FilterID2 = 0x200;

	   if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	     {
	       /* Filter configuration Error */
	       Error_Handler();

	     }

	    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);
	    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

}




