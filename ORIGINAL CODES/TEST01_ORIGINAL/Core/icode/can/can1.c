/*
 * can.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/can/can1.h" //���ļ�����

#include "main.h"

uint8_t CAN1_RX_BUF[CAN1_REC_LEN];//���ջ���,���CAN1_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
uint16_t CAN1_RX_STA;//����״̬���

CAN_TxHeaderTypeDef     TxMeg;//CAN����������ؽṹ��
CAN_RxHeaderTypeDef     RxMeg;//CAN����������ؽṹ��

void CAN_User_Init(CAN_HandleTypeDef* hcan  )//CAN�����û���ʼ������
{
    CAN_FilterTypeDef  sFilterConfig;
    HAL_StatusTypeDef  HAL_Status;
    TxMeg.IDE = CAN_ID_STD;//��չ֡��ʶ��STD��׼֡/EXT��չ֡��
    TxMeg.RTR = CAN_RTR_DATA;//Զ��֡��ʶ��DATA����֡/REMOTEԶ��֡��
    sFilterConfig.FilterBank = 0;//������0
    sFilterConfig.FilterMode =   CAN_FILTERMODE_IDMASK;//��ΪIDLIST�б�ģʽ/IDMASK����ģʽ
    sFilterConfig.FilterScale =  CAN_FILTERSCALE_32BIT;//������λ���
    sFilterConfig.FilterIdHigh = CAN1_ID_H;//32λ����ID���ã���16λ��
    sFilterConfig.FilterIdLow  = CAN1_ID_L;//32λ����ID���ã���16λ��
    sFilterConfig.FilterMaskIdHigh =  CAN1_MASK_H;//32λ����MASK���ã���16λ��
    sFilterConfig.FilterMaskIdLow  =  CAN1_MASK_L;//32λ����MASK���ã���16λ��
    sFilterConfig.FilterFIFOAssignment =  CAN_RX_FIFO1;//���յ��ı��ķ���FIFO1λ��
    sFilterConfig.FilterActivation =  ENABLE;//ENABLE�����������DISABLE��ֹ������
    sFilterConfig.SlaveStartFilterBank  =  0;//�����������ã�����CAN����ʱ���ã�
    HAL_Status=HAL_CAN_ConfigFilter(hcan,&sFilterConfig);//�����Ͻṹ��������õ�CAN�Ĵ�����
    if(HAL_Status!=HAL_OK){//�жϿ����Ƿ�ɹ�
       //����CAN����ʧ�ܵĴ������д�ڴ˴�
    	USART1_printf("\n\rCAN����ʧ�ܣ�\n\r"); //���ڷ���
    }
    HAL_Status=HAL_CAN_Start(hcan);  //����CAN���߹���
    if(HAL_Status!=HAL_OK){//�жϿ����Ƿ�ɹ�
       //����CAN����ʧ�ܵĴ������д�ڴ˴�
    	USART1_printf("\n\rCAN��ʼ��ʧ�ܣ�\n\r"); //���ڷ���
    }
    //����ʹ��CAN�жϣ���ɾ������4��
    HAL_Status=HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO1_MSG_PENDING);//����CAN�����ж�
    if(HAL_Status!=HAL_OK){
       //����CAN���߹����ж�ʧ�ܵĴ������д�ڴ˴�
    	USART1_printf("\n\rCAN�жϳ�ʼ��ʧ�ܣ�\n\r"); //���ڷ���
    }
}
void  HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)  //���ջص����������������ɸģ�
{
    uint8_t  Data[8];//���ջ�������
    HAL_StatusTypeDef HAL_RetVal;//�ж�״̬��ö��
	HAL_RetVal=HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxMeg,Data);//���������е�����
	if (HAL_OK==HAL_RetVal){//�жϽ����Ƿ�ɹ�
		//���ճɹ�������ݴ������д�ڴ˴�����������Data�����У�
		//����2���ǲ��ü򵥵ļĴ�����Ѱ��ʽ����������ݣ�ÿ��ֻ����1λ����ʵ����Ŀ�еĸ��ӽ��ճ�������б�д��
		CAN1_RX_BUF[0]=Data[0];//�����յ������ݷ��뻺�����飨��ֻ�õ�1�����ݣ�����ֻ���������[0]λ�ã�
		CAN1_RX_STA++;//���ݽ��ձ�־λ��1
	}
}
//CAN�������ݺ�������������������ID���������飬����������ֵ��0�ɹ�HAL_OK��1��������HAL_ERROR��2����ʧ��HAL_BUSY��
//ʾ����CAN1_SendNormalData(&hcan1,0,CAN_buffer,8);//CAN�������ݺ���
uint8_t  CAN1_SendNormalData(CAN_HandleTypeDef* hcan,uint16_t ID,uint8_t *pData,uint16_t  Len)
{
    HAL_StatusTypeDef HAL_RetVal;//�ж�״̬��ö��
    uint16_t SendTimes,SendCNT=0;
    uint8_t  FreeTxNum=0;
    uint32_t CAN_TX_BOX0;
    TxMeg.StdId=ID;
    if(!hcan||!pData||!Len){
    	USART1_printf("\n\rCAN����ʧ�ܣ�\n\r"); //���ڷ���
    	return  HAL_ERROR;//��������������ݡ������κ�һ��Ϊ0�򷵻�ֵΪ1
    }
    SendTimes=Len/8+(Len%8?1:0);
    FreeTxNum=HAL_CAN_GetTxMailboxesFreeLevel(hcan);//�ó��������������
    TxMeg.DLC=8;
    while(SendTimes--){//ѭ���жϷ��������Ƿ����
       if(0==SendTimes){//����������ͽ���
           if(Len%8)TxMeg.DLC=Len%8;//����������8������������
       }
       while(0 == FreeTxNum){
            FreeTxNum = HAL_CAN_GetTxMailboxesFreeLevel(hcan);
        }
//       HAL_Delay(1);//��ʱ��ֹ�ٶȹ��쵼�µķ���ʧ��
       //��ʼ�������ݣ������������������ò��������ݣ�����ţ�
       HAL_RetVal=HAL_CAN_AddTxMessage(hcan,&TxMeg,pData+SendCNT,&CAN_TX_BOX0);
       if(HAL_RetVal!=HAL_OK){
    	   USART1_printf("\n\rCAN����æµ��\n\r"); //���ڷ���
    		   return  HAL_BUSY;//�������ʧ�ܣ��򷵻�ֵΪ2
       }
       SendCNT+=8;
    }
    return HAL_OK;//������ͳɹ�����������ֵΪ0
}
//CAN����ͨ�ţ�ʹ��CAN1������CANר�õ�printf����
//���÷�����CAN1_printf("123"); //��CAN�����ַ�123
void CAN1_printf (char *fmt, ...)
{
    char buff[CAN1_REC_LEN+1];  //���ڴ��ת��������� [����]
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(buff, CAN1_REC_LEN+1, fmt,  arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>CAN1_REC_LEN)i=CAN1_REC_LEN;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    CAN1_SendNormalData(&hcan1,CAN1_HOST_ID,(uint8_t *)buff,i);//CAN�������ݺ�����IDΪ0x12��
    va_end(arg_ptr);
}
