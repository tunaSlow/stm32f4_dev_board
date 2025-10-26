/*
 * w25qxx.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/w25q128/w25qxx.h"

#include "main.h"


uint16_t W25QXX_TYPE=W25Q128;//Ĭ����W25Q128
//4KbytesΪһ��Sector��16������Ϊ1��Block������Ϊ16M�ֽڹ���128��Block,4096��Sector


uint8_t SPI2_ReadWriteByte(uint8_t TxData){//��SPI2����д��/����һ���ֽڣ�������д����ֽڣ�����ֵ�Ƕ������ֽڣ�
    uint8_t Rxdata;//����һ������Rxdata
     HAL_SPI_TransmitReceive(&hspi2,&TxData,&Rxdata,1,1000);//���ù̼��⺯���շ�����
    return Rxdata;//�����յ�������
}

void W25QXX_CS(uint8_t a){//������ƺ�����0Ϊ�͵�ƽ������ֵΪ�ߵ�ƽ��
    if(a==0)HAL_GPIO_WritePin(W25Q128_CS_GPIO_Port, W25Q128_CS_Pin, GPIO_PIN_RESET);
    else HAL_GPIO_WritePin(W25Q128_CS_GPIO_Port,  W25Q128_CS_Pin, GPIO_PIN_SET);
}

uint8_t W25QXX_Init(void){//��ʼ��SPI FLASHоƬ
    uint8_t temp;//����һ������temp
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    W25QXX_WAKEUP();//����оƬ
    W25QXX_TYPE = W25QXX_ReadID();//��ȡFLASH  ID.
    if(W25QXX_TYPE == W25Q256){//SPI FLASHΪW25Q256ʱ��������Ϊ4�ֽڵ�ַģʽ
       temp = W25QXX_ReadSR(3);//��ȡ״̬�Ĵ���3���жϵ�ַģʽ
       if((temp&0x01)==0){//�������4�ֽڵ�ַģʽ,�����4�ֽڵ�ַģʽ
           W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
           SPI2_ReadWriteByte(W25X_Enable4ByteAddr);//���ͽ���4�ֽڵ�ַģʽָ��
           W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
       }
    }
    if(W25QXX_TYPE==W25Q256||W25QXX_TYPE==W25Q128||W25QXX_TYPE==W25Q64
    ||W25QXX_TYPE==W25Q32||W25QXX_TYPE==W25Q16||W25QXX_TYPE==W25Q80)
    return 0; else return 1;//�������ID�������ͺ��б��е�һ������ʶ��оƬ�ɹ���
}

//״̬�Ĵ���1��BIT7  6   5   4   3   2   1   0
//         SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����) Ĭ��:0x00
//״̬�Ĵ���2��BIT7  6   5   4   3   2   1   0
//         SUS  CMP LB3 LB2 LB1 (R) QE  SRP1
//״̬�Ĵ���3��BIT7      6    5    4   3   2   1   0
//        HOLD/RST  DRV1 DRV0 (R) (R) WPS (R) (R)
//regno:״̬�Ĵ����ţ���:1~3
//����ֵ:״̬�Ĵ���ֵ
uint8_t W25QXX_ReadSR(uint8_t regno){//��ȡW25QXX��״̬�Ĵ�����W25QXX��3��״̬�Ĵ�����
    uint8_t byte=0,command=0;
    switch(regno){
        case 1:command=W25X_ReadStatusReg1;break;//��״̬�Ĵ���1ָ��
        case 2:command=W25X_ReadStatusReg2;break;//��״̬�Ĵ���2ָ��
        case 3:command=W25X_ReadStatusReg3;break;//��״̬�Ĵ���3ָ��
        default:command=W25X_ReadStatusReg1;break;//��״̬�Ĵ���1ָ��
    }
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(command);//���Ͷ�ȡ״̬�Ĵ�������
    byte=SPI2_ReadWriteByte(0Xff);//��ȡһ���ֽ�
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    return byte;//���ر���byte
}

void W25QXX_Write_SR(uint8_t regno,uint8_t  sr){//дW25QXX״̬�Ĵ���
    uint8_t command=0;
    switch(regno){
        case 1:command=W25X_WriteStatusReg1;break;//д״̬�Ĵ���1ָ��
        case 2:command=W25X_WriteStatusReg2;break;//д״̬�Ĵ���2ָ��
        case 3:command=W25X_WriteStatusReg3;break;//д״̬�Ĵ���3ָ��
        default:command=W25X_WriteStatusReg1;break;
    }
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(command);//����дȡ״̬�Ĵ�������
    SPI2_ReadWriteByte(sr);//д��һ���ֽ�
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
}

void W25QXX_Write_Enable(void){//W25QXXдʹ��
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_WriteEnable);//����дʹ��//��WEL��λ
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
}

void W25QXX_Write_Disable(void){//W25QXXд��ֹ
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_WriteDisable);//����д��ָֹ��//��WEL����
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
}

//0XEF13�ͺ�ΪW25Q80
//0XEF14�ͺ�ΪW25Q16
//0XEF15�ͺ�ΪW25Q32
//0XEF16�ͺ�ΪW25Q64
//0XEF17�ͺ�ΪW25Q128��Ŀǰ����2�ſ�����ʹ��128����оƬ��
//0XEF18�ͺ�ΪW25Q256
uint16_t W25QXX_ReadID(void){//��ȡоƬID(��8λ�ǳ��̴���,��8λ��������С)
    uint16_t Temp = 0;
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(0x90);//���Ͷ�ȡID����
    SPI2_ReadWriteByte(0x00);
    SPI2_ReadWriteByte(0x00);
    SPI2_ReadWriteByte(0x00);
    Temp|=SPI2_ReadWriteByte(0xFF)<<8;
    Temp|=SPI2_ReadWriteByte(0xFF);
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    return Temp;
}

//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t* pBuffer,uint32_t  ReadAddr,uint16_t NumByteToRead){//W25QXX������
    uint16_t i;
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_ReadData);//���Ͷ�ȡ����
    if(W25QXX_TYPE==W25Q256){//�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
         SPI2_ReadWriteByte((uint8_t)((ReadAddr)>>24));
    }
     SPI2_ReadWriteByte((uint8_t)((ReadAddr)>>16));//����24bit��ַ
     SPI2_ReadWriteByte((uint8_t)((ReadAddr)>>8));
    SPI2_ReadWriteByte((uint8_t)ReadAddr);
    for(i=0;i<NumByteToRead;i++){
         pBuffer[i]=SPI2_ReadWriteByte(0XFF);//ѭ������
    }
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
}

//һҳ��д������256���ֽڵ�����(0~65535)
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!
void W25QXX_Write_Page(uint8_t*  pBuffer,uint32_t WriteAddr,uint16_t  NumByteToWrite){//W25QXXҳд��
    uint16_t i;
    W25QXX_Write_Enable();//SET WEL
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_PageProgram);//����дҳ����
    if(W25QXX_TYPE==W25Q256){//�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
         SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>24));
    }
     SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>16));//����24bit��ַ
     SPI2_ReadWriteByte((uint8_t)((WriteAddr)>>8));
    SPI2_ReadWriteByte((uint8_t)WriteAddr);
     for(i=0;i<NumByteToWrite;i++)SPI2_ReadWriteByte(pBuffer[i]);//ѭ��д��
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    W25QXX_Wait_Busy();//�ȴ�д�����
}

//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//��ָ����ַ��ʼд��ָ�����ȵ�����,�豣��ַ���ޣ������Զ���ҳ���ܣ�
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
void W25QXX_Write_NoCheck(uint8_t*  pBuffer,uint32_t WriteAddr,uint16_t  NumByteToWrite){//�޼�����W25QXXд������
    uint16_t pageremain;
    pageremain=256-WriteAddr%256; //��ҳʣ����ֽ���
    if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;//������256���ֽ�
    while(1){
       W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
       if(NumByteToWrite==pageremain)break;//д�������
        else{ //NumByteToWrite>pageremain
           pBuffer+=pageremain;
           WriteAddr+=pageremain;
           NumByteToWrite-=pageremain;            //��ȥ�Ѿ�д���˵��ֽ���
           if(NumByteToWrite>256)pageremain=256; //һ�ο���д��256���ֽ�
           else pageremain=NumByteToWrite;     //����256���ֽ���
       }
    }
}

//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
uint8_t W25QXX_BUFFER[4096];
void W25QXX_Write(uint8_t* pBuffer,uint32_t  WriteAddr,uint16_t NumByteToWrite){//��W25QXXд�����ݣ�������������
    uint32_t secpos;
    uint16_t secoff;
    uint16_t secremain;
    uint16_t i;
    uint8_t* W25QXX_BUF;
    W25QXX_BUF=W25QXX_BUFFER;
    secpos=WriteAddr/4096;//������ַ
    secoff=WriteAddr%4096;//�������ڵ�ƫ��
    secremain=4096-secoff;//����ʣ��ռ��С
    if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//������4096���ֽ�
    while(1){
       W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//������������������
       for(i=0;i<secremain;i++){//У������
           if(W25QXX_BUF[secoff+i]!=0XFF)break;//��Ҫ����
       }
       if(i<secremain){//��Ҫ����
           W25QXX_Erase_Sector(secpos);//�����������
           for(i=0;i<secremain;i++){//����
               W25QXX_BUF[i+secoff]=pBuffer[i];
           }
           W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);//д����������
       }else  W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
       if(NumByteToWrite==secremain)break;//д�������
       	 else{//д��δ����
           secpos++;//������ַ��1
           secoff=0;//ƫ��λ��Ϊ0
           pBuffer+=secremain;  //ָ��ƫ��
           WriteAddr+=secremain;//д��ַƫ��
           NumByteToWrite-=secremain;//�ֽ����ݼ�
           if(NumByteToWrite>4096)secremain=4096;//��һ����������д����
           else  secremain=NumByteToWrite;//��һ����������д����
       }
    }
}

void W25QXX_Erase_Chip(void){//ȫƬ�������ȴ�ʱ��ϳ���
    W25QXX_Write_Enable();//SET WEL
    W25QXX_Wait_Busy();//�ȴ�æ״̬
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_ChipErase);//����Ƭ��������
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    W25QXX_Wait_Busy();//�ȴ�оƬ��������
}

//Dst_Addr:Ҫ������������ַ
void W25QXX_Erase_Sector(uint32_t Dst_Addr){//����һ����������ʱ:150ms��
    Dst_Addr*=4096;
    W25QXX_Write_Enable();//SET WEL
    W25QXX_Wait_Busy();
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_SectorErase);//������������ָ��
    if(W25QXX_TYPE==W25Q256){//�����W25Q256�Ļ���ַΪ4�ֽڵģ�Ҫ�������8λ
         SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>24));
    }
    SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>16));//����24bit��ַ
    SPI2_ReadWriteByte((uint8_t)((Dst_Addr)>>8));
    SPI2_ReadWriteByte((uint8_t)Dst_Addr);
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    W25QXX_Wait_Busy();//�ȴ��������
}

void W25QXX_Wait_Busy(void){//�ȴ�����
    while((W25QXX_ReadSR(1)&0x01)==0x01);//�ȴ�BUSYλ���
}

void W25QXX_PowerDown(void){//�������ģʽ
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_PowerDown);//���͵������� 0xB9
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    delay_us(3);//�ȴ�TPD
}

void W25QXX_WAKEUP(void){//����
    W25QXX_CS(0);//0Ƭѡ������1Ƭѡ�ر�
    SPI2_ReadWriteByte(W25X_ReleasePowerDown);//���͵�Դ����ָ�� 0xAB
    W25QXX_CS(1);//0Ƭѡ������1Ƭѡ�ر�
    delay_us(3);//�ȴ�TRES1
}

/*********************************************************************************************
 * ���ҵ��� www.DoYoung.net
 * ���ֳ�����븴�������翪Դ���� ������Ȩ����ϵ���Ǵ���
 * ���ҵ���ԭ��������벿�־�δ������Ȩ �����ɸ���ʹ�� ���ǲ��Դ������κε���
*********************************************************************************************/
