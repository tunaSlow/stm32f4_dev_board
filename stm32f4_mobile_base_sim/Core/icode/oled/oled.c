/*
 * oled.c
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

#include "../../Core/icode/oled/oled.h"

#include "../../Core/icode/oled/ASCII.h"
#include "../../Core/icode/oled/CHS_16x16.h"
#include "../../Core/icode/oled/CHS_24x24.h"
#include "../../Core/icode/oled/PIC1.h"

void OLED0561_Init (void){//OLED������ʾ��ʼ��
	OLED_DISPLAY_OFF(); //OLED����ʾ
	OLED_DISPLAY_CLEAR(); //�����Ļ����
	OLED_DISPLAY_ON(); //OLED����ʼֵ���ò�����ʾ
}
void OLED_DISPLAY_ON (void){//OLED����ʼֵ���ò�����ʾ
	uint8_t buf[28]={
	0xae,//0xae:����ʾ��0xaf:����ʾ
    0x00,0x10,//��ʼ��ַ��˫�ֽڣ�
	0xd5,0x80,//��ʾʱ��Ƶ�ʣ�
	0xa8,0x3f,//�����ʣ�
	0xd3,0x00,//��ʾƫ�ƣ�
	0XB0,//д��ҳλ�ã�0xB0~7��
	0x40,//��ʾ��ʼ��
	0x8d,0x14,//VCC��Դ
	0xa1,//���ö�����ӳ�䣿
	0xc8,//COM�����ʽ��
	0xda,0x12,//COM�����ʽ��
	0x81,0xff,//�Աȶȣ�ָ�0x81�����ݣ�0~255��255��ߣ�
	0xd9,0xf1,//������ڣ�
	0xdb,0x30,//VCC��ѹ���
	0x20,0x00,//ˮƽѰַ����
	0xa4,//0xa4:������ʾ��0xa5:�������
	0xa6,//0xa6:������ʾ��0xa7:��ɫ��ʾ
	0xaf//0xae:����ʾ��0xaf:����ʾ
	}; //
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,28,1000);
}
void OLED_DISPLAY_OFF (void){//OLED������ʾ
	uint8_t buf[3]={
		0xae,//0xae:����ʾ��0xaf:����ʾ
		0x8d,0x10,//VCC��Դ
	}; //
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,3,1000);
}
void OLED_DISPLAY_LIT (uint8_t x){//OLED���������ã�0~255��
	uint8_t buf=0x81;
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf,1,1000);
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&x,1,1000);//����ֵ
}
void OLED_DISPLAY_CLEAR(void){//��������
	uint8_t j,t;
	uint8_t buf[2]={0x10,0x00};
	for(t=0xB0;t<0xB8;t++){	//������ʼҳ��ַΪ0xB0
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&t,1,1000); 	//ҳ��ַ����0xB0��0xB7��
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //��ʼ�е�ַ�ĸ�4λ
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000);	//��ʼ�е�ַ�ĵ�4λ
		for(j=0;j<132;j++){	//��ҳ�������
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000);
 		}
	}
}

//----------------------- ����Ӣ�����ֵ�8*16 ASCII����ʾ�ĳ��� -------------------------//

//��ʾӢ��������8*16��ASCII��
//ȡģ��СΪ16*16��ȡģ��ʽΪ�������Ҵ��ϵ��¡�������8���¸�λ��
void OLED_DISPLAY_0816(uint8_t x,uint8_t y,uint16_t w){//��ʾ12*24�ߴ�ĺ��ֻ�ͼ��
	//������x���ֵ�ҳ���꣨��0xB0��0xB7����y���ֵ������꣨��0��63����w���ֵı��
	uint8_t j,t,c=0;
	uint8_t buf[4];
	y=y+2; //��OLED������������оƬ�Ǵ�0x02����Ϊ��������һ�У�����Ҫ����ƫ����
	for(t=0;t<2;t++){
		buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16;
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //ҳ��ַ����0xB0��0xB7��
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //��ʼ�е�ַ�ĸ�4λ
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//��ʼ�е�ַ�ĵ�4λ
		for(j=0;j<8;j++){ //��ҳ�������
			if(x<10)buf[3]=ASCII_8x16[(w*16)+c-512];//������1��0~7����ԭɫ��ʾ
			else buf[3]=~ASCII_8x16[(w*16)+c-512];//10~17�Ƿ�ɫ��ʾ
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//Ϊ�˺�ASII���ӦҪ��512
			c++;}x++; //ҳ��ַ��1
	}
}

//���÷�����OLED_printf_0816(0,0,"123");// ��������1��0~7����ԭɫ��ʾ��10~17�Ƿ�ɫ��ʾ��
void OLED_printf_0816_ASCII(uint8_t row,uint8_t column,char *fmt, ...){//OLEDר�õ�printf��������ʾ8*16��ASCII�룩
    char buff[17];  //���ڴ��ת��������� [����]
    uint16_t i=0,r=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>16)i=16;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
    while(i != r){//i�ǳ���ֵ������ʾ��i֮���˳�
    	OLED_DISPLAY_0816(row,r*8+column,buff[r]);//��ʾӢ��������8*16��ASCII��
    	r++;
    }
    va_end(arg_ptr);
}

//----------------------- ����16*16������ʾ�ĳ��� -------------------------//

#define OLED_GB_16_AllWordNum		100  //�����������������ʵ��ȡģ����������ʱ�޸ģ�С��100ʱ���øģ�

void OLED_DISPLAY_1616(uint8_t x,uint8_t y,uint16_t w){//��ʾ16*16�ߴ�ĺ��ֻ�ͼ�꣨����8*16��Ӣ���ַ�ͬһ����ʾ��
	//������x���ֵ�ҳ���꣨��0xB0��0xB7����y���ֵ������꣨��0��63����w�������루��˫���żӺ���"��"��ʾ��
	uint8_t j,t,c=0;
	uint8_t buf[4];
	uint16_t wordNum;//��ʱ�������
	y=y+2; //��OLED������������оƬ�Ǵ�0x02����Ϊ��������һ�У�����Ҫ����ƫ����
	for(wordNum=0;wordNum<OLED_GB_16_AllWordNum;wordNum++){//ѭ���ȶ����к�������
		if(w/0x100 == (uint8_t)OLED_GB_16[wordNum].Index[0] && w%0x100 == (uint8_t)OLED_GB_16[wordNum].Index[1]){//������ĺ�����ȡģ�����ȶ�
			for(t=0;t<2;t++){//ѭ��д��3��
				buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16;
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //ҳ��ַ����0xB0��0xB7��
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //��ʼ�е�ַ�ĸ�4λ
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//��ʼ�е�ַ�ĵ�4λ
				for(j=0;j<16;j++){ //���������
					if(x<10)buf[3]= OLED_GB_16[wordNum].Msk[c];//������1��0~7����ԭɫ��ʾ
					else buf[3]= ~OLED_GB_16[wordNum].Msk[c];//10~17�Ƿ�ɫ��ʾ
					HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//������д����Ļ
					c++;//����ֵ1
				}
				x++; //ҳ��ַ��1
			}wordNum = 0xFFFE;//�ֶ�ʹ��ֵ���������˳�ѭ��
		}
	}
}

void OLED_printf_1616_CHS (uint8_t row,uint8_t column,char *fmt, ...){//24*24������ʾ��printf�����������������е��ã�
	//�������У�0~7�����У�0~127�����������ݡ����÷�����OLED_printf(0,0,"���ҵ���");
    char buff[17];  //���ڴ��ת��������� [����]
    uint16_t i=0;
	uint8_t r=0,n=0;
	va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//����ת��
    i=strlen(buff)/2;//�ó����ݳ���
    if(strlen(buff)>16)i=16;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
	while(i != r){//i�ǳ���ֵ������ʾ��i֮���˳�
		OLED_DISPLAY_1616(row,r*16+column,buff[n]*0x100+buff[n+1]);//��ʾ����24*24
		r++;n+=2;//������
    }
    va_end(arg_ptr);
}

//----------------------- ����24*24������ʾ�ĳ��� -------------------------//

#define OLED_GB_24_AllWordNum		100  //�����������������ʵ��ȡģ����������ʱ�޸ģ�С��100ʱ���øģ�

void OLED_DISPLAY_2424(uint8_t x,uint8_t y,uint16_t w){//��ʾ24*24�ߴ�ĺ��ֻ�ͼ��
	//������x���ֵ�ҳ���꣨��0xB0��0xB7����y���ֵ������꣨��0��63����w�������루��˫���żӺ���"��"��ʾ��
	uint8_t j,t,c=0;
	uint8_t buf[4];
	uint16_t wordNum;//��ʱ�������
	y=y+2; //��OLED������������оƬ�Ǵ�0x02����Ϊ��������һ�У�����Ҫ����ƫ����
	for(wordNum=0;wordNum<OLED_GB_24_AllWordNum;wordNum++){//ѭ���ȶ����к�������
		if(w/0x100 == (uint8_t)OLED_GB_24[wordNum].Index[0] && w%0x100 == (uint8_t)OLED_GB_24[wordNum].Index[1]){//������ĺ�����ȡģ�����ȶ�
			for(t=0;t<3;t++){//ѭ��д��3��
				buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16;
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //ҳ��ַ����0xB0��0xB7��
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //��ʼ�е�ַ�ĸ�4λ
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//��ʼ�е�ַ�ĵ�4λ
				for(j=0;j<24;j++){ //���������
					if(x<10)buf[3]= OLED_GB_24[wordNum].Msk[c];//������1��0~7����ԭɫ��ʾ
					else buf[3]= ~OLED_GB_24[wordNum].Msk[c];//10~17�Ƿ�ɫ��ʾ
					HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//������д����Ļ
					c++;//����ֵ1
				}
				x++; //ҳ��ַ��1
			}
			wordNum = 0xFFFE;//�ֶ�ʹ��ֵ���������˳�ѭ��
		}
	}
}

void OLED_printf_2424_CHS (uint8_t row,uint8_t column,char *fmt, ...){//24*24������ʾ��printf�����������������е��ã�
	//�������У�0~7�����У�0~127�����������ݡ����÷�����OLED_printf(0,0,"���ҵ���");
    char buff[17];  //���ڴ��ת��������� [����]
    uint16_t i=0;
	uint8_t r=0,n=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//����ת��
    i=strlen(buff);//�ó����ݳ���
    if(strlen(buff)>16)i=16;//������ȴ������ֵ���򳤶ȵ������ֵ��������ֺ��ԣ�
	while(i != r){//i�ǳ���ֵ������ʾ��i֮���˳�
		OLED_DISPLAY_2424(row,r*24+column,buff[n]*0x100+buff[n+1]);//��ʾ����24*24
		r++;n+=2;//������
    }
    va_end(arg_ptr);
}

//----------------------- ����ȫ��ͼƬ��ʾ�ĳ��� -------------------------//��С�ߴ�ͼ����ú��ַ�ʽ��ʾ��

void OLED_BMP_DISPLAY(uint8_t *BMP){ //��ʾȫ����ɫBMPͼƬ��������ͼƬ����ָ�룩
	uint8_t m,i;
	uint8_t buf[4];
	for(m=0;m<8;m++){//
		buf[0]=0xb0+m; buf[1]=16; buf[2]=2;
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //ҳ��ַ����0xB0��0xB7��
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //��ʼ�е�ַ�ĸ�4λ
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//��ʼ�е�ַ�ĵ�4λ
		for(i=0;i<128;i++){//
			buf[3]=BMP[i+m*128];//�ڴ˿����޸�ͼƬ��������
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);
		}
	}
}

void OLED_DISPLAY_PIC1(void){ //��ʾ����LOGOͼƬ
	OLED_BMP_DISPLAY((uint8_t *)PIC1);//����ͼƬ.h�ļ���Ķ�Ӧ����������ͬ��ͼƬ
	//�����磺PIC1 ָ�򿪻�LOGOͼƬ��
}


/*********************************************************************************************
 * �������� www.DoYoung.net
 * ���ҵ��� www.DoYoung.net/YT
*********************************************************************************************/
