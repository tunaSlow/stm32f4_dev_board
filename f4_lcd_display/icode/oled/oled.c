/*
 * oled.c
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

#include "oled.h"
#include "CHS_16x16.h"
#include "CHS_24x24.h"
#include "PIC1.h"
#include "ASCII.h"
#include "PIC2.h"
#include "../delay/delay.h"
#include "CAR01.h"
#include "CAR02.h"
#include "CAR03.h"
#include "CAR04.h"
#include "CAR05.h"
#include "CAR06.h"
#include "CAR07.h"
#include "CAR08.h"
#include "CAR09.h"
#include "CAR10.h"
#include "CAR11.h"
#include "CAR12.h"
#include "CAR13.h"
#include "CAR14.h"
#include "CAR15.h"
#include "CAR16.h"
#include "CAR17.h"

void OLED0561_Init (void){//OLED屏开显示初始化
	OLED_DISPLAY_OFF(); //OLED关显示
	OLED_DISPLAY_CLEAR(); //清空屏幕内容
	OLED_DISPLAY_ON(); //OLED屏初始值设置并开显示
}
void OLED_DISPLAY_ON (void){//OLED屏初始值设置并开显示
	uint8_t buf[28]={
	0xae,//0xae:关显示，0xaf:开显示
    0x00,0x10,//开始地址（双字节）
	0xd5,0x80,//显示时钟频率？
	0xa8,0x3f,//复用率？
	0xd3,0x00,//显示偏移？
	0XB0,//写入页位置（0xB0~7）
	0x40,//显示开始线
	0x8d,0x14,//VCC电源
	0xa1,//设置段重新映射？
	0xc8,//COM输出方式？
	0xda,0x12,//COM输出方式？
	0x81,0xff,//对比度，指令：0x81，数据：0~255（255最高）
	0xd9,0xf1,//充电周期？
	0xdb,0x30,//VCC电压输出
	0x20,0x00,//水平寻址设置
	0xa4,//0xa4:正常显示，0xa5:整体点亮
	0xa6,//0xa6:正常显示，0xa7:反色显示
	0xaf//0xae:关显示，0xaf:开显示
	}; //
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,28,1000);
}
void OLED_DISPLAY_OFF (void){//OLED屏关显示
	uint8_t buf[3]={
		0xae,//0xae:关显示，0xaf:开显示
		0x8d,0x10,//VCC电源
	}; //
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,buf,3,1000);
}
void OLED_DISPLAY_LIT (uint8_t x){//OLED屏亮度设置（0~255）
	uint8_t buf=0x81;
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf,1,1000);
	HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&x,1,1000);//亮度值
}
void OLED_DISPLAY_CLEAR(void){//清屏操作
	uint8_t j,t;
	uint8_t buf[2]={0x10,0x00};
	for(t=0xB0;t<0xB8;t++){	//设置起始页地址为0xB0
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&t,1,1000); 	//页地址（从0xB0到0xB7）
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //起始列地址的高4位
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000);	//起始列地址的低4位
		for(j=0;j<132;j++){	//整页内容填充
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000);
 		}
	}
}

//----------------------- 用于英文数字的8*16 ASCII码显示的程序 -------------------------//


//取模大小为8*16，取模方式为“从左到右从上到下”“纵向8点下高位”
void OLED_DISPLAY_0816(uint8_t x,uint8_t y,uint16_t w){//显示8*16尺寸的ASCII字符
	//参数：x字符的页坐标（从0xB0到0xB7），y字符的列坐标（从0到128），w字符的编号
	uint8_t j,t,c=0;//临时变量
	uint8_t buf[4];//临时数组
	y=y+2; //【因OLED屏的内置驱动芯片是从0x02列作为屏上最左一列，所以要加上偏移量】
	for(t=0;t<2;t++){//将2个8×8区块从上到下分2次写入数据
		buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16; //将参数中的行和列数据转换后放入buf数组
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000);//页地址（从0xB0到0xB7）
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000);//起始列地址的高4位
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);//起始列地址的低4位
		for(j=0;j<8;j++){//从左到右将8个字节数据写入8个列
			if(x<10)buf[3]=ASCII_8x16[(w*16)+c-512];//当参数1在0~7行是原色显示
			else buf[3]=~ASCII_8x16[(w*16)+c-512];//10~17是反色显示
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//和ASCII表对应要减512
			c++; //每写入1列数据后c加1，偏移到下一列
		}x++; //每写入8列数据后x加1，偏移到下方一组8×8区块
	}
}
//调用方法：OLED_printf_0816(0,0,"123");// 【当参数1在0~7行是原色显示，10~17是反色显示】
void OLED_printf_0816_ASCII(uint8_t row,uint8_t column,char *fmt, ...){//OLED专用printf函数，显示8*16尺寸ASCII字符
    char buff[17];  //用于存放转换后的数据 [长度]
    uint16_t i=0,r=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>16)i=16;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    while(i != r){//i是长度值，当显示到i之后退出
    	OLED_DISPLAY_0816(row,r*8+column,buff[r]);//显示英文与数字8*16的ASCII码
    	r++;
    }
    va_end(arg_ptr);
}

//----------------------- 用于16*16汉字显示的程序 -------------------------//

#define OLED_GB_16_AllWordNum	100  //索引的最大字数（当实际取模大于总数量时修改，小于100时不用改）

void OLED_DISPLAY_1616(uint8_t x,uint8_t y,uint16_t w){//显示16*16尺寸的汉字或图标（可与8*16的英文字符同一行显示）
	//参数：x汉字的页坐标（从0xB0到0xB7），y汉字的列坐标（从0到63），w汉字内码（用双引号加汉字"洋"表示）
	uint8_t j,t,c=0;
	uint8_t buf[4];
	uint16_t wordNum;//临时存放字数
	y=y+2; //因OLED屏的内置驱动芯片是从0x02列作为屏上最左一列，所以要加上偏移量
	for(wordNum=0;wordNum<OLED_GB_16_AllWordNum;wordNum++){//循环比对所有汉字索引
		if(w/0x100==(uint8_t)OLED_GB_16[wordNum].Index[0] && w%0x100==(uint8_t)OLED_GB_16[wordNum].Index[1]){//将输入的汉字与取模索引比对
			for(t=0;t<2;t++){//循环写入3行
				buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16;
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //页地址（从0xB0到0xB7）
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //起始列地址的高4位
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//起始列地址的低4位
				for(j=0;j<16;j++){ //列内容填充
					if(x<10)buf[3]= OLED_GB_16[wordNum].Msk[c];//当参数1在0~7行是原色显示
					else buf[3]= ~OLED_GB_16[wordNum].Msk[c];//10~17是反色显示
					HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//将数据写入屏幕
					c++;//计数值1
				}
				x++; //页地址加1
			}wordNum = 0xFFFE;//手动使数值超出，以退出循环
		}
	}
}

void OLED_printf_1616_CHS (uint8_t row,uint8_t column,char *fmt, ...){//16*16中文显示的printf函数（可在主函数中调用）
	//参数：行（0~7），列（0~127），汉字内容。调用方法：OLED_printf(0,0,"洋桃电子");
    char buff[17];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
	uint8_t r=0,n=0;
	va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//数据转换
    i=strlen(buff)/2;//得出数据长度
    if(strlen(buff)>16)i=16;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
	while(i != r){//i是长度值，当显示到i之后退出
		OLED_DISPLAY_1616(row,r*16+column,buff[n]*0x100+buff[n+1]);//显示中文16*16
		r++;n+=2;//计数加
    }
    va_end(arg_ptr);
}

//----------------------- 用于24*24汉字显示的程序 -------------------------//

#define OLED_GB_24_AllWordNum		100  //索引的最大字数（当实际取模大于总数量时修改，小于100时不用改）

void OLED_DISPLAY_2424(uint8_t x,uint8_t y,uint16_t w){//显示24*24尺寸的汉字或图标
	//参数：x汉字的页坐标（从0xB0到0xB7），y汉字的列坐标（从0到63），w汉字内码（用双引号加汉字"洋"表示）
	uint8_t j,t,c=0;
	uint8_t buf[4];
	uint16_t wordNum;//临时存放字数
	y=y+2; //因OLED屏的内置驱动芯片是从0x02列作为屏上最左一列，所以要加上偏移量
	for(wordNum=0;wordNum<OLED_GB_24_AllWordNum;wordNum++){//循环比对所有汉字索引
		if(w/0x100 == (uint8_t)OLED_GB_24[wordNum].Index[0] && w%0x100 == (uint8_t)OLED_GB_24[wordNum].Index[1]){//将输入的汉字与取模索引比对
			for(t=0;t<3;t++){//循环写入3行
				buf[0]=0xb0+(x%10); buf[1]=y/16+16; buf[2]=y%16;
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //页地址（从0xB0到0xB7）
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //起始列地址的高4位
				HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//起始列地址的低4位
				for(j=0;j<24;j++){ //列内容填充
					if(x<10)buf[3]= OLED_GB_24[wordNum].Msk[c];//当参数1在0~7行是原色显示
					else buf[3]= ~OLED_GB_24[wordNum].Msk[c];//10~17是反色显示
					HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//将数据写入屏幕
					c++;//计数值1
				}
				x++; //页地址加1
			}
			wordNum = 0xFFFE;//手动使数值超出，以退出循环
		}
	}
}

void OLED_printf_2424_CHS (uint8_t row,uint8_t column,char *fmt, ...){//24*24中文显示的printf函数（可在主函数中调用）
	//参数：行（0~7），列（0~127），汉字内容。调用方法：OLED_printf(0,0,"洋桃电子");
    char buff[17];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
	uint8_t r=0,n=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff, 20,fmt,arg_ptr);//数据转换
    i=strlen(buff);//得出数据长度
    if(strlen(buff)>16)i=16;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
	while(i != r){//i是长度值，当显示到i之后退出
		OLED_DISPLAY_2424(row,r*24+column,buff[n]*0x100+buff[n+1]);//显示中文24*24
		r++;n+=2;//计数加
    }
    va_end(arg_ptr);
}

//----------------------- 用于全局图片显示的程序 -------------------------//（小尺寸图标可用汉字方式显示）

void OLED_BMP_DISPLAY(uint8_t *BMP){ //显示全屏单色BMP图片（参数：图片数组指针）
	uint8_t m,i;
	uint8_t buf[4];
	for(m=0;m<8;m++){//循环执行8行，每次变量m加1，对应8行的显示区域
		buf[0]=0xb0+m; buf[1]=16; buf[2]=2;//计算得出OLED起始位置，行放入buf[0]，列放入buf[1][2]
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[0],1,1000); //页地址0xB0~0xB7
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[1],1,1000); //起始列地址的高4位
		HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,COM,I2C_MEMADD_SIZE_8BIT,&buf[2],1,1000);	//起始列地址的低4位
		for(i=0;i<128;i++){//循环128次，每次i的值加1，对应着全屏的128列的显示区域
			buf[3]=BMP[i+m*128];//在此可以修改图片的数组名
			HAL_I2C_Mem_Write(&hi2c2,OLED0561_ADD,DAT,I2C_MEMADD_SIZE_8BIT,&buf[3],1,1000);//用I2C写入数据
		}
	}
}

void OLED_DISPLAY_PIC1(void){ //显示开机LOGO图片1
	OLED_BMP_DISPLAY((uint8_t *)PIC1);//.h文件数组名对向不同图片
	//（例如：PIC1 指向PIC1.h中的PIC1图片数组）
}

void OLED_DISPLAY_PIC2(void){ //显示开机LOGO图片2
	OLED_BMP_DISPLAY((uint8_t *)PIC2);//.h文件数组名对向不同图片
	//（例如：PIC2 指向PIC2.h中的PIC2图片数组）
}

void OLED_DISPLAY_GIF(uint8_t SPEED){ //显示动画（17帧）参数：刷新速度
	OLED_BMP_DISPLAY((uint8_t *)CAR01);delay_us(SPEED);//显示图片并延时一帧停留时间
	OLED_BMP_DISPLAY((uint8_t *)CAR02);delay_us(SPEED);//延时单位：uS
	OLED_BMP_DISPLAY((uint8_t *)CAR03);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR04);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR05);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR06);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR07);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR08);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR09);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR10);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR11);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR12);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR13);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR14);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR15);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR16);delay_us(SPEED);
	OLED_BMP_DISPLAY((uint8_t *)CAR17);delay_us(SPEED);
}

/*********************************************************************************************
 * 洋桃电子 www.DoYoung.net
*********************************************************************************************/
