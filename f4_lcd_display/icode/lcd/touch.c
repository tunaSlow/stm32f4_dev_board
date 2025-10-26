/*
 * oled.c
 *
 *  Created on: Jun 11, 2022
 *      Author: Administrator
 */

/*
//杜洋工作室出品
//洋桃系列开发板应用程序
//关注微信公众号：洋桃电子
//洋桃开发板资料下载 www.DoYoung.net
//即可免费看所有教学视频，下载技术资料，技术疑难提问
//更多内容尽在 杜洋工作室主页 www.doyoung.net
*/

#include "touch.h" 

//本驱动程序可支持GT9147和GT911两款芯片（其电路与寄存器地址基本兼容）

uint8_t TOUCH_ID[5]={0};//触摸芯片ID码（TOUCH_Init初始化后可使用，以ASCII码方式的“9147”或“911”）
uint16_t TOUCH_X[TOUCH_MAX];//当前触发时的X坐标
uint16_t TOUCH_Y[TOUCH_MAX];//当前触发时的Y坐标
uint8_t  TOUCH_STA;//触摸屏当前触发状态

const uint16_t GT9xxx_TP_DAT[6]={GT9xxx_TP1,GT9xxx_TP2,GT9xxx_TP3,GT9xxx_TP4,GT9xxx_TP5};//5个触摸点坐标的I2C子地址（包括GT911和GT9xxx地址有错位）

const uint8_t GT9xxx_Config[]={//GT9xxx配置数据（从设置寄存器0x8047地址开始写入，详见《GT9147编程指南》）
0X60,0XE0,0X01,0X20,0X03,0X05,Module_Switch1,0X00,0X02,0X08,0X1E,0X08,0X50,0X3C,0X0F,0X05,0X00,0X00,0XFF,0X67,0X50,
0X00,0X00,0X18,0X1A,0X1E,0X14,0X89,0X28,0X0A,0X30,0X2E,0XBB,0X0A,0X03,0X00,0X00,0X02,0X33,0X1D,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X32,0X00,0X00,0X2A,0X1C,0X5A,0X94,0XC5,0X02,0X07,0X00,0X00,0X00,0XB5,0X1F,0X00,
0X90,0X28,0X00,0X77,0X32,0X00,0X62,0X3F,0X00,0X52,0X50,0X00,0X52,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X0F,0X0F,0X03,0X06,0X10,0X42,
0XF8,0X0F,0X14,0X00,0X00,0X00,0X00,0X1A,0X18,0X16,0X14,0X12,0X10,0X0E,0X0C,0X0A,0X08,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X29,0X28,0X24,0X22,0X20,
0X1F,0X1E,0X1D,0X0E,0X0C,0X0A,0X08,0X06,0X05,0X04,0X02,0X00,0XFF,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,
0X00,0X00,0X00,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF,0XFF
};

void GT9xxx_Write_Config(uint8_t save){//发送GT9xxx_Config配置数据（初始化时使用）（参数：1保存数据到GT9xxx芯片，0不保存）
	uint8_t buf[2];
	uint8_t i=0;
	buf[0]=0;
	buf[1]=save;//控制是否将配置数据保存到GT9147/GT911芯片中（0不保存，1保存）
	for(i=0;i<sizeof(GT9xxx_Config);i++) buf[0]+=GT9xxx_Config[i];//求得配置数据的校验和
    buf[0]=(~buf[0])+1;
	HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,(uint8_t*)GT9xxx_Config,sizeof(GT9xxx_Config),1000);//发送配置数据
	HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_CHKSUM,I2C_MEMADD_SIZE_16BIT,buf,2,1000);//发送校验和结果
}

uint8_t TOUCH_Init(void){//触摸屏初始化（驱动芯片GT9xxx）（返回1成功，0失败）
	uint8_t t;
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(TOUTH_RST_GPIO_Port,TOUTH_RST_Pin, GPIO_PIN_RESET);//向RST复位接口输出10毫秒低电平脉冲
	HAL_Delay(5);
	HAL_GPIO_WritePin(TOUTH_RST_GPIO_Port,TOUTH_RST_Pin, GPIO_PIN_SET);//
	HAL_Delay(10);
	//器件地址设置完成后，将INT设置为高阻输入
	GPIO_InitStruct.Pin = TOUCH_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);//写入IO设置
	HAL_Delay(100);
	TOUCH_STA = HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_PID,I2C_MEMADD_SIZE_16BIT,TOUCH_ID,4,1000);//读出产品ID
	if(strcmp((char*)TOUCH_ID,"9147")==0 || strcmp((char*)TOUCH_ID,"911")==0){//判断ID是"9147"或“911”（ASCII码方式的ID）
		t=0X02;
		HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,&t,1,1000);//写指令控制：复位芯片
		HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_Ver,I2C_MEMADD_SIZE_16BIT,&t,1,1000); //读取配置文件版本号
		if(t<0X60)GT9xxx_Write_Config(1);//判断版本号过小时//写入配置文件（参数：1保存数据到GT9xxx芯片，0不保存）
		HAL_Delay(10);//延时
		t=0;//GT9xxx_COM赋值0：读坐标状态
		HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_COM,I2C_MEMADD_SIZE_16BIT,&t,1,1000);//写入指令控制，进入读坐标的工作状态
		return 1;//返回成功
	}
	return 0;//返回失败
}

void TOUCH_Read(uint8_t dir){//读取触摸屏状态（有触发则改变TOUCH_STA，读出5个触发点坐标在TOUCH_X[]和TOUCH_Y[]数组中）
	//TOUCH_STA低BIT0-BIT3位保存触发点数量，BIT4触摸状态（1触发，0放开），BIT7数据更新状态（1数据已更新，0数据未准备好）
	//参数：竖屏或横屏的方向（详见touch.h文件）
	uint8_t buf[4];
	uint8_t i=0,STA=0;
	if(HAL_GPIO_ReadPin(GPIOG,TOUCH_INT_Pin)==GPIO_PIN_RESET){//读INT接口的电平（必须将Module_Switch1设置为0x35下降沿触发）
		HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_STA,I2C_MEMADD_SIZE_16BIT,&STA,1,1000); //读取触摸屏状态总标志位
		if((STA&0X80) && ((STA&0x0F)<=TOUCH_MAX)){//判断BIT7的就绪标志位是不是1，同时BIT0-3的触发点数量要小于最大点数
			TOUCH_STA = STA;//将总标志位中的数据放入TOUCH_STA全局标志位（可在主函数中使用TOUCH_STA）
			for(i=0;i<5;i++){//写入坐标之前先清0
				TOUCH_X[i]=0;TOUCH_Y[i]=0;//寄存器清0
			}
			for(i=0;i<(STA&0x0F);i++){//循环读出5个坐标值
				HAL_I2C_Mem_Read(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_TP_DAT[i],I2C_MEMADD_SIZE_16BIT,buf,4,1000); //读取XY坐标值
				if(dir==Portrait){//判断是横屏还是竖屏（0是竖屏，1是横屏）
					TOUCH_X[i]=((uint16_t)buf[1]<<8)+buf[0];//写入坐标值到全局X坐标数组（竖屏坐标写入）
					TOUCH_Y[i]=((uint16_t)buf[3]<<8)+buf[2];//写入坐标值到全局Y坐标数组（竖屏坐标写入）
				}else if(dir==Landscape){
					TOUCH_Y[i]=LCD_Width-(((uint16_t)buf[1]<<8)+buf[0]);//写入坐标值到全局X坐标数组（横屏坐标写入）
					TOUCH_X[i]=((uint16_t)buf[3]<<8)+buf[2];//写入坐标值到全局Y坐标数组（横屏坐标写入）
				}else if(dir==Portrait_reversal){
					TOUCH_X[i]=LCD_Width-(((uint16_t)buf[1]<<8)+buf[0]);//写入坐标值到全局X坐标数组（竖屏坐标写入）
					TOUCH_Y[i]=LCD_Height-(((uint16_t)buf[3]<<8)+buf[2]);//写入坐标值到全局Y坐标数组（竖屏坐标写入）
				}else if(dir==Landscape_reversal){
					TOUCH_Y[i]=((uint16_t)buf[1]<<8)+buf[0];//写入坐标值到全局X坐标数组（横屏坐标写入）
					TOUCH_X[i]=LCD_Height-(((uint16_t)buf[3]<<8)+buf[2]);//写入坐标值到全局Y坐标数组（横屏坐标写入）
				}
			}
			i=0;
			HAL_I2C_Mem_Write(&hi2c2,TOUCH_GT9xxx_ADD,GT9xxx_STA,I2C_MEMADD_SIZE_16BIT,&i,1,1000);//清除触发标志
		}
	}
}

/*********************************************************************************************
 * 洋桃电子 www.doyoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/
