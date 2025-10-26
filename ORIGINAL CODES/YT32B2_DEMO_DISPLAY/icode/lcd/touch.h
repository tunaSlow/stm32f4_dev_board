#ifndef __TOUCH_H__
#define __TOUCH_H__

#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h"
#include "lcd.h"
#include "stdlib.h"
#include "math.h"
#include "lcd.h"

extern I2C_HandleTypeDef hi2c2;//声明I2C2的HAL库结构体
#define TOUCH_GT9xxx_ADD	0x28	//GT9147/GT911芯片的I2C器件地址
#define TOUCH_MAX  5    //电容屏支持的点数,固定为5点

#define Module_Switch1		0x35 //模块配置1（其中低2位控制INT输出方式
//INT输出方式（0x34上升沿触发，0x35下降沿触发，0x36低电平查询，0x37高电平查询）

extern	uint8_t TOUCH_ID[5];//触摸芯片ID码（TOUCH_Init初始化后可使用，以ASCII码方式的“9147”）
extern	uint16_t TOUCH_X[TOUCH_MAX];//当前触发时的X坐标
extern	uint16_t TOUCH_Y[TOUCH_MAX];//当前触发时的Y坐标
extern	uint8_t  TOUCH_STA;//触摸屏当前触发状态
//TOUCH_STA低BIT0-BIT3位保存触发点数量，BIT6触摸状态（1触发，0放开），BIT7数据更新状态（1数据已更新，0数据未准备好）

//扫描方向定义（原始方向是以竖屏左上角为基点X坐标=0，Y坐标=0）
#define Portrait			0  //原始竖屏坐标（竖屏时的左上角向右是X坐标，向下是Y坐标）
#define Landscape			1  //横屏坐标（竖屏时的右上角向下是X坐标，向左是Y坐标，即横屏）（横屏适用于洋桃2号开发板）
#define Portrait_reversal	2  //竖屏坐标反转180度
#define Landscape_reversal	3  //横屏坐标反转180度

//-----------------【GT9147/GT911子地址表】-----------------------//
//
#define GT9xxx_COM	 	0X8040   //指令控制。操作内容如下：
//【GT9xxx_COM说明】
//0：读坐标状态 //1、2：差值原始值 //3：基准更新 //4：基准校准//5：关屏 //6：进入充电模式 7：退出充电模式8：切换手势唤醒固件
//0x20:进入从机接近检测模式 //0x21：进入主机接近检测模式 //0x22：进入数据传输模式 //0x23:进入主机传输模式//0x28：退出从机检测模式
//0x29：退出主机接近检测模式 //0x2A：退出数据传输模式//0xAA:ESD 保护机制使用，由驱动定时写入0xAA 并定时读取检查 //其余值无效

#define GT9xxx_Ver 		0X8047   	//配置文件版本号
#define GT9xxx_CHKSUM 	0X80FF   	//配置信息校验(0x8047到0x80FE之字节和的补码)
#define GT9xxx_PID 		0X8140   	//产品ID（从0X8140~0X8143）

#define GT9xxx_STA	 	0X814E   	//触摸屏状态总标志位
#define GT9xxx_TP1 		0X8150  	//GT9147/GT911芯片第1个触摸点坐标地址
#define GT9xxx_TP2 		0X8158  	//GT9147/GT911芯片第2个触摸点坐标地址
#define GT9xxx_TP3 		0X8160		//GT9147/GT911芯片第3个触摸点坐标地址
#define GT9xxx_TP4 		0X8168		//GT9147/GT911芯片第4个触摸点坐标地址
#define GT9xxx_TP5 		0X8170		//GT9147/GT911芯片第5个触摸点坐标地址
//GT9147数据手册有错误，第1个触摸点坐标地址是0X8150（以此类推）

void GT9xxx_Write_Config(uint8_t save);
uint8_t TOUCH_Init(void);//触摸屏初始化（驱动芯片GT9xxx）（返回1成功，0失败）
void TOUCH_Read(uint8_t dir);//读取触摸屏当前状态，有触发则改变TOUCH_STA，读出5个触发点坐标在TOUCH_X和TOUCH_Y数组中
 
#endif

/*********************************************************************************************
 * 洋桃电子 www.doyoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/















