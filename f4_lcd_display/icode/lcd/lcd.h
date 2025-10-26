#ifndef __LCD_H
#define __LCD_H		

#include "stm32f4xx_hal.h" //HAL库文件声明
#include "main.h"
#include "../delay/delay.h"
#include <string.h>//用于字符串处理的库
#include <stdarg.h>
#include <stdlib.h>
#include "stdio.h"
 
//本驱动程序只适用于NT35510/RM68120/OTM8009A芯片（显示分辨率800x480）
#define LCD_ID_NT35510		0x00008000  //NT35510的ID值0x00008000
#define LCD_ID_RM68120		0x00008200  //RM68120的ID值0x00008200
#define LCD_ID_OTM8009		0x00004000  //OTM8009A的ID值0x00004000

#define LCD_PRINTF_LEN  	200 //限制printf函数一次的字符上限

#define LCD_Height			800 //洋桃LCD彩屏模块 固定分辨率800x480
#define LCD_Width			480 //（以竖屏为原始方位，长边是上下高度LCD_Height，短边是左右宽度LCD_Width）

#define LCD_COM				(uint32_t *)0X60000000  //将A18地址端口置0后的地址（表示向LCD发送指令）
#define LCD_DAT				(uint32_t *)0X60080000  //将A18地址端口置1后的地址（表示向LCD发送数据）
//当数据/指令切换端口RS连接到A18时，计算地址偏移是0x40000，加上Bank1选区的FSMC_NE1基地址是0x60000000，
//得到最终使RS置1的地址是0x60040000（二进制0110 0000 0000 0100 0000 0000 0000 0000）
//但由于在执行时地址偏移值会右移1位，所以在设置时要当成A19端口，右移后正好是A18，所以真正可用地址是0X60080000

extern uint16_t SET_Height; //设置屏幕分辨率高度
extern uint16_t SET_Width; //设置屏幕分辨率宽度
extern uint16_t SET_GRAM; //设置GRAM图形寄存器
extern uint16_t SET_X; //设置X方向
extern uint16_t SET_Y; //设备Y方向
extern uint32_t LCD_ID; //读取的彩屏ID存放寄存器

extern SRAM_HandleTypeDef hsram1;    //FSMC总线的SRAM句柄（此处用于LCD模块通信）

//字体与背景颜色设置，在使用LCD_printf函数之前设置（例:ForeColor=White;BackColor=Blue;）
extern uint16_t  ForeColor;//字体颜色寄存器（全局字符/图形）
extern uint16_t  BackColor;//背景颜色（全局）

//扫描方向定义
#define LRUD  0x00 		//从左到右,从上到下
#define LRDU  0x80 		//从左到右,从下到上
#define RLUD  0x40 		//从右到左,从上到下
#define RLDU  0xC0 		//从右到左,从下到上
#define UDLR  0x20 		//从上到下,从左到右
#define UDRL  0x60 		//从上到下,从右到左
#define DULR  0xA0 		//从下到上,从左到右
#define DURL  0xE0		//从下到上,从右到左

//常用彩色值
#define White        	0xFFFF//纯白
#define Black        	0x0000//纯黑
#define Blue         	0x001F//蓝
#define Purple         	0XF81F//紫
#define SkyBlue			0X07FF//天蓝
#define Red          	0xF800//红
#define Magenta      	0xF81F//洋红
#define Green        	0x07E0//绿
#define Cyan         	0x7FFF//青
#define Yellow       	0xFFE0//黄
#define Brown 			0XBC40//深棕
#define Maroon	 		0XFC07//栗色
#define Gray  			0X8430//深灰

//底层驱动函数
void LCD_Write_COM(uint16_t R);//向LCD写指令COM
void LCD_Write_DAT(uint16_t D);//向LCD写数据DAT
uint16_t LCD_Read_DAT(void);//读LCD的RAM空间
void LCD_Write_REG(uint16_t R,uint16_t D);//写LCD寄存器（参数：地址，数据）
uint16_t LCD_Read_REG(uint16_t R);//读LCD寄存器（参数：LCD寄存器地址，返回：读出数据）
void LCD_Write_Cursor(uint16_t x,uint16_t y);//设置光标位置（参数：X坐标，Y坐标）
//LCD基础功能
uint8_t LCD_Init(void);//LCD彩屏初始化（返回值1：初始化成功，0：初始化失败）
void LCD_Direction(uint8_t dir);//设置LCD显示方向
void LCD_ON(void);//开显示和背光
void LCD_OFF(void);//关显示和背光
void LCD_CLEAR(uint16_t Color);//清屏（单色背景）
//ASCII字符显示
void LCD_DISPLAY_ASCII(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);//在屏上显示一个字符
void LCD_printf_ASCII(uint8_t row,uint8_t column,uint8_t size,char *fmt, ...);//LCD专用的printf函数
//汉字显示
void LCD_DISPLAY_CHS(uint16_t x,uint16_t y,uint16_t w,uint8_t size,uint8_t overlay);
void LCD_printf_CHS(uint16_t row,uint16_t column,uint8_t size,char *fmt, ...);
//矢量图显示
void LCD_Vector_Point(uint16_t x,uint16_t y);//绘制单像素点（参数：X坐标，Y坐标）
void LCD_Vector_Point2(uint16_t x,uint16_t y,uint16_t COLOR);//绘制单像素点2（参数：X坐标，Y坐标，颜色）
void LCD_Vector_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);//绘制矢量线段（参数：起点X，起点Y，终点X，终点Y坐标）
void LCD_Vector_Circle(uint16_t x0,uint16_t y0,uint8_t r);//绘制矢量正圆形（参数：圆心X坐标，圆心Y坐标，半径）
void LCD_Vector_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);//绘制矢量空心矩形（参数：左上角X，左上角Y，右下角X，右下角Y坐标）
void LCD_Vector_Rectangle_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);//绘制矢量实心矩形（参数：左上角X，左上角Y，右下角X，右下角Y坐标,颜色）
//BMP图片显示
void LCD_DISPLAY_BMP(uint16_t sx,uint16_t sy,uint16_t *COLOR);//绘制BMP位图（参数：左上角X，左上角Y坐标,BMP位图数组）
//用户自定义的图片显示
void LCD_DISPLAY_BMP_YoungTalkLogo(uint16_t sx,uint16_t sy);//显示洋桃LOGO的自定义图片显示函数（参数：起点X，起点Y坐标，即图片左上角）

#endif  
	 
/*********************************************************************************************
 * 洋桃电子 www.DoYoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/
