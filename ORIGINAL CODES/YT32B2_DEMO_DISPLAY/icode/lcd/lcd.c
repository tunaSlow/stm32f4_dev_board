/*
 * lcd.c
 *
 *  Created on: Oct 21, 2022
 *      Author: Administrator
 */

#include "lcd.h"
#include "font.h"
#include "bmp.h"
#include "font_chs1616.h"
#include "font_chs2424.h"
#include "font_chs3232.h"
#include "font_chs4848.h"

//本驱动程序只适用于NT35510/RM68120/OTM8009A芯片，显示分辨率800x480
uint32_t LCD_ID; //读取的彩屏ID存放寄存器

uint16_t SET_Height; //设置屏幕分辨率高度
uint16_t SET_Width; //设置屏幕分辨率宽度
uint16_t SET_GRAM; //设置GRAM图形寄存器
uint16_t SET_X; //设置X方向
uint16_t SET_Y; //设备Y方向

uint16_t ForeColor = Black;//字体颜色寄存器（全局字符/图形）
uint16_t BackColor = White;//背景颜色（全局）

//--------------------------------------涉及HAL_SRAM底层函数-开始-----------------------------------------//

void LCD_Write_COM(uint16_t R){//向LCD写指令COM
	__NOP();__NOP();
	HAL_SRAM_Write_16b(&hsram1,LCD_COM,&R,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
}
void LCD_Write_DAT(uint16_t D){//向LCD写数据DAT
	__NOP();__NOP();
	HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&D,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
}
uint16_t LCD_Read_DAT(void){//读LCD的RAM空间
	uint16_t RAM;
	HAL_SRAM_Read_16b(&hsram1,LCD_DAT,&RAM,1);//向LCD读16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	return RAM;
}					   
void LCD_Write_REG(uint16_t R,uint16_t D){//写LCD寄存器（参数：地址，数据）
	HAL_SRAM_Write_16b(&hsram1,LCD_COM,&R,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&D,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
}	   
uint16_t LCD_Read_REG(uint16_t R){//读LCD寄存器（参数：LCD寄存器地址，返回：读出数据）
	LCD_Write_COM(R);//写入要读的寄存器序号
	delay_us(5);		  
	return LCD_Read_DAT();//返回读到的值
}

void LCD_Write_Cursor(uint16_t x,uint16_t y){//设置光标位置（参数：X坐标，Y坐标）
	if(LCD_ID==LCD_ID_OTM8009){//判断LCD ID OTM8009A（不同型号的指令有差异）
		LCD_Write_REG(SET_X,x>>8);
		LCD_Write_REG(SET_X+1,x&0xFF);
		LCD_Write_REG(SET_X+2,(SET_Width-1)>>8);
		LCD_Write_REG(SET_X+3,(SET_Width-1)&0xFF);
		LCD_Write_REG(SET_Y,y>>8);
		LCD_Write_REG(SET_Y+1,y&0xFF);
		LCD_Write_REG(SET_Y+2,(SET_Height-1)>>8);
		LCD_Write_REG(SET_Y+3,(SET_Height-1)&0xFF);
	}else{ //判断LCD ID NT35510/RM68120
		LCD_Write_REG(SET_X,x>>8);
		LCD_Write_REG(SET_X+1,x&0xFF);
		LCD_Write_REG(SET_Y,y>>8);
		LCD_Write_REG(SET_Y+1,y&0xFF);
	}
}
//--------------------------------------初始化与基础设置-----------------------------------------//

uint8_t LCD_Init(void){ //LCD彩屏初始化（其中参数参考LCD模块数据手册）（返回值1：初始化成功，0：初始化失败）
	uint16_t a,i;
	uint8_t buf[52]= //Gamma设置的参数
	{0x00,0x33,0x00,0x34,0x00,0x3A,0x00,0x4A,0x00,0x5C,0x00,0x81,0x00,0xA6,0x00,0xE5,
	0x01,0x13,0x01,0x54,0x01,0x82,0x01,0xCA,0x02,0x00,0x02,0x01,0x02,0x34,0x02,0x67,
	0x02,0x84,0x02,0xA4,0x02,0xB7,0x02,0xCF,0x02,0xDE,0x02,0xF2,0x02,0xFE,0x03,0x10,
	0x03,0x33,0x03,0x6D};
	LCD_ID = LCD_Read_REG(0XDA00);//读取LCD ID指令（24位ID顺序D3，D2，D1）参考LCD数据手册
	LCD_ID<<=8;//将数据左移8位
	LCD_ID |= LCD_Read_REG(0XDB00);
	LCD_ID<<=8;//将数据左移8位
	LCD_ID |= LCD_Read_REG(0XDC00);
	a=0;//初始化失败的值预先赋值
	if(LCD_ID==LCD_ID_NT35510){ //判断LCD ID是否读取正确
		LCD_Write_REG(0xF000,0x55);LCD_Write_REG(0xF001,0xAA);LCD_Write_REG(0xF002,0x52);
		LCD_Write_REG(0xF003,0x08);LCD_Write_REG(0xF004,0x01);
		//AVDD Set AVDD 5.2V
		LCD_Write_REG(0xB000,0x0D);LCD_Write_REG(0xB001,0x0D);LCD_Write_REG(0xB002,0x0D);
		//AVDD ratio
		LCD_Write_REG(0xB600,0x34);LCD_Write_REG(0xB601,0x34);LCD_Write_REG(0xB602,0x34);
		//AVEE -5.2V
		LCD_Write_REG(0xB100,0x0D);LCD_Write_REG(0xB101,0x0D);LCD_Write_REG(0xB102,0x0D);
		//AVEE ratio
		LCD_Write_REG(0xB700,0x34);LCD_Write_REG(0xB701,0x34);LCD_Write_REG(0xB702,0x34);
		//VCL -2.5V
		LCD_Write_REG(0xB200,0x00);LCD_Write_REG(0xB201,0x00);LCD_Write_REG(0xB202,0x00);
		//VCL ratio
		LCD_Write_REG(0xB800,0x24);LCD_Write_REG(0xB801,0x24);LCD_Write_REG(0xB802,0x24);
		//VGH 15V (Free pump)
		LCD_Write_REG(0xBF00,0x01);LCD_Write_REG(0xB300,0x0F);LCD_Write_REG(0xB301,0x0F);
		LCD_Write_REG(0xB302,0x0F);
		//VGH ratio
		LCD_Write_REG(0xB900,0x34);LCD_Write_REG(0xB901,0x34);LCD_Write_REG(0xB902,0x34);
		//VGL_REG -10V
		LCD_Write_REG(0xB500,0x08);LCD_Write_REG(0xB501,0x08);LCD_Write_REG(0xB502,0x08);
		LCD_Write_REG(0xC200,0x03);
		//VGLX ratio
		LCD_Write_REG(0xBA00,0x24);LCD_Write_REG(0xBA01,0x24);LCD_Write_REG(0xBA02,0x24);
		//VGMP/VGSP 4.5V/0V
		LCD_Write_REG(0xBC00,0x00);LCD_Write_REG(0xBC01,0x78);LCD_Write_REG(0xBC02,0x00);
		//VGMN/VGSN -4.5V/0V
		LCD_Write_REG(0xBD00,0x00);LCD_Write_REG(0xBD01,0x78);LCD_Write_REG(0xBD02,0x00);
		//VCOM
		LCD_Write_REG(0xBE00,0x00);LCD_Write_REG(0xBE01,0x64);
		//Gamma Setting
		for(i=0xD1;i<=0xD6;i++){ //循环写入buf数组值
			for(a=0;a<52;a++){
				LCD_Write_REG(i*0x100+a,buf[a]);
			}
		}
		//LV2 Page 0 enable
		LCD_Write_REG(0xF000,0x55);LCD_Write_REG(0xF001,0xAA);LCD_Write_REG(0xF002,0x52);
		LCD_Write_REG(0xF003,0x08);LCD_Write_REG(0xF004,0x00);
		//Display control
		LCD_Write_REG(0xB100, 0xCC);LCD_Write_REG(0xB101, 0x00);
		//Source hold time
		LCD_Write_REG(0xB600,0x05);
		//Gate EQ control
		LCD_Write_REG(0xB700,0x70);LCD_Write_REG(0xB701,0x70);
		//Source EQ control (Mode 2)
		LCD_Write_REG(0xB800,0x01);LCD_Write_REG(0xB801,0x03);LCD_Write_REG(0xB802,0x03);
		LCD_Write_REG(0xB803,0x03);
		//Inversion mode (2-dot)
		LCD_Write_REG(0xBC00,0x02);LCD_Write_REG(0xBC01,0x00);LCD_Write_REG(0xBC02,0x00);
		//Timing control 4H w/ 4-delay
		LCD_Write_REG(0xC900,0xD0);LCD_Write_REG(0xC901,0x02);LCD_Write_REG(0xC902,0x50);
		LCD_Write_REG(0xC903,0x50);LCD_Write_REG(0xC904,0x50);LCD_Write_REG(0x3500,0x00);
		LCD_Write_REG(0x3A00,0x55);  //16-bit/pixel
		LCD_Write_COM(0x1100);
		delay_us(120);//设置后必要的延时
		LCD_OFF();//初始化设置之前关显示（防止显示出乱码）
		LCD_Direction(UDRL);//设置显示方向
		LCD_CLEAR(White);//清屏
		LCD_ON();//初始化完成后打开显示和背光
		a=1;//写入初始化成功的值1
	}
	else if(LCD_ID==LCD_ID_RM68120){ //判断LCD ID是否读取正确RM68120
		LCD_Write_REG(0xF000,0x55); LCD_Write_REG(0xF001,0xAA);LCD_Write_REG(0xF002,0x52);
		LCD_Write_REG(0xF003,0x08);LCD_Write_REG(0xF004,0x01);//page 1
		//Gamma setting Red
		LCD_Write_REG(0xD100,0x00);LCD_Write_REG(0xD101,0x00);LCD_Write_REG(0xD102,0x22);
		LCD_Write_REG(0xD103,0x55);LCD_Write_REG(0xD104,0x78);LCD_Write_REG(0xD105,0x40);
		LCD_Write_REG(0xD106,0x93);LCD_Write_REG(0xD107,0xBF);LCD_Write_REG(0xD108,0xE0);
		LCD_Write_REG(0xD109,0x10);LCD_Write_REG(0xD10A,0x55);LCD_Write_REG(0xD10B,0x33);
		LCD_Write_REG(0xD10C,0x4F);LCD_Write_REG(0xD10D,0x66);LCD_Write_REG(0xD10E,0x7A);
		LCD_Write_REG(0xD10F,0x55);LCD_Write_REG(0xD110,0x8C);LCD_Write_REG(0xD111,0x9C);
		LCD_Write_REG(0xD112,0xA9);LCD_Write_REG(0xD113,0xB6);LCD_Write_REG(0xD114,0x55);
		LCD_Write_REG(0xD115,0xC2);LCD_Write_REG(0xD116,0xCE);LCD_Write_REG(0xD117,0xD8);
		LCD_Write_REG(0xD118,0xE2);LCD_Write_REG(0xD119,0x55);LCD_Write_REG(0xD11A,0xEC);
		LCD_Write_REG(0xD11B,0xED);LCD_Write_REG(0xD11C,0xF6);LCD_Write_REG(0xD11D,0xFF);
		LCD_Write_REG(0xD11E,0xAA);LCD_Write_REG(0xD11F,0x07);LCD_Write_REG(0xD120,0x0F);
		LCD_Write_REG(0xD121,0x17);LCD_Write_REG(0xD122,0x20);LCD_Write_REG(0xD123,0xAA);
		LCD_Write_REG(0xD124,0x28);LCD_Write_REG(0xD125,0x30);LCD_Write_REG(0xD126,0x3A);
		LCD_Write_REG(0xD127,0x44);LCD_Write_REG(0xD128,0xAA);LCD_Write_REG(0xD129,0x52);
		LCD_Write_REG(0xD12A,0x66);LCD_Write_REG(0xD12B,0x86);LCD_Write_REG(0xD12C,0xC5);
		LCD_Write_REG(0xD12D,0xFF);LCD_Write_REG(0xD12E,0x00);LCD_Write_REG(0xD12F,0x51);
		LCD_Write_REG(0xD130,0x8D);LCD_Write_REG(0xD131,0xC4);LCD_Write_REG(0xD132,0x0F);
		LCD_Write_REG(0xD133,0xE0);LCD_Write_REG(0xD134,0xFF);
		//Gamma setting Green
		LCD_Write_REG(0xD200,0x00);LCD_Write_REG(0xD201,0x00);LCD_Write_REG(0xD202,0x22);
		LCD_Write_REG(0xD203,0x55);LCD_Write_REG(0xD204,0x78);LCD_Write_REG(0xD205,0x40);
		LCD_Write_REG(0xD206,0x93);LCD_Write_REG(0xD207,0xBF);LCD_Write_REG(0xD208,0xE0);
		LCD_Write_REG(0xD209,0x10);LCD_Write_REG(0xD20A,0x55);LCD_Write_REG(0xD20B,0x33);
		LCD_Write_REG(0xD20C,0x4F);LCD_Write_REG(0xD20D,0x66);LCD_Write_REG(0xD20E,0x7A);
		LCD_Write_REG(0xD20F,0x55);LCD_Write_REG(0xD210,0x8C);LCD_Write_REG(0xD211,0x9C);
		LCD_Write_REG(0xD212,0xA9);LCD_Write_REG(0xD213,0xB6);LCD_Write_REG(0xD214,0x55);
		LCD_Write_REG(0xD215,0xC2);LCD_Write_REG(0xD216,0xCE);LCD_Write_REG(0xD217,0xD8);
		LCD_Write_REG(0xD218,0xE2);LCD_Write_REG(0xD219,0x55);LCD_Write_REG(0xD21A,0xEC);
		LCD_Write_REG(0xD21B,0xED);LCD_Write_REG(0xD21C,0xF6);LCD_Write_REG(0xD21D,0xFF);
		LCD_Write_REG(0xD21E,0xAA);LCD_Write_REG(0xD21F,0x07);LCD_Write_REG(0xD220,0x0F);
		LCD_Write_REG(0xD221,0x17);LCD_Write_REG(0xD222,0x20);LCD_Write_REG(0xD223,0xAA);
		LCD_Write_REG(0xD224,0x28);LCD_Write_REG(0xD225,0x30);LCD_Write_REG(0xD226,0x3A);
		LCD_Write_REG(0xD227,0x44);LCD_Write_REG(0xD228,0xAA);LCD_Write_REG(0xD229,0x52);
		LCD_Write_REG(0xD22A,0x66);LCD_Write_REG(0xD22B,0x86);LCD_Write_REG(0xD22C,0xC5);
		LCD_Write_REG(0xD22D,0xFF);LCD_Write_REG(0xD22E,0x00);LCD_Write_REG(0xD22F,0x51);
		LCD_Write_REG(0xD230,0x8D);LCD_Write_REG(0xD231,0xC4);LCD_Write_REG(0xD232,0x0F);
		LCD_Write_REG(0xD233,0xE0);LCD_Write_REG(0xD234,0xFF);
		//Gamma setting Blue
		LCD_Write_REG(0xD300,0x00);LCD_Write_REG(0xD301,0x00);LCD_Write_REG(0xD302,0x22);
		LCD_Write_REG(0xD303,0x55);LCD_Write_REG(0xD304,0x78);LCD_Write_REG(0xD305,0x40);
		LCD_Write_REG(0xD306,0x93);LCD_Write_REG(0xD307,0xBF);LCD_Write_REG(0xD308,0xE0);
		LCD_Write_REG(0xD309,0x10);LCD_Write_REG(0xD30A,0x55);LCD_Write_REG(0xD30B,0x33);
		LCD_Write_REG(0xD30C,0x4F);LCD_Write_REG(0xD30D,0x66);LCD_Write_REG(0xD30E,0x7A);
		LCD_Write_REG(0xD30F,0x55);LCD_Write_REG(0xD310,0x8C);LCD_Write_REG(0xD311,0x9C);
		LCD_Write_REG(0xD312,0xA9);LCD_Write_REG(0xD313,0xB6);LCD_Write_REG(0xD314,0x55);
		LCD_Write_REG(0xD315,0xC2);LCD_Write_REG(0xD316,0xCE);LCD_Write_REG(0xD317,0xD8);
		LCD_Write_REG(0xD318,0xE2);LCD_Write_REG(0xD319,0x55);LCD_Write_REG(0xD31A,0xEC);
		LCD_Write_REG(0xD31B,0xED);LCD_Write_REG(0xD31C,0xF6);LCD_Write_REG(0xD31D,0xFF);
		LCD_Write_REG(0xD31E,0xAA);LCD_Write_REG(0xD31F,0x07);LCD_Write_REG(0xD320,0x0F);
		LCD_Write_REG(0xD321,0x17);LCD_Write_REG(0xD322,0x20);LCD_Write_REG(0xD323,0xAA);
		LCD_Write_REG(0xD324,0x28);LCD_Write_REG(0xD325,0x30);LCD_Write_REG(0xD326,0x3A);
		LCD_Write_REG(0xD327,0x44);LCD_Write_REG(0xD328,0xAA);LCD_Write_REG(0xD329,0x52);
		LCD_Write_REG(0xD32A,0x66);LCD_Write_REG(0xD32B,0x86);LCD_Write_REG(0xD32C,0xC5);
		LCD_Write_REG(0xD32D,0xFF);LCD_Write_REG(0xD32E,0x00);LCD_Write_REG(0xD32F,0x51);
		LCD_Write_REG(0xD330,0x8D);LCD_Write_REG(0xD331,0xC4);LCD_Write_REG(0xD332,0x0F);
		LCD_Write_REG(0xD333,0xE0);LCD_Write_REG(0xD334,0xFF);
		//Gamma setting Red
		LCD_Write_REG(0xD400,0x00);LCD_Write_REG(0xD401,0x00);LCD_Write_REG(0xD402,0x22);
		LCD_Write_REG(0xD403,0x55);LCD_Write_REG(0xD404,0x78);LCD_Write_REG(0xD405,0x40);
		LCD_Write_REG(0xD406,0x93);LCD_Write_REG(0xD407,0xBF);LCD_Write_REG(0xD408,0xE0);
		LCD_Write_REG(0xD409,0x10);LCD_Write_REG(0xD40A,0x55);LCD_Write_REG(0xD40B,0x33);
		LCD_Write_REG(0xD40C,0x4F);LCD_Write_REG(0xD40D,0x66);LCD_Write_REG(0xD40E,0x7A);
		LCD_Write_REG(0xD40F,0x55);LCD_Write_REG(0xD410,0x8C);LCD_Write_REG(0xD411,0x9C);
		LCD_Write_REG(0xD412,0xA9);LCD_Write_REG(0xD413,0xB6);LCD_Write_REG(0xD414,0x55);
		LCD_Write_REG(0xD415,0xC2);LCD_Write_REG(0xD416,0xCE);LCD_Write_REG(0xD417,0xD8);
		LCD_Write_REG(0xD418,0xE2);LCD_Write_REG(0xD419,0x55);LCD_Write_REG(0xD41A,0xEC);
		LCD_Write_REG(0xD41B,0xED);LCD_Write_REG(0xD41C,0xF6);LCD_Write_REG(0xD41D,0xFF);
		LCD_Write_REG(0xD41E,0xAA);LCD_Write_REG(0xD41F,0x07);LCD_Write_REG(0xD420,0x0F);
		LCD_Write_REG(0xD421,0x17);LCD_Write_REG(0xD422,0x20);LCD_Write_REG(0xD423,0xAA);
		LCD_Write_REG(0xD424,0x28);LCD_Write_REG(0xD425,0x30);LCD_Write_REG(0xD426,0x3A);
		LCD_Write_REG(0xD427,0x44);LCD_Write_REG(0xD428,0xAA);LCD_Write_REG(0xD429,0x52);
		LCD_Write_REG(0xD42A,0x66);LCD_Write_REG(0xD42B,0x86);LCD_Write_REG(0xD42C,0xC5);
		LCD_Write_REG(0xD42D,0xFF);LCD_Write_REG(0xD42E,0x00);LCD_Write_REG(0xD42F,0x51);
		LCD_Write_REG(0xD430,0x8D);LCD_Write_REG(0xD431,0xC4);LCD_Write_REG(0xD432,0x0F);
		LCD_Write_REG(0xD433,0xE0);LCD_Write_REG(0xD434,0xFF);
		//Gamma setting Green
		LCD_Write_REG(0xD500,0x00);LCD_Write_REG(0xD501,0x00);LCD_Write_REG(0xD502,0x22);
		LCD_Write_REG(0xD503,0x55);LCD_Write_REG(0xD504,0x78);LCD_Write_REG(0xD505,0x40);
		LCD_Write_REG(0xD506,0x93);LCD_Write_REG(0xD507,0xBF);LCD_Write_REG(0xD508,0xE0);
		LCD_Write_REG(0xD509,0x10);LCD_Write_REG(0xD50A,0x55);LCD_Write_REG(0xD50B,0x33);
		LCD_Write_REG(0xD50C,0x4F);LCD_Write_REG(0xD50D,0x66);LCD_Write_REG(0xD50E,0x7A);
		LCD_Write_REG(0xD50F,0x55);LCD_Write_REG(0xD510,0x8C);LCD_Write_REG(0xD511,0x9C);
		LCD_Write_REG(0xD512,0xA9);LCD_Write_REG(0xD513,0xB6);LCD_Write_REG(0xD514,0x55);
		LCD_Write_REG(0xD515,0xC2);LCD_Write_REG(0xD516,0xCE);LCD_Write_REG(0xD517,0xD8);
		LCD_Write_REG(0xD518,0xE2);LCD_Write_REG(0xD519,0x55);LCD_Write_REG(0xD51A,0xEC);
		LCD_Write_REG(0xD51B,0xED);LCD_Write_REG(0xD51C,0xF6);LCD_Write_REG(0xD51D,0xFF);
		LCD_Write_REG(0xD51E,0xAA);LCD_Write_REG(0xD51F,0x07);LCD_Write_REG(0xD520,0x0F);
		LCD_Write_REG(0xD521,0x17);LCD_Write_REG(0xD522,0x20);LCD_Write_REG(0xD523,0xAA);
		LCD_Write_REG(0xD524,0x28);LCD_Write_REG(0xD525,0x30);LCD_Write_REG(0xD526,0x3A);
		LCD_Write_REG(0xD527,0x44);LCD_Write_REG(0xD528,0xAA);LCD_Write_REG(0xD529,0x52);
		LCD_Write_REG(0xD52A,0x66);LCD_Write_REG(0xD52B,0x86);LCD_Write_REG(0xD52C,0xC5);
		LCD_Write_REG(0xD52D,0xFF);LCD_Write_REG(0xD52E,0x00);LCD_Write_REG(0xD52F,0x51);
		LCD_Write_REG(0xD530,0x8D);LCD_Write_REG(0xD531,0xC4);LCD_Write_REG(0xD532,0x0F);
		LCD_Write_REG(0xD533,0xE0);LCD_Write_REG(0xD534,0xFF);
		//Gamma setting Blue
		LCD_Write_REG(0xD600,0x00);LCD_Write_REG(0xD601,0x00);LCD_Write_REG(0xD602,0x22);
		LCD_Write_REG(0xD603,0x55);LCD_Write_REG(0xD604,0x78);LCD_Write_REG(0xD605,0x40);
		LCD_Write_REG(0xD606,0x93);LCD_Write_REG(0xD607,0xBF);LCD_Write_REG(0xD608,0xE0);
		LCD_Write_REG(0xD609,0x10);LCD_Write_REG(0xD60A,0x55);LCD_Write_REG(0xD60B,0x33);
		LCD_Write_REG(0xD60C,0x4F);LCD_Write_REG(0xD60D,0x66);LCD_Write_REG(0xD60E,0x7A);
		LCD_Write_REG(0xD60F,0x55);LCD_Write_REG(0xD610,0x8C);LCD_Write_REG(0xD611,0x9C);
		LCD_Write_REG(0xD612,0xA9);LCD_Write_REG(0xD613,0xB6);LCD_Write_REG(0xD614,0x55);
		LCD_Write_REG(0xD615,0xC2);LCD_Write_REG(0xD616,0xCE);LCD_Write_REG(0xD617,0xD8);
		LCD_Write_REG(0xD618,0xE2);LCD_Write_REG(0xD619,0x55);LCD_Write_REG(0xD61A,0xEC);
		LCD_Write_REG(0xD61B,0xED);LCD_Write_REG(0xD61C,0xF6);LCD_Write_REG(0xD61D,0xFF);
		LCD_Write_REG(0xD61E,0xAA);LCD_Write_REG(0xD61F,0x07);LCD_Write_REG(0xD620,0x0F);
		LCD_Write_REG(0xD621,0x17);LCD_Write_REG(0xD622,0x20);LCD_Write_REG(0xD623,0xAA);
		LCD_Write_REG(0xD624,0x28);LCD_Write_REG(0xD625,0x30);LCD_Write_REG(0xD626,0x3A);
		LCD_Write_REG(0xD627,0x44);LCD_Write_REG(0xD628,0xAA);LCD_Write_REG(0xD629,0x52);
		LCD_Write_REG(0xD62A,0x66);LCD_Write_REG(0xD62B,0x86);LCD_Write_REG(0xD62C,0xC5);
		LCD_Write_REG(0xD62D,0xFF);LCD_Write_REG(0xD62E,0x00);LCD_Write_REG(0xD62F,0x51);
		LCD_Write_REG(0xD630,0x8D);LCD_Write_REG(0xD631,0xC4);LCD_Write_REG(0xD632,0x0F);
		LCD_Write_REG(0xD633,0xE0);LCD_Write_REG(0xD634,0xFF);
		//AVDD
		LCD_Write_REG(0xB000,0x00);LCD_Write_REG(0xB001,0x00);LCD_Write_REG(0xB002,0x00);
		//AVEE
		LCD_Write_REG(0xB100,0x05);LCD_Write_REG(0xB101,0x05);LCD_Write_REG(0xB102,0x05);
		//AVDD Boosting
		LCD_Write_REG(0xB600,0x44);LCD_Write_REG(0xB601,0x44);LCD_Write_REG(0xB602,0x44);
		//AVEE Boosting
		LCD_Write_REG(0xB700,0x34);LCD_Write_REG(0xB701,0x34);LCD_Write_REG(0xB702,0x34);
		//VGH
		LCD_Write_REG(0xB900,0x34);LCD_Write_REG(0xB901,0x34);LCD_Write_REG(0xB902,0x34);
		//VGL
		LCD_Write_REG(0xBA00,0x14);LCD_Write_REG(0xBA01,0x14);LCD_Write_REG(0xBA02,0x14);
		LCD_Write_REG(0xBC00,0x00);LCD_Write_REG(0xBC01,0xB8);//VGMP
		LCD_Write_REG(0xBD00,0x00);LCD_Write_REG(0xBD01,0xB8);//VGMN
		LCD_Write_REG(0xBE00,0x00);//VCOM
		LCD_Write_REG(0xBE01,0x58);//VCOM
		//Enable Page 0
		LCD_Write_REG(0xF000,0x55);LCD_Write_REG(0xF001,0xAA);LCD_Write_REG(0xF002,0x52);
		LCD_Write_REG(0xF003,0x08);LCD_Write_REG(0xF004,0x00);
		LCD_Write_REG(0xB400,0x10);//Color Enhancement Enable
		LCD_Write_REG(0xB600,0x02);//Source Output Control
		LCD_Write_REG(0xB000,0x00);//Control Signal Polarity//RGB mode1/mode2 select no enable mode
		LCD_Write_REG(0xB100,0xf8);//RAM Keep
		LCD_Write_REG(0xB101,0x00);//Normal Scan
		delay_us(10);//延时
		LCD_Write_REG(0xB700,0x22);LCD_Write_REG(0xB701,0x22);//Gate EQ Control
		//Display Timing Control
		LCD_Write_REG(0xC800,0x01);LCD_Write_REG(0xC801,0x00);LCD_Write_REG(0xC802,0x54);
		LCD_Write_REG(0xC803,0x38);LCD_Write_REG(0xC804,0x54);LCD_Write_REG(0xC805,0x38);
		LCD_Write_REG(0xC806,0x54);LCD_Write_REG(0xC807,0x38);LCD_Write_REG(0xC808,0x54);
		LCD_Write_REG(0xC809,0x38);LCD_Write_REG(0xC80A,0x8C);LCD_Write_REG(0xC80B,0x2A);
		LCD_Write_REG(0xC80C,0x2A);LCD_Write_REG(0xC80D,0x8C);LCD_Write_REG(0xC80E,0x8C);
		LCD_Write_REG(0xC80F,0x2A);LCD_Write_REG(0xC810,0x2A);
		//Source EQ Control
		LCD_Write_REG(0xB800,0x01);LCD_Write_REG(0xB801,0x03);LCD_Write_REG(0xB802,0x03);
		LCD_Write_REG(0xB803,0x03);
		//Z-inversion
		LCD_Write_REG(0xBC00,0x05);LCD_Write_REG(0xBC01,0x05);LCD_Write_REG(0xBC02,0x05);
		LCD_Write_REG(0xD000,0x01);//PWM_ENH_OE=1
		LCD_Write_REG(0xBA00,0x01);//PRG=0
		//Porch Lines
		LCD_Write_REG(0xBD00,0x01);LCD_Write_REG(0xBD01,0x10);LCD_Write_REG(0xBD02,0x07);
		LCD_Write_REG(0xBD03,0x07);LCD_Write_REG(0xBE00,0x01);LCD_Write_REG(0xBE01,0x10);
		LCD_Write_REG(0xBE02,0x07);LCD_Write_REG(0xBE03,0x07);LCD_Write_REG(0xBF00,0x01);
		LCD_Write_REG(0xBF01,0x10);LCD_Write_REG(0xBF02,0x07);LCD_Write_REG(0xBF03,0x07);
		//Enable Page 2
		LCD_Write_REG(0xF000,0x55);LCD_Write_REG(0xF001,0xAA);LCD_Write_REG(0xF002,0x52);
		LCD_Write_REG(0xF003,0x08);LCD_Write_REG(0xF004,0x02);LCD_Write_REG(0xC300,0x00);
		LCD_Write_REG(0xC301,0xA9);LCD_Write_REG(0xFE00,0x00);LCD_Write_REG(0xFE01,0x94);
		LCD_Write_REG(0x3500,0x00);//TE Enable
		LCD_Write_REG(0x3600,0x08); ///0x08
		LCD_Write_REG(0x3A00,0x55);//Color Depth
		LCD_Write_REG(0x1100,0x00);//Sleep out
		delay_us(120);//设置后必要的延时
		LCD_OFF();//初始化设置之前关显示（防止显示出乱码）
		LCD_Direction(UDRL);//设置显示方向
		LCD_CLEAR(White);//清屏
		LCD_ON();//初始化完成后打开显示和背光
		a=1;//写入初始化成功的值1
	}
	else if(LCD_ID==LCD_ID_OTM8009){ //判断LCD ID是否读取正确
		LCD_Write_REG(0xFF00,0x80);LCD_Write_REG(0xFF01,0x09);LCD_Write_REG(0xFF02,0x01);
		LCD_Write_REG(0xFF80,0x80);LCD_Write_REG(0xFF81,0x09);LCD_Write_REG(0xFF03,0x01);
		LCD_Write_REG(0xC0B4,0x10);LCD_Write_REG(0xC489,0x08);LCD_Write_REG(0xC0A3,0x00);
		LCD_Write_REG(0xC582,0xA3);LCD_Write_REG(0xC590,0xD6);LCD_Write_REG(0xC591,0x87);
		LCD_Write_REG(0xD800,0x74);LCD_Write_REG(0xD801,0x72);LCD_Write_REG(0xD900,0x60);
		LCD_Write_REG(0xE100,0x09);LCD_Write_REG(0xE101,0x0C);LCD_Write_REG(0xE102,0x12);
		LCD_Write_REG(0xE103,0x0E);LCD_Write_REG(0xE104,0x08);LCD_Write_REG(0xE105,0x19);
		LCD_Write_REG(0xE106,0x0C);LCD_Write_REG(0xE107,0x0B);LCD_Write_REG(0xE108,0x01);
		LCD_Write_REG(0xE109,0x05);LCD_Write_REG(0xE10A,0x03);LCD_Write_REG(0xE10B,0x07);
		LCD_Write_REG(0xE10C,0x0E);LCD_Write_REG(0xE10D,0x26);LCD_Write_REG(0xE10E,0x23);
		LCD_Write_REG(0xE10F,0x1B);
		LCD_Write_REG(0xE200,0x09);LCD_Write_REG(0xE201,0x0C);LCD_Write_REG(0xE202,0x12);
		LCD_Write_REG(0xE203,0x0E);LCD_Write_REG(0xE204,0x08);LCD_Write_REG(0xE205,0x19);
		LCD_Write_REG(0xE206,0x0C);LCD_Write_REG(0xE207,0x0B);LCD_Write_REG(0xE208,0x01);
		LCD_Write_REG(0xE209,0x05);LCD_Write_REG(0xE20A,0x03);LCD_Write_REG(0xE20B,0x07);
		LCD_Write_REG(0xE20C,0x0E);LCD_Write_REG(0xE20D,0x26);LCD_Write_REG(0xE20E,0x23);
		LCD_Write_REG(0xE20F,0x1B);
		LCD_Write_REG(0xC181,0x66);LCD_Write_REG(0xC1A1,0x08);LCD_Write_REG(0xC481,0x83);
		LCD_Write_REG(0xC592,0x01);LCD_Write_REG(0xC5B1,0xA9);LCD_Write_REG(0xCE80,0x85);
		LCD_Write_REG(0xCE81,0x03);LCD_Write_REG(0xCE82,0x00);LCD_Write_REG(0xCE83,0x84);
		LCD_Write_REG(0xCE84,0x03);LCD_Write_REG(0xCE85,0x00);LCD_Write_REG(0xCE86,0x83);
		LCD_Write_REG(0xCE87,0x03);LCD_Write_REG(0xCE88,0x00);LCD_Write_REG(0xCE89,0x82);
		LCD_Write_REG(0xCE8A,0x03);LCD_Write_REG(0xCE8B,0x00);LCD_Write_REG(0xCEA0,0x38);
		LCD_Write_REG(0xCEA1,0x02);LCD_Write_REG(0xCEA2,0x03);LCD_Write_REG(0xCEA3,0x21);
		LCD_Write_REG(0xCEA4,0x00);LCD_Write_REG(0xCEA5,0x00);LCD_Write_REG(0xCEA6,0x00);
		LCD_Write_REG(0xCEA7,0x38);LCD_Write_REG(0xCEA8,0x01);LCD_Write_REG(0xCEA9,0x03);
		LCD_Write_REG(0xCEAA,0x22);LCD_Write_REG(0xCEAB,0x00);LCD_Write_REG(0xCEAC,0x00);
		LCD_Write_REG(0xCEAD,0x00);LCD_Write_REG(0xCEB0,0x38);LCD_Write_REG(0xCEB1,0x00);
		LCD_Write_REG(0xCEB2,0x03);LCD_Write_REG(0xCEB3,0x23);LCD_Write_REG(0xCEB4,0x00);
		LCD_Write_REG(0xCEB5,0x00);LCD_Write_REG(0xCEB6,0x00);LCD_Write_REG(0xCEB7,0x30);
		LCD_Write_REG(0xCEB8,0x00);LCD_Write_REG(0xCEB9,0x03);LCD_Write_REG(0xCEBA,0x24);
		LCD_Write_REG(0xCEBB,0x00);LCD_Write_REG(0xCEBC,0x00);LCD_Write_REG(0xCEBD,0x00);
		LCD_Write_REG(0xCEC0,0x30);LCD_Write_REG(0xCEC1,0x01);LCD_Write_REG(0xCEC2,0x03);
		LCD_Write_REG(0xCEC3,0x25);LCD_Write_REG(0xCEC4,0x00);LCD_Write_REG(0xCEC5,0x00);
		LCD_Write_REG(0xCEC6,0x00);LCD_Write_REG(0xCEC7,0x30);LCD_Write_REG(0xCEC8,0x02);
		LCD_Write_REG(0xCEC9,0x03);LCD_Write_REG(0xCECA,0x26);LCD_Write_REG(0xCECB,0x00);
		LCD_Write_REG(0xCECC,0x00);LCD_Write_REG(0xCECD,0x00);LCD_Write_REG(0xCED0,0x30);
		LCD_Write_REG(0xCED1,0x03);LCD_Write_REG(0xCED2,0x03);LCD_Write_REG(0xCED3,0x27);
		LCD_Write_REG(0xCED4,0x00);LCD_Write_REG(0xCED5,0x00);LCD_Write_REG(0xCED6,0x00);
		LCD_Write_REG(0xCED7,0x30);LCD_Write_REG(0xCED8,0x04);LCD_Write_REG(0xCED9,0x03);
		LCD_Write_REG(0xCEDA,0x28);LCD_Write_REG(0xCEDB,0x00);LCD_Write_REG(0xCEDC,0x00);
		LCD_Write_REG(0xCEDD,0x00);LCD_Write_REG(0xCFC0,0x00);LCD_Write_REG(0xCFC1,0x00);
		LCD_Write_REG(0xCFC2,0x00);LCD_Write_REG(0xCFC3,0x00);LCD_Write_REG(0xCFC4,0x00);
		LCD_Write_REG(0xCFC5,0x00);LCD_Write_REG(0xCFC6,0x00);LCD_Write_REG(0xCFC7,0x00);
		LCD_Write_REG(0xCFC8,0x00);LCD_Write_REG(0xCFC9,0x00);LCD_Write_REG(0xCFD0,0x00);
		LCD_Write_REG(0xCBC0,0x00);LCD_Write_REG(0xCBC1,0x00);LCD_Write_REG(0xCBC2,0x00);
		LCD_Write_REG(0xCBC3,0x00);LCD_Write_REG(0xCBC4,0x04);LCD_Write_REG(0xCBC5,0x04);
		LCD_Write_REG(0xCBC6,0x04);LCD_Write_REG(0xCBC7,0x04);LCD_Write_REG(0xCBC8,0x04);
		LCD_Write_REG(0xCBC9,0x04);LCD_Write_REG(0xCBCA,0x00);LCD_Write_REG(0xCBCB,0x00);
		LCD_Write_REG(0xCBCC,0x00);LCD_Write_REG(0xCBCD,0x00);LCD_Write_REG(0xCBCE,0x00);
		LCD_Write_REG(0xCBD0,0x00);LCD_Write_REG(0xCBD1,0x00);LCD_Write_REG(0xCBD2,0x00);
		LCD_Write_REG(0xCBD3,0x00);LCD_Write_REG(0xCBD4,0x00);LCD_Write_REG(0xCBD5,0x00);
		LCD_Write_REG(0xCBD6,0x00);LCD_Write_REG(0xCBD7,0x00);LCD_Write_REG(0xCBD8,0x00);
		LCD_Write_REG(0xCBD9,0x04);LCD_Write_REG(0xCBDA,0x04);LCD_Write_REG(0xCBDB,0x04);
		LCD_Write_REG(0xCBDC,0x04);LCD_Write_REG(0xCBDD,0x04);LCD_Write_REG(0xCBDE,0x04);
		LCD_Write_REG(0xCBE0,0x00);LCD_Write_REG(0xCBE1,0x00);LCD_Write_REG(0xCBE2,0x00);
		LCD_Write_REG(0xCBE3,0x00);LCD_Write_REG(0xCBE4,0x00);LCD_Write_REG(0xCBE5,0x00);
		LCD_Write_REG(0xCBE6,0x00);LCD_Write_REG(0xCBE7,0x00);LCD_Write_REG(0xCBE8,0x00);
		LCD_Write_REG(0xCBE9,0x00);LCD_Write_REG(0xCC80,0x00);LCD_Write_REG(0xCC81,0x00);
		LCD_Write_REG(0xCC82,0x00);LCD_Write_REG(0xCC83,0x00);LCD_Write_REG(0xCC84,0x0C);
		LCD_Write_REG(0xCC85,0x0A);LCD_Write_REG(0xCC86,0x10);LCD_Write_REG(0xCC87,0x0E);
		LCD_Write_REG(0xCC88,0x03);LCD_Write_REG(0xCC89,0x04);LCD_Write_REG(0xCC90,0x00);
		LCD_Write_REG(0xCC91,0x00);LCD_Write_REG(0xCC92,0x00);LCD_Write_REG(0xCC93,0x00);
		LCD_Write_REG(0xCC94,0x00);LCD_Write_REG(0xCC95,0x00);LCD_Write_REG(0xCC96,0x00);
		LCD_Write_REG(0xCC97,0x00);LCD_Write_REG(0xCC98,0x00);LCD_Write_REG(0xCC99,0x00);
		LCD_Write_REG(0xCC9A,0x00);LCD_Write_REG(0xCC9B,0x00);LCD_Write_REG(0xCC9C,0x00);
		LCD_Write_REG(0xCC9D,0x00);LCD_Write_REG(0xCC9E,0x0B);LCD_Write_REG(0xCCA0,0x09);
		LCD_Write_REG(0xCCA1,0x0F);LCD_Write_REG(0xCCA2,0x0D);LCD_Write_REG(0xCCA3,0x01);
		LCD_Write_REG(0xCCA4,0x02);LCD_Write_REG(0xCCA5,0x00);LCD_Write_REG(0xCCA6,0x00);
		LCD_Write_REG(0xCCA7,0x00);LCD_Write_REG(0xCCA8,0x00);LCD_Write_REG(0xCCA9,0x00);
		LCD_Write_REG(0xCCAA,0x00);LCD_Write_REG(0xCCAB,0x00);LCD_Write_REG(0xCCAC,0x00);
		LCD_Write_REG(0xCCAD,0x00);LCD_Write_REG(0xCCAE,0x00);LCD_Write_REG(0xCCB0,0x00);
		LCD_Write_REG(0xCCB1,0x00);LCD_Write_REG(0xCCB2,0x00);LCD_Write_REG(0xCCB3,0x00);
		LCD_Write_REG(0xCCB4,0x0D);LCD_Write_REG(0xCCB5,0x0F);LCD_Write_REG(0xCCB6,0x09);
		LCD_Write_REG(0xCCB7,0x0B);LCD_Write_REG(0xCCB8,0x02);LCD_Write_REG(0xCCB9,0x01);
		LCD_Write_REG(0xCCC0,0x00);LCD_Write_REG(0xCCC1,0x00);LCD_Write_REG(0xCCC2,0x00);
		LCD_Write_REG(0xCCC3,0x00);LCD_Write_REG(0xCCC4,0x00);LCD_Write_REG(0xCCC5,0x00);
		LCD_Write_REG(0xCCC6,0x00);LCD_Write_REG(0xCCC7,0x00);LCD_Write_REG(0xCCC8,0x00);
		LCD_Write_REG(0xCCC9,0x00);LCD_Write_REG(0xCCCA,0x00);LCD_Write_REG(0xCCCB,0x00);
		LCD_Write_REG(0xCCCC,0x00);LCD_Write_REG(0xCCCD,0x00);LCD_Write_REG(0xCCCE,0x0E);
		LCD_Write_REG(0xCCD0,0x10);LCD_Write_REG(0xCCD1,0x0A);LCD_Write_REG(0xCCD2,0x0C);
		LCD_Write_REG(0xCCD3,0x04);LCD_Write_REG(0xCCD4,0x03);LCD_Write_REG(0xCCD5,0x00);
		LCD_Write_REG(0xCCD6,0x00);LCD_Write_REG(0xCCD7,0x00);LCD_Write_REG(0xCCD8,0x00);
		LCD_Write_REG(0xCCD9,0x00);LCD_Write_REG(0xCCDA,0x00);LCD_Write_REG(0xCCDB,0x00);
		LCD_Write_REG(0xCCDC,0x00);LCD_Write_REG(0xCCDD,0x00);LCD_Write_REG(0xCCDE,0x00);
		//LCD_Write_REG(0xB391,0x20);LCD_Write_REG(0xB392,0x20);//BGR
		LCD_Write_REG(0xFF00,0xFF);LCD_Write_REG(0xFF01,0xFF);LCD_Write_REG(0xFF02,0xFF);
		LCD_Write_REG(0x3600,0xD0);//DisplAy DirECtion180
		LCD_Write_REG(0x3500,0x00);//TE(FmArk)SignAl On
		LCD_Write_REG(0x4400,0x01);
		LCD_Write_REG(0x4401,0xFF);//TE(FmArk)SignAl Output Position
		LCD_Write_REG(0x3600,0x08);//DisplAy DirECtion 0
		LCD_Write_REG(0x3500,0x00);//TE(FmArk)SignAl On
		LCD_Write_REG(0x4400,0x01);
		LCD_Write_REG(0x4401,0x22);//TE(FmArk)SignAl Output Position
		LCD_Write_REG(0x5100,0xFF);//BACklight LEvEl Control
		LCD_Write_REG(0x5300,0x2C);//BACklight On
		LCD_Write_REG(0x5500,0x00);//CABC FunCtion OFF
		LCD_Write_REG(0x3A00,0x55);
		LCD_Write_COM(0x1100);
		delay_us(120);//设置后必要的延时
		LCD_OFF();//初始化设置之前关显示（防止显示出乱码）
		LCD_Direction(UDRL);//设置显示方向
		LCD_CLEAR(White);//清屏
		LCD_ON();//初始化完成后打开显示和背光
		a=1;//写入初始化成功的值1
	}
	return a;
}  

void LCD_Direction(uint8_t dir){//设置LCD显示方向
	SET_GRAM=0X2C00;//设置写入GRAM的指令
	SET_X=0X2A00;//设置写X坐标指令
	SET_Y=0X2B00; //设置写Y坐标指令
	if(dir==LRUD || dir==LRDU || dir==RLUD || dir==RLDU){	//竖屏
		SET_Width=LCD_Width;//设置宽度
		SET_Height=LCD_Height;//设置高度
	}else{ //横屏
		SET_Width=LCD_Height;//设置宽度
		SET_Height=LCD_Width;//设置高度
	}
	LCD_Write_REG(0X3600,dir);//写入显示方向
	LCD_Write_COM(SET_X);LCD_Write_DAT(0);
	LCD_Write_COM(SET_X+1);LCD_Write_DAT(0);
	LCD_Write_COM(SET_X+2);LCD_Write_DAT((SET_Width-1)>>8);
	LCD_Write_COM(SET_X+3);LCD_Write_DAT((SET_Width-1)&0XFF);
	LCD_Write_COM(SET_Y);LCD_Write_DAT(0);
	LCD_Write_COM(SET_Y+1);LCD_Write_DAT(0);
	LCD_Write_COM(SET_Y+2);LCD_Write_DAT((SET_Height-1)>>8);
	LCD_Write_COM(SET_Y+3);LCD_Write_DAT((SET_Height-1)&0XFF);
}

void LCD_ON(void){//LCD开启显示和背光
	LCD_Write_COM(0X2900);	//开启显示
	HAL_GPIO_WritePin(GPIOB,LCD_BL_Pin,GPIO_PIN_SET);
}

void LCD_OFF(void){//LCD关闭显示和背光
	HAL_GPIO_WritePin(GPIOB,LCD_BL_Pin,GPIO_PIN_RESET);
	LCD_Write_COM(0X2800);	//关闭显示
}

void LCD_CLEAR(uint16_t COLOR){//清屏函数（参数：背景的单色值）
	uint32_t index=0;
	uint32_t totalpoint=LCD_Width;
	totalpoint*=LCD_Height;//得到总点数
	LCD_Write_Cursor(0x00,0x0000);//设置光标位置
	LCD_Write_COM(SET_GRAM);//开始写入GRAM
	for(index=0;index<totalpoint;index++){
		HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&COLOR,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	}
	BackColor = COLOR; //将清屏后的颜色值作为字符显示的背景颜色
	ForeColor = ~COLOR;//将清屏后的颜色值的“补色”作为字符颜色
}

//--------------------------------------LCD字符显示函数（应用层，可在主函数中调用）-----------------------------------------//
void LCD_DISPLAY_ASCII(uint16_t x,uint16_t y,uint8_t adcii,uint8_t size,uint8_t overlay){//在屏上显示一个字符
	//参数：X坐标，Y坐标，字符内容，字号（12/16/24/32/48），叠加/覆盖（1叠加，0覆盖）
    uint8_t b,r;
	uint16_t t,z=y;
	uint16_t csize=(size/8)*(size/2);//得出一个字符所用到的字节数量
	adcii=adcii-' ';//得到偏移后的值（ASCII字库是从空格开始取模，所以-' '就是对应字符的字库）
	for(t=0;t<csize;t++){
		if(size==12)b=ASCII_1206[adcii][t]; 	 	//调用1206字体
		else if(size==16)b=ASCII_1608[adcii][t];	//调用1608字体
		else if(size==24)b=ASCII_2412[adcii][t];	//调用2412字体
		else if(size==32)b=ASCII_3216[adcii][t];	//调用3216字体
		else if(size==48)b=ASCII_4824[adcii][t];	//调用4824字体
		else b=ASCII_4824[adcii][t];	//如输入字体值错误，则调用4824字体
		for(r=0;r<8;r++){
			if(b&0x80)LCD_Vector_Point2(x,y,ForeColor);
			else if(overlay==0)LCD_Vector_Point2(x,y,BackColor);
			b<<=1;y++;
			if(y>=SET_Height)return;//判断Y方向超出显示区域
			if((y-z)==size){
				y=z;x++;
				if(x>=SET_Width)return;//判断X方向超出显示区域
				break;
			}
		}
	}
}

//调用方法：LCD_printf(1,0,32,"TIME %d:%d",a,b); //在LCD上显示字符串
void LCD_printf_ASCII(uint8_t row,uint8_t column,uint8_t size,char *fmt, ...){//LCD专用的ASCII码的printf显示函数
	//参数：row是行数，column是列数，size是字号（12/16/24/32/48），之后是通用printf函数用法
    char buff[LCD_PRINTF_LEN+1];  //用于存放转换后的数据 [长度]
    uint16_t t=0,i=0;
    va_list arg_ptr;
	va_start(arg_ptr, fmt);
	vsnprintf(buff, LCD_PRINTF_LEN+1, fmt,  arg_ptr);//数据转换
	i=strlen(buff);//得出数据长度
	if(strlen(buff)>LCD_PRINTF_LEN) i=LCD_PRINTF_LEN;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
	for(t=0;t<i;t++){ //循环显示出字符串
		LCD_DISPLAY_ASCII((column+t)*(size/2),row*size,buff[t],size,0);//显示出单个字符
	}
	va_end(arg_ptr);
}

//----------------------- 用于汉字显示的程序 -------------------------//

#define LCD_GB_AllWordNum		100  //索引的总字数（当实际取模大于总数量时修改，小于100时不用改）

void LCD_DISPLAY_CHS(uint16_t x,uint16_t y,uint16_t w,uint8_t size,uint8_t overlay){//显示24*24尺寸的汉字或图标
	//参数：x汉字的页坐标，y汉字的列坐标，w汉字内码（用双引号加汉字"洋"表示），overlay叠加覆盖（1叠加，0覆盖）
	uint8_t b,r;
	uint16_t wordNum;//临时存放字数
	uint16_t t,z=y;
	uint16_t csize=(size/8)*size;//得出一个汉字的字节数量
	for(wordNum=0;wordNum<LCD_GB_AllWordNum;wordNum++){//循环比对所有汉字索引
		if(size==16){r = (uint8_t)LCD_GB_16[wordNum].Index[0]; b = (uint8_t)LCD_GB_16[wordNum].Index[1];}
		else if(size==24){r = (uint8_t)LCD_GB_24[wordNum].Index[0]; b = (uint8_t)LCD_GB_24[wordNum].Index[1];}
		else if(size==32){r = (uint8_t)LCD_GB_32[wordNum].Index[0]; b = (uint8_t)LCD_GB_32[wordNum].Index[1];}
		else if(size==48){r = (uint8_t)LCD_GB_48[wordNum].Index[0]; b = (uint8_t)LCD_GB_48[wordNum].Index[1];}
		if(w/0x100 == r && w%0x100 == b){//将输入的汉字与取模索引比对
			for(t=0;t<csize;t++){//所要写入的总字节数循环
				if(size==16)b=LCD_GB_16[wordNum].Msk[t];//调用1616汉字数组
				else if(size==24)b=LCD_GB_24[wordNum].Msk[t];//调用2424汉字数组
				else if(size==32)b=LCD_GB_32[wordNum].Msk[t];//调用3232汉字数组
				else if(size==48)b=LCD_GB_48[wordNum].Msk[t];//调用4848汉字数组
				for(r=0;r<8;r++){//完成一个字节8位的写入
					if(b&0x80)LCD_Vector_Point2(x,y,ForeColor);//如果最高位为1则写入字体颜色
					else if(overlay==0)LCD_Vector_Point2(x,y,BackColor);//如果叠加覆盖位为0则写入背景色
					b<<=1;y++;//位右移到下一位
					if(y>=SET_Height)return;//判断Y方向超出显示区域
					if((y-z)==size){
						y=z;x++;
						if(x>=SET_Width)return;//判断X方向超出显示区域
						break;
					}
				}
			}wordNum = 0xFFFE;//手动使数值超出，以退出循环
		}
	}
}

void LCD_printf_CHS(uint16_t row,uint16_t column,uint8_t size,char *fmt, ...){//24*24中文显示的printf函数（可在主函数中调用）
	//参数：行，列，汉字内容。调用方法：OLED_printf(0,0,"洋桃电子");
    char buff[100];  //用于存放转换后的数据 [长度]
    uint16_t i=0;
	uint8_t t=0,n=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buff,100,fmt,arg_ptr);//数据转换
    i=strlen(buff)/2;//得出数据长度
    if(strlen(buff)>99)i=99;//如果长度大于最大值，则长度等于最大值（多出部分忽略）
    for(t=0;t<i;t++){ //循环显示出字符串
		LCD_DISPLAY_CHS((column+t)*size,row*size,buff[n]*0x100+buff[n+1],size,0);//显示中文24*24
		n+=2;//计数加
    }
    va_end(arg_ptr);
}

//--------------------------------------LCD矢量图绘制函数（应用层，可在主函数中调用）-----------------------------------------//

void LCD_Vector_Point(uint16_t x,uint16_t y){//绘制单像素点（参数：X坐标，Y坐标）
	LCD_Write_Cursor(x,y);//设置光标位置
	LCD_Write_COM(SET_GRAM);//开始写入GRAM
	HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&ForeColor,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
}

void LCD_Vector_Point2(uint16_t x,uint16_t y,uint16_t COLOR){//绘制单像素点（参数：X坐标，Y坐标，颜色）
	if(LCD_ID==LCD_ID_OTM8009){//判断LCD ID OTM8009A（不同型号的指令有差异）
		LCD_Write_COM(SET_X);LCD_Write_DAT(x>>8);//写入
		LCD_Write_COM(SET_X+1);LCD_Write_DAT(x&0xFF);
		LCD_Write_COM(SET_X+2);LCD_Write_DAT(x>>8);
		LCD_Write_COM(SET_X+3);LCD_Write_DAT(x&0xFF);
		LCD_Write_COM(SET_Y);LCD_Write_DAT(y>>8);
		LCD_Write_COM(SET_Y+1);LCD_Write_DAT(y&0xFF);
		LCD_Write_COM(SET_Y+2);LCD_Write_DAT(y>>8);
		LCD_Write_COM(SET_Y+3);LCD_Write_DAT(y&0XFF);
	}else{//判断LCD ID NT35510/RM68120
		LCD_Write_COM(SET_X);LCD_Write_DAT(x>>8);//写入
		LCD_Write_COM(SET_X+1);LCD_Write_DAT(x&0xFF);
		LCD_Write_COM(SET_Y);LCD_Write_DAT(y>>8);
		LCD_Write_COM(SET_Y+1);LCD_Write_DAT(y&0xFF);
	}
	HAL_SRAM_Write_16b(&hsram1,LCD_COM,&SET_GRAM,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&COLOR,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
}

void LCD_Vector_Line(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2){//绘制矢量线段（参数：起点X，起点Y，终点X，终点Y坐标）
	uint16_t t;
	int xerr=0,yerr=0,delta_x,delta_y,distance; 
	int incx,incy,uRow,uCol; 
	delta_x=x2-x1; //计算坐标增量 
	delta_y=y2-y1; 
	uRow=x1; uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向 
	else if(delta_x==0)incx=0;//垂直线 
	else {incx=-1;delta_x=-delta_x;} 
	if(delta_y>0)incy=1; 
	else if(delta_y==0)incy=0;//水平线 
	else{incy=-1;delta_y=-delta_y;} 
	if( delta_x>delta_y)distance=delta_x;
	else distance=delta_y;//选取基本增量坐标轴
	for(t=0;t<=distance+1;t++){//绘制线输出
		LCD_Vector_Point(uRow,uCol);//绘制点
		xerr+=delta_x; yerr+=delta_y;
		if(xerr>distance){xerr-=distance; uRow+=incx;}
		if(yerr>distance){yerr-=distance; uCol+=incy;}
	}  
}    

void LCD_Vector_Circle(uint16_t x0,uint16_t y0,uint8_t r){//绘制矢量正圆形（参数：圆心X坐标，圆心Y坐标，半径）
	int a,b,di;
	if(x0>r && y0>r && x0<(LCD_Height-r) && y0<(LCD_Width-r)){//判断坐标+半径后不要超出绘制区域
		a=0;b=r;
		di=3-(r<<1);//判断下个点位置的标志
		while(a<=b){//使用Bresenham直线算法
			LCD_Vector_Point(x0+a,y0-b);//
			LCD_Vector_Point(x0+b,y0-a);//
			LCD_Vector_Point(x0+b,y0+a);//
			LCD_Vector_Point(x0+a,y0+b);//
			LCD_Vector_Point(x0-a,y0+b);//
			LCD_Vector_Point(x0-b,y0+a);//
			LCD_Vector_Point(x0-a,y0-b);//
			LCD_Vector_Point(x0-b,y0-a);//
			a++;
			if(di<0)di+=4*a+6;
			else{di+=10+4*(a-b); b--;}
		}
	}
}

void LCD_Vector_Rectangle(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2){//绘制矢量空心矩形（参数：左上角X，左上角Y，右下角X，右下角Y坐标）
	LCD_Vector_Line(x1,y1,x2,y1);//调用绘制线段函数
	LCD_Vector_Line(x1,y1,x1,y2);
	LCD_Vector_Line(x1,y2,x2,y2);
	LCD_Vector_Line(x2,y1,x2,y2);
}

void LCD_Vector_Rectangle_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t COLOR){//绘制矢量实心矩形（参数：左上角X，左上角Y，右下角X，右下角Y坐标,颜色）
	uint16_t i,j,xlen=0;
	xlen=ex-sx+1;
	for(i=sy;i<=ey;i++){
		LCD_Write_Cursor(sx,i);//设置光标位置
		LCD_Write_COM(SET_GRAM);//开始写入GRAM
		for(j=0;j<xlen;j++)//显示颜色
		HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&COLOR,1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	}
}

//--------------------------------------LCD位图绘制函数（应用层，可在主函数中调用）-----------------------------------------//

void LCD_DISPLAY_BMP(uint16_t sx,uint16_t sy,uint16_t *COLOR){//绘制BMP位图（参数：左上角X，左上角Y坐标,BMP位图数组）
	uint16_t i,j,h,w;
	w= COLOR[1];//得出绘图宽度（通过Image2lcd软件生成数组时带有的图片头数据得到）
	h= COLOR[2];//得出绘图高度
	for(i=0;i<h;i++){//循环写入行数（高度h）
		LCD_Write_Cursor(sx,sy+i);//设置光标位置
		LCD_Write_COM(SET_GRAM);//开始写入GRAM
		for(j=0;j<w;j++)//循环写入列出（宽度w）
			HAL_SRAM_Write_16b(&hsram1,LCD_DAT,&COLOR[i*w+j+4],1);//向LCD写16位数据（句柄，指令COM/数据DAT，存放寄存器，数量）
	}
}

//--------------------------------------LCD位图绘制函数（用户自定义的位图函数）-----------------------------------------//

void LCD_DISPLAY_BMP_YoungTalkLogo(uint16_t sx,uint16_t sy){ //显示洋桃LOGO的自定义图片显示函数（参数：起点X，起点Y坐标，即图片左上角）
	//调用绘制BPM位图
	LCD_DISPLAY_BMP(sx,sy,(uint16_t *)gImage_YoungTalkBMP);//根据bmp.h文件里的对应数组名对向不同的图片
	//（例如：gImage_YoungTalkBMP 指向洋桃电子LOGO图片）（图片尺寸来自图片头数据）
}

/*********************************************************************************************
 * 洋桃电子 www.DoYoung.net
 * 部分程序代码复制自网络开源资料 如有侵权请联系我们处理
 * 洋桃电子原创程序代码部分均未声明版权 可自由复制使用 我们不对代码做任何担保
*********************************************************************************************/
