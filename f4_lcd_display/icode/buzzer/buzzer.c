/*
 * buzzer.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "buzzer.h"
#include "midi.h"

#define time1 50 //单音的时长
#define hz1 1 //单音的音调（单位毫秒）


void BUZZER_SOLO1(void){//蜂鸣器输出单音的报警音（样式1：HAL库的精准延时函数）
    uint16_t i;
    for(i=0;i<time1;i++){//循环次数决定单音的时长
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //蜂鸣器接口输出低电平0
       HAL_Delay(hz1); //延时（毫秒级延时最小1微秒，实现的单调较低，因不需要额外编写微秒级延时函数所以最简单实用）
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //蜂鸣器接口输出高电平1
       HAL_Delay(hz1); //延时
    }
}
#define time2 200 //单音的时长
#define hz2 500 //单音的音调（单位微秒）
void BUZZER_SOLO2(void){//蜂鸣器输出单音的报警音（样式2：CPU微秒级延时）
    uint16_t i;
    for(i=0;i<time2;i++){//循环次数决定单音的时长
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //蜂鸣器接口输出低电平0
       delay_us(hz2); //延时
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //蜂鸣器接口输出高电平1
       delay_us(hz2); //延时
    }
}

//-----------------------以下是MIDI播放效果-----------------------------//

//#define	MIDI_name		music1  //音乐数组名称
//#define	MIDI_len		78  	//音乐数组总数量

void BUZZER_MIDI_PLAY(uint32_t *MIDI_name ,uint16_t MIDI_len){//蜂鸣器MIDI音乐播放（参数：音乐数组名称指针，数组总数量）
	uint16_t i,e;
	for(i=0;i<(MIDI_len/2);i++){
		for(e=0;e<MIDI_name[i*2]*MIDI_name[i*2+1]/1000;e++){
			HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //蜂鸣器接口输出低电平0
			delay_us(500000/MIDI_name[i*2]); //延时
			HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //蜂鸣器接口输出高电平1
			delay_us(500000/MIDI_name[i*2]); //延时
		}
	}
}


//以下可以在主函数中调用，如果要增加新音乐，可复制添加BUZZER_PLAY_music3等函数，并指向音乐数组名

void BUZZER_PLAY_music1(void){//播放MIDI音乐1
	BUZZER_MIDI_PLAY((uint32_t *)music1,78);//蜂鸣器MIDI音乐播放（参数：音乐数组名称指针，数组总数量）
}

void BUZZER_PLAY_music2(void){
	BUZZER_MIDI_PLAY((uint32_t *)music2,8);//蜂鸣器MIDI音乐播放（参数：音乐数组名称指针，数组总数量）
}

