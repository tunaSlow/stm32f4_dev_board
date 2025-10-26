/*
 * buzzer.c
 *
 *  Created on: Oct 21, 2021
 *      Author: Administrator
 */

#include "../../Core/icode/buzzer/buzzer.h"

#include "../../Core/icode/buzzer/midi.h"

#define time1 50 //������ʱ��
#define hz1 1 //��������������λ���룩


void BUZZER_SOLO1(void){//��������������ı���������ʽ1��HAL��ľ�׼��ʱ������
    uint16_t i;
    for(i=0;i<time1;i++){//ѭ����������������ʱ��
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //�������ӿ�����͵�ƽ0
       HAL_Delay(hz1); //��ʱ�����뼶��ʱ��С1΢�룬ʵ�ֵĵ����ϵͣ�����Ҫ�����д΢�뼶��ʱ�����������ʵ�ã�
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //�������ӿ�����ߵ�ƽ1
       HAL_Delay(hz1); //��ʱ
    }
}
#define time2 200 //������ʱ��
#define hz2 500 //��������������λ΢�룩
void BUZZER_SOLO2(void){//��������������ı���������ʽ2��CPU΢�뼶��ʱ��
    uint16_t i;
    for(i=0;i<time2;i++){//ѭ����������������ʱ��
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //�������ӿ�����͵�ƽ0
       delay_us(hz2); //��ʱ
       HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //�������ӿ�����ߵ�ƽ1
       delay_us(hz2); //��ʱ
    }
}

//-----------------------������MIDI����Ч��-----------------------------//

//#define	MIDI_name		music1  //������������
//#define	MIDI_len		78  	//��������������

void BUZZER_MIDI_PLAY(uint32_t *MIDI_name ,uint16_t MIDI_len){//������MIDI���ֲ��ţ�������������������ָ�룬������������
	uint16_t i,e;
	for(i=0;i<(MIDI_len/2);i++){
		for(e=0;e<MIDI_name[i*2]*MIDI_name[i*2+1]/1000;e++){
			HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_RESET); //�������ӿ�����͵�ƽ0
			delay_us(500000/MIDI_name[i*2]); //��ʱ
			HAL_GPIO_WritePin(BEEP1_GPIO_Port,BEEP1_Pin,GPIO_PIN_SET); //�������ӿ�����ߵ�ƽ1
			delay_us(500000/MIDI_name[i*2]); //��ʱ
		}
	}
}


//���¿������������е��ã����Ҫ���������֣��ɸ������BUZZER_PLAY_music3�Ⱥ�������ָ������������

void BUZZER_PLAY_music1(void){//����MIDI����1
	BUZZER_MIDI_PLAY((uint32_t *)music1,78);//������MIDI���ֲ��ţ�������������������ָ�룬������������
}

void BUZZER_PLAY_music2(void){
	BUZZER_MIDI_PLAY((uint32_t *)music2,8);//������MIDI���ֲ��ţ�������������������ָ�룬������������
}

