/*
 * buzzer.h
 *
 *  Created on: Oct 20, 2021
 *      Author: Administrator
 */

#ifndef ICODE_BUZZER_BUZZER_H_
#define ICODE_BUZZER_BUZZER_H_

#include "stm32f4xx_hal.h" //HAL���ļ�����
#include "main.h"
#include "../delay/delay.h"


void BUZZER_SOLO1(void);
void BUZZER_SOLO2(void);

void BUZZER_MIDI_PLAY(uint32_t *MIDI_name ,uint16_t MIDI_len);

void BUZZER_PLAY_music1(void);
void BUZZER_PLAY_music2(void);

#endif /* ICODE_BUZZER_BUZZER_H_ */
