/*
 * Encoder.h
 *
 * Created: 16.04.2016 16:53:22
 * Author: Wraith
 *
 * created from EncPoll by Dx http://easyelectronics.ru/repository.php?act=view&id=13
 */ 

#include <avr/pgmspace.h>

#define __GET_ENCODER_PORT_DATA(PORT_LETTER, PORT_PIN) (((PIN ## PORT_LETTER)&(1<<(PIN ## PORT_LETTER ## PORT_PIN)))>>(PIN ## PORT_LETTER ## PORT_PIN))
#define GET_ENCODER_PORT_DATA(PORT_LETTER, PORT_PIN) __GET_ENCODER_PORT_DATA(PORT_LETTER, PORT_PIN)


//���� ���� A
#define ENCPOLL_A_PORT D

//���� ���� B
#define ENCPOLL_B_PORT D

//���� ������
#define ENCPOLL_BUTT_PORT D

//��� A
#define ENCPOLL_A_PIN 4

//��� B
#define ENCPOLL_B_PIN 5 

//��� ������
#define ENCPOLL_BUTT_PIN 6




const	int8_t	EncState[] PROGMEM =
{
	0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0
};
