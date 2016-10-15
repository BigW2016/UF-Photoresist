/*
 * UF_Photoresist.c
 *
 * Created: 12.10.2016 21:41:06
 *  Author: Wraith
 */ 



//������� ��� ��������������� ����������� ����������
/*#define __GET_DDR(DDR_LETTER) DDR ## DDR_LETTER
#define GET_DDR(DDR_LETTER) __GET_DDR(DDR_LETTER)

#define __GET_PORT(PORT_LETTER) PORT ## PORT_LETTER
#define GET_PORT(PORT_LETTER) __GET_PORT(PORT_LETTER)*/


#define __GET_PORT_DATA(PORT_LETTER, PORT_PIN) ((PIN ## PORT_LETTER)&(1<<(PIN ## PORT_LETTER ## PORT_PIN)))
#define GET_PORT_DATA(PORT_LETTER, PORT_PIN) __GET_PORT_DATA(PORT_LETTER, PORT_PIN)

#define F_CPU			8000000L
#define TACT1_PIN		2
#define TACT2_PIN		3
#define BUZZ_PIN		0

//��������� ��������� ������
#define LCD_TIME		7
#define LCD_PRESET		6
#define LCD_UPD			5
#define LCD_CUR_TIME	4
#define LCD_BL_H		3
#define LCD_BL_M		2
#define LCD_BL_S		1

//��������� �������� �����������
#define MOSFET_DDR		DDRD
#define MOSFET_PORT		PORTD
#define MOSFET_GATE		7



#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "Encoder.h"
#include "Lcd_lib.c"
#include <stdlib.h>
#include <string.h>


//����������
volatile uint8_t encCurState=0;//���������� ��������� ��������, ����� ������� ��� ����������� ����������� ��������
volatile uint8_t encPushDown=1;//��� ������ � ������� ��������
		 uint8_t flagTact1=0;
		 uint8_t flagTact2=0;
		 uint8_t flagTimeStop=0;//���� ��� ����������� ��������� �������
volatile uint8_t encCount=1;//���-�� ������� �������� ��� ����������� ����� ����� � ������� ������
		 uint8_t flagLCD=0;//���� ����������� ������	1 1 1 1 1 1 1 1
							//							^����� ������� ������� 0x80
							//							  ^����� ������ ������������� 0x40
							//							    ^���������� �������� ����� 0x20
							//								  ^����� ���������(�������)/����������� �������� ������� 0x10 ��������� � ��������� ���� � ��������� 2-8 (&0x0F)
							//									^����� �������� (���/���/���) ������� ��� 4 ����
							//��� 0 ���������� 00:00:00
							 
		 

typedef struct{
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
}time;

//��������� �������
time ExpTime;


int8_t EncStep =0; //������� ���-�� �����


// ����������� �� ������������ ������� 2
ISR(TIMER2_OVF_vect)
{
	//��������� �� 1 ���/���/���, ���� ���������� ���� ������ 0
    if (--ExpTime.second==255)    //�� ������� + ����������
    {
	    ExpTime.second=59;
	    
	    if (--ExpTime.minute==255)
	    {
		    ExpTime.minute=59;

		    if (--ExpTime.hour==255)
		    {
			    flagTimeStop=1;
		    }
	    }
    }
	
	flagLCD|=(1<<LCD_UPD);
}

//���������� ������1
ISR(INT0_vect)
{
	cli();
	EIFR=1<<INT0;
	_delay_ms(20);
	if (!GET_PORT_DATA(D,TACT1_PIN))
	{
		//������ ������
		flagTact1=1;
	}
/*	else
	{
		//������ ��������
		if (Flag_tact1)
		{
			//������ ���� ������
			Flag_tact1=0;
		}
	}*/
	sei();
}

//���������� ������2
ISR(INT1_vect)
{
	cli();
	EIFR=1<<INT1;
	_delay_ms(20);
	if (!GET_PORT_DATA(D,TACT2_PIN))
	{
		//������ ������
		flagTact2=1;
	}
/*	else
	{
		//������ ��������
		if (Flag_tact2)
		{
			//������ ���� ������
			Flag_tact2=0;
		}
	}*/
	sei();
}


//������������� �������� - �������2
void initTimer2 (void)
{
	cli();
	// ��������� ���������� ������� 2 �� ������
	TIMSK2 &=~(1<<OCIE2A | 1<<OCIE2B | 1<< TOIE2);
	//��������� ������� ������������ � ���������� ����������� �����
//	ASSR|=(1<<EXCLK)|(1<<AS2);
	ASSR|=(1<<AS2);
	TCNT2=0;
	//������������ �� 128, ��� ������������ (255 �����) �� 1 ���
	TCCR2B|= (1<<CS22)|(0<<CS21)|(1<<CS20);
	
	//_delay_ms(50);
	while (ASSR&0x1F) {} //���� ���� 1 ���� ��������� - ����

	// ���������� ����� ����������, �� ������ ������.
	TIFR2 |= (1<<OCIE2B)|(1<<OCIE2A)|(1<< TOIE2);
	//��������� ���������� �� ������������
	TIMSK2|=(1<<TOIE2);
	sei();
}

void disableTimer2 (void)
{
	// ��������� ���������� ������� 2 �� ������
	TIMSK2 &=~(1<<OCIE2A | 1<<OCIE2B | 1<< TOIE2);
	ASSR&=~(1<<AS2);
	while (ASSR&0x1F) {} //���� ���� 1 ����� ��������� - ����
	// ���������� ����� ����������, �� ������ ������.
	TIFR2 |= (1<<OCIE2B)|(1<<OCIE2A)|(1<< TOIE2);
}


void buzz(uint16_t count)
{
	uint16_t j;
	
	for (j=1;j<count;j++)
	{
		PORTD^=(1<<BUZZ_PIN);
		_delay_ms(3);
	}
	
	PORTD&=~(1<<BUZZ_PIN);	
}


int8_t	encPollDelta()//��������� �������� �������� � �����������
{
	
	encCurState <<= 2;
	encCurState &= 0b00001100;
	encCurState += (GET_ENCODER_PORT_DATA(ENCPOLL_A_PORT, ENCPOLL_A_PIN)<<1)|GET_ENCODER_PORT_DATA(ENCPOLL_B_PORT, ENCPOLL_B_PIN);
	
	return pgm_read_byte(&(EncState[encCurState]));
}

void fillZero(char *d)
{
	if (!d[1]) 
	{
		d[1]=d[0];
		d[0]=0x30;//"0"
	}
}

void showTime(void)
{
	char digit[2];
	char string[8];
	
	//LCDclr();
	LCDGotoXY(4,0);
	itoa(ExpTime.hour,digit,10);
	fillZero(digit);
	string[0]=digit[0];
	string[1]=digit[1];
	string[2]=0x3A;//":"
	itoa(ExpTime.minute,digit,10);
	fillZero(digit);
	string[3]=digit[0];
	string[4]=digit[1];
	string[5]=0x3A;//":"
	itoa(ExpTime.second,digit,10);
	fillZero(digit);
	string[6]=digit[0];
	string[7]=digit[1];
	LCDstring((uint8_t*)string,8);

}

void checkTime(void)
{
	if (ExpTime.second>150) ExpTime.second=59;
	if (ExpTime.minute>150) ExpTime.minute=59;
	if (ExpTime.hour>150) ExpTime.hour=24;

	if (ExpTime.second>59) ExpTime.second=0;
	if (ExpTime.minute>59) ExpTime.minute=0;
	if (ExpTime.hour>24) ExpTime.hour=0;

	//��������� �����	
	flagLCD|=(1<<LCD_UPD);

}


int main(void)
{

	uint8_t txt[16];
	uint8_t i;

	cli();
	//������������� ������ //������� ������������� �������� ��������� ��������
	DDRC=0;
	PORTC=0xFF;
	DDRB=0;
	PORTB=0xFF;
	//DDRD=0;
	//PORTD=0xFF;
	
	//�������������� ������
	DDRD&=~((1<<TACT1_PIN)|(1<<TACT2_PIN));//��� 2,3 �� ����
	PORTD|=((1<<TACT1_PIN)|(1<<TACT2_PIN));//�������� �� ����
	//����������� ���������� INT0, INT1
	EICRA=(0<<ISC11)|(1<<ISC10)|(0<<ISC01)|(1<<ISC00);//�� ���������� ������
	EIMSK=(1<<INT1)|(1<<INT0);
	
	//�������������� ���� � ��� �������
	PORTD&=~(1<<BUZZ_PIN);
	DDRD|=1<<BUZZ_PIN;
	
	//�������� ���
	//buzz(7);
	
	//�������������� �����
	LCDinit();
	LCDclr();
	
	//������� �������� ��� ����� �������
	for(i=0;i<16;i++) txt[i]=0xFF;
	LCDGotoXY(0,0);
	LCDstring((uint8_t*)txt,16);
	LCDGotoXY(0,1);
	LCDstring((uint8_t*)txt,16);
	_delay_ms(500);
	LCDclr();
	
	//������������� ����� MOSFET
	//������������� � 0
	MOSFET_PORT&=~(1<<MOSFET_GATE);
	//�����
	MOSFET_DDR|=(1<<MOSFET_GATE);
	
	//�������� �����
	ExpTime.hour=0;
	ExpTime.minute=0;
	ExpTime.second=0;
	//���������� ���
	flagLCD|=(1<<LCD_UPD)|(1<<LCD_CUR_TIME);
	



	sei();
	
	
    while(1)
    {
		//���� �� ���� �������� ������ �� �������� � ���������
		if (!(flagLCD&(1<<LCD_TIME)))
		{
			//���������� ��������� ��������
			EncStep += encPollDelta(encCurState);
	
			if (EncStep>1) 
			{
				EncStep=0;
		
				//���� � ������ ��������� �������, �� ������ �����
				if (flagLCD&(1<<LCD_CUR_TIME))
				{
					switch (encCount&0x0E)
					{
						case 2: ExpTime.second--;break;
						case 4: ExpTime.minute--;break;
						case 8: ExpTime.hour--;
		
					}

					checkTime();
				}
				//���� � ������ ������ ������� - ������ �������
				if (flagLCD&(1<<LCD_PRESET))
				{
					//���
				
					//��������� �����
					flagLCD|=(1<<LCD_UPD);

				}
			}


			if (EncStep<-1) 
			{
				EncStep=0;

				//���� � ������ ��������� �������, �� ������ �����
				if (flagLCD&(1<<LCD_CUR_TIME))
				{
					switch (encCount&0x0E)
					{
						case 2: ExpTime.second++;break;
						case 4: ExpTime.minute++;break;
						case 8: ExpTime.hour++;
		
					}
					checkTime();
				}
				//���� � ������ ������ ������� - ������ �������
				if (flagLCD&(1<<LCD_PRESET))
				{
					//���
					
					//��������� �����
					flagLCD|=(1<<LCD_UPD);

				}
				
			}
		}
			
					
		//���������� ����� �� ������� (�������� � 0)
		if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) 
		{
			_delay_ms(20);
			if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) //����� �����!
			{
				
				if (encPushDown) //���� ����� �������
				{
					
					if (!(flagLCD&(1<<LCD_TIME)))//���� �� ���� �������� ������
					{
						//������ ���� �������������, ���� �� ���
						
						flagLCD&=~(1<<LCD_PRESET);
						
						encCount=(encCount<<1)&0x0F;
						//���� �� ��������� �.�. � �������� 2-F.������ 2 ���� �� ����� �.�. ��������� 1 � 
						//� ���������� ������ ��� ����� �����
						if (encCount)
						{
							flagLCD|=((1<<LCD_UPD)|(1<<LCD_CUR_TIME)|(encCount));
						}
						else
						{
							encCount=1;
							flagLCD=0;
						}
					}
					
					
				}
				encPushDown=0;
			}
			else
			{
				encPushDown=1;
			}

		}


		//������ ������ 1 - ������ ������� - ���� � ���������� INT0
		if (flagTact1)
		{
			//������ ���� ������, ���������
			flagTact1=0;
			LCDclr();
			LCDGotoXY(0,0);
			LCDsendChar(0x2B);//"+"
			_delay_ms(100);
			LCDclr();
			encCount=1;
		}

		//������ ������ 2 - ������ �������� - ���� � ���������� INT1
		if (flagTact2)
		{
			//������ ���� ������, ���������
			flagTact2=0;
			encCount=1;
			
			//���� ����� �� 0 (���� �� �������)
			if (ExpTime.hour|ExpTime.minute|ExpTime.second)
			{
				flagLCD|=(1<<LCD_TIME);
				initTimer2();
				//��������� MOSFET
				MOSFET_PORT|=(1<<MOSFET_GATE);
			}
			
		}
		
		//���� ����� ���������
		if (flagTimeStop)
		{
			cli();
			flagTimeStop=0;
			
			//�������� ���� ������
			flagLCD=0;
			//LCDclr();
			ExpTime.hour=0;
			ExpTime.minute=0;
			ExpTime.second=0;
			//�������� MOSFET
			MOSFET_PORT&=~(1<<MOSFET_GATE);
			
			//�����
			buzz(40);
			
			
			//��������� ����������
			disableTimer2();
			sei();			
		}
		
		//���� ��������� ���� ���������� ������
		if (flagLCD&(1<<LCD_UPD))
		{
			//������� ���
			flagLCD&=~(1<<LCD_UPD);
			
			if (flagLCD&(1<<LCD_CUR_TIME))
			{
				//���������� ������� ������������� ����� (�������� ��������� �� ���������)
				showTime();
			}
			
			if (flagLCD&(1<<LCD_TIME))
			{
				//���������� ������� ����� (�������� ��������� �� ���������)
				showTime();
			}
			
		}
	
    }
}