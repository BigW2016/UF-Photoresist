/*
 * UF_Photoresist.c
 *
 * Created: 12.10.2016 21:41:06
 *  Author: Wraith
 */ 



//макросы для автоматического определения параметров
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

//константы отрисовки экрана
#define LCD_TIME		7
#define LCD_PRESET		6
#define LCD_UPD			5
#define LCD_CUR_TIME	4
#define LCD_BL_H		3
#define LCD_BL_M		2
#define LCD_BL_S		1

//константы силового транзистора
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


//переменные
volatile uint8_t encCurState=0;//содержится состояние энкодера, нужно хранить для определения направления вращения
volatile uint8_t encPushDown=1;//для работы с кнопкой энкодера
		 uint8_t flagTact1=0;
		 uint8_t flagTact2=0;
		 uint8_t flagTimeStop=0;//флаг для определения окончания времени
volatile uint8_t encCount=1;//кол-во нажатий энкодера для определения какую цифру в таймере меняем
		 uint8_t flagLCD=0;//флаг перерисовки экрана	1 1 1 1 1 1 1 1
							//							^режим отсчета времени 0x80
							//							  ^режим выбора предустановки 0x40
							//							    ^необходимо обновить экран 0x20
							//								  ^режим установки(мигания)/отображения текущего времени 0x10 установка с морганием если в интервале 2-8 (&0x0F)
							//									^каким разрядом (час/мин/сек) моргать при 4 бите
							//при 0 показываем 00:00:00
							 
		 

typedef struct{
	unsigned char second;
	unsigned char minute;
	unsigned char hour;
}time;

//структура времени
time ExpTime;


int8_t EncStep =0; //счетчик кол-ва шагов


// Прерырвание по переполнению таймера 2
ISR(TIMER2_OVF_vect)
{
	//уменьшаем на 1 сек/мин/час, если предыдущий ушел меньше 0
    if (--ExpTime.second==255)    //из примера + доработано
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

//прерывание кнопка1
ISR(INT0_vect)
{
	cli();
	EIFR=1<<INT0;
	_delay_ms(20);
	if (!GET_PORT_DATA(D,TACT1_PIN))
	{
		//кнопка нажата
		flagTact1=1;
	}
/*	else
	{
		//кнопка отпущена
		if (Flag_tact1)
		{
			//кнопка была нажата
			Flag_tact1=0;
		}
	}*/
	sei();
}

//прерывание кнопка2
ISR(INT1_vect)
{
	cli();
	EIFR=1<<INT1;
	_delay_ms(20);
	if (!GET_PORT_DATA(D,TACT2_PIN))
	{
		//кнопка нажата
		flagTact2=1;
	}
/*	else
	{
		//кнопка отпущена
		if (Flag_tact2)
		{
			//кнопка была нажата
			Flag_tact2=0;
		}
	}*/
	sei();
}


//инициализация часового - таймера2
void initTimer2 (void)
{
	cli();
	// Запрещаем прерывания таймера 2 на всякий
	TIMSK2 &=~(1<<OCIE2A | 1<<OCIE2B | 1<< TOIE2);
	//разрешаем внешнее тактирование и выставляем асинхронный режим
//	ASSR|=(1<<EXCLK)|(1<<AS2);
	ASSR|=(1<<AS2);
	TCNT2=0;
	//предделитель на 128, для переполнения (255 тиков) за 1 сек
	TCCR2B|= (1<<CS22)|(0<<CS21)|(1<<CS20);
	
	//_delay_ms(50);
	while (ASSR&0x1F) {} //если хоть 1 флаг выставлен - ждем

	// Сбрасываем флаги прерываний, на всякий случай.
	TIFR2 |= (1<<OCIE2B)|(1<<OCIE2A)|(1<< TOIE2);
	//разрешаем прерывания по переполнению
	TIMSK2|=(1<<TOIE2);
	sei();
}

void disableTimer2 (void)
{
	// Запрещаем прерывания таймера 2 на всякий
	TIMSK2 &=~(1<<OCIE2A | 1<<OCIE2B | 1<< TOIE2);
	ASSR&=~(1<<AS2);
	while (ASSR&0x1F) {} //если хоть 1 флагн выставлен - ждем
	// Сбрасываем флаги прерываний, на всякий случай.
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


int8_t	encPollDelta()//вычисляем смещение энкодера и направление
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

	//обновляем экран	
	flagLCD|=(1<<LCD_UPD);

}


int main(void)
{

	uint8_t txt[16];
	uint8_t i;

	cli();
	//инициализация портов //сделать инициализацию энкодера отдельной функцией
	DDRC=0;
	PORTC=0xFF;
	DDRB=0;
	PORTB=0xFF;
	//DDRD=0;
	//PORTD=0xFF;
	
	//инициализируем кнопки
	DDRD&=~((1<<TACT1_PIN)|(1<<TACT2_PIN));//пин 2,3 на вход
	PORTD|=((1<<TACT1_PIN)|(1<<TACT2_PIN));//подтяжку на пины
	//настраиваем прерывания INT0, INT1
	EICRA=(0<<ISC11)|(1<<ISC10)|(0<<ISC01)|(1<<ISC00);//по спадающему фронту
	EIMSK=(1<<INT1)|(1<<INT0);
	
	//инициализируем порт и пин буззера
	PORTD&=~(1<<BUZZ_PIN);
	DDRD|=1<<BUZZ_PIN;
	
	//тестовый пик
	//buzz(7);
	
	//инициализируем экран
	LCDinit();
	LCDclr();
	
	//выводим квадраты для теста дисплея
	for(i=0;i<16;i++) txt[i]=0xFF;
	LCDGotoXY(0,0);
	LCDstring((uint8_t*)txt,16);
	LCDGotoXY(0,1);
	LCDstring((uint8_t*)txt,16);
	_delay_ms(500);
	LCDclr();
	
	//инициализация порта MOSFET
	//устанавливаем в 0
	MOSFET_PORT&=~(1<<MOSFET_GATE);
	//выход
	MOSFET_DDR|=(1<<MOSFET_GATE);
	
	//обнуляем время
	ExpTime.hour=0;
	ExpTime.minute=0;
	ExpTime.second=0;
	//отображаем его
	flagLCD|=(1<<LCD_UPD)|(1<<LCD_CUR_TIME);
	



	sei();
	
	
    while(1)
    {
		//если не идет обратный отсчет то работаем с энкодером
		if (!(flagLCD&(1<<LCD_TIME)))
		{
			//определяем положение энкодера
			EncStep += encPollDelta(encCurState);
	
			if (EncStep>1) 
			{
				EncStep=0;
		
				//если в режиме изменения времени, то меняем время
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
				//если в режиме выбора пресета - меняем пресеты
				if (flagLCD&(1<<LCD_PRESET))
				{
					//код
				
					//обновляем экран
					flagLCD|=(1<<LCD_UPD);

				}
			}


			if (EncStep<-1) 
			{
				EncStep=0;

				//если в режиме изменения времени, то меняем время
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
				//если в режиме выбора пресета - меняем пресеты
				if (flagLCD&(1<<LCD_PRESET))
				{
					//код
					
					//обновляем экран
					flagLCD|=(1<<LCD_UPD);

				}
				
			}
		}
			
					
		//определяем нажат ли энкодер (притянут к 0)
		if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) 
		{
			_delay_ms(20);
			if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) //точно нажат!
			{
				
				if (encPushDown) //если нажат впервые
				{
					
					if (!(flagLCD&(1<<LCD_TIME)))//если не идет обратный отсчет
					{
						//снимем флаг предустановки, если он был
						
						flagLCD&=~(1<<LCD_PRESET);
						
						encCount=(encCount<<1)&0x0F;
						//если не обнулился т.е. в пределах 2-F.Меньше 2 быть не может т.к. начальное 1 и 
						//в предыдущей строке еще сдвиг влево
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


		//нажата кнопка 1 - выбора пресета - флаг в прерывании INT0
		if (flagTact1)
		{
			//кнопка была нажата, отпустили
			flagTact1=0;
			LCDclr();
			LCDGotoXY(0,0);
			LCDsendChar(0x2B);//"+"
			_delay_ms(100);
			LCDclr();
			encCount=1;
		}

		//нажата кнопка 2 - старта засветки - флаг в прерывании INT1
		if (flagTact2)
		{
			//кнопка была нажата, отпустили
			flagTact2=0;
			encCount=1;
			
			//если время не 0 (чтоб не моргало)
			if (ExpTime.hour|ExpTime.minute|ExpTime.second)
			{
				flagLCD|=(1<<LCD_TIME);
				initTimer2();
				//запускаем MOSFET
				MOSFET_PORT|=(1<<MOSFET_GATE);
			}
			
		}
		
		//если время кончилось
		if (flagTimeStop)
		{
			cli();
			flagTimeStop=0;
			
			//обнуляем флаг экрана
			flagLCD=0;
			//LCDclr();
			ExpTime.hour=0;
			ExpTime.minute=0;
			ExpTime.second=0;
			//вырубаем MOSFET
			MOSFET_PORT&=~(1<<MOSFET_GATE);
			
			//пищим
			buzz(40);
			
			
			//выключаем прерывание
			disableTimer2();
			sei();			
		}
		
		//если выставлен флаг обновления экрана
		if (flagLCD&(1<<LCD_UPD))
		{
			//снимаем его
			flagLCD&=~(1<<LCD_UPD);
			
			if (flagLCD&(1<<LCD_CUR_TIME))
			{
				//отображаем текущее установленное время (передаем указатель на структуру)
				showTime();
			}
			
			if (flagLCD&(1<<LCD_TIME))
			{
				//отображаем текущее время (передаем указатель на структуру)
				showTime();
			}
			
		}
	
    }
}