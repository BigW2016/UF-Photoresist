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

//константы для отрисовки экрана
#define WORK_TIME		7
#define PRESET			6
#define LCD_UPD			5
#define CUR_TIME		4
#define BLINK_H			3
#define BLINK_M			2
#define BLINK_S			1

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
#include <avr/eeprom.h>


//константы наименований предустановок
#define MAX_PRESET 3

/*
const char string1[] PROGMEM = "preset-1";
const char string2[] PROGMEM = "preset-2";
const char string3[] PROGMEM = "preset-3";
const char string4[] PROGMEM = " saved";

PGM_P namePreset[] = 
{
	string1,
	string2,
	string3,
	string4
};
*/

const char string1[] = "preset-1";
const char string2[] = "preset-2";
const char string3[] = "preset-3";
const char string4[] = "saved";
const char string5[] = "canceled";


const char* namePreset[] =
{
	string1,
	string2,
	string3,
	string4,
	string5
};

//переменные
volatile uint8_t encCurState=0;//содержится состояние энкодера, нужно хранить для определения направления вращения
volatile uint8_t encPushDown=1;//для работы с кнопкой энкодера
		 uint8_t flagTact1=0;
		 uint8_t flagTact2=0;
		 int8_t presetNameCur=0;//текущий пресет
		 uint8_t flagTimeStop=0;//флаг для определения окончания времени
volatile uint8_t encCount=1;//кол-во нажатий энкодера для определения какую цифру в таймере меняем
		 uint8_t flagWork=0;//флаг управления			1 1 1 1 1 1 1 1
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

//структура текущего времени
time ExpTime;

//набор времени для пресетов
time presetTime[3];

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
	
	flagWork|=(1<<LCD_UPD);
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
//	uint8_t pos;
	
	//LCDclr();
	LCDcursorOFF();
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
	if (ExpTime.hour>150) ExpTime.hour=23;

	if (ExpTime.second>59) ExpTime.second=0;
	if (ExpTime.minute>59) ExpTime.minute=0;
	if (ExpTime.hour>23) ExpTime.hour=0;

}

void loadEpprom(void)
{
	uint8_t i;
	
	for (i=0;i<3;i++)
	{
		presetTime[i].hour=eeprom_read_byte((uint8_t*)10+i*3);
		presetTime[i].minute=eeprom_read_byte((uint8_t*)11+i*3);
		presetTime[i].second=eeprom_read_byte((uint8_t*)12+i*3);
	}
	
}


int main(void)
{

	char pos;

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
/*
	for(i=0;i<16;i++) txt[i]=0xFF;
	LCDGotoXY(0,0);
	LCDstring((uint8_t*)txt,16);
	LCDGotoXY(0,1);
	LCDstring((uint8_t*)txt,16);
	_delay_ms(500);
	LCDclr();
*/
	
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
	flagWork|=(1<<LCD_UPD)|(1<<CUR_TIME);
	
	//читаем предустановки
	loadEpprom();
	
	sei();
	
	
	
    while(1)
    {
		//если не идет обратный отсчет то работаем с энкодером
		if (!(flagWork&(1<<WORK_TIME)))
		{
			//определяем положение энкодера
			EncStep += encPollDelta(encCurState);
			//если покрутили - флаг на обновление экрана
			if ((EncStep>1) || (EncStep<-1)) flagWork|=(1<<LCD_UPD);
	
		}
			
					
		//определяем нажат ли энкодер (притянут к 0)
		if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) 
		{
			_delay_ms(20);
			if (!GET_PORT_DATA(ENCPOLL_BUTT_PORT,ENCPOLL_BUTT_PIN)) //точно нажат!
			{
				
				if (encPushDown) //если нажат впервые
				{
					
					if (!(flagWork&(1<<WORK_TIME)))//если не идет обратный отсчет
					{
						//снимем флаг предустановки, если он был
						//очищаем экран что бы убрать имя пресета, если было
						LCDclr();
						flagWork&=~(1<<PRESET);
						
						//подсвечиваем сегмент
						encCount=((encCount<<1)&0x0F);
						//если не обнулился т.е. в пределах 2-F.Меньше 2 быть не может т.к. начальное 1 и 
						//в предыдущей строке еще сдвиг влево
						if (encCount)
						{
							//
							flagWork|=((1<<LCD_UPD)|(1<<CUR_TIME)|(encCount));
						}
						else
						{
							encCount=1;
							//очищаем флаг экрана + обновляем экран
							flagWork=(1<<LCD_UPD)|(1<<CUR_TIME);

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
			
			//если не идет отсчет
			if (!(flagWork&(1<<WORK_TIME)))
			{

				if (flagWork&(1<<PRESET))
				{
					//обнуляем флаг предустановки
					flagWork&=~(1<<PRESET);
		
				}
				else
				{
					//сохраняем установленное время если мы его правим сейчас
					if (flagWork&((1<<BLINK_H)|(1<<BLINK_M)|(1<<BLINK_S)))
					{
						presetTime[presetNameCur]=ExpTime;
			
						eeprom_write_byte((uint8_t*)10+presetNameCur*3,ExpTime.hour);
						eeprom_write_byte((uint8_t*)11+presetNameCur*3,ExpTime.minute);
						eeprom_write_byte((uint8_t*)12+presetNameCur*3,ExpTime.second);
						showTime();
						LCDGotoXY(2,1);
						LCDstring((uint8_t*)(namePreset[presetNameCur]),8);
						LCDGotoXY(11,1);
						LCDstring((uint8_t*)(namePreset[3]),5);
						_delay_ms(500);
						LCDclr();

					}

					flagWork=(1<<PRESET);
				}


				flagWork|=(1<<LCD_UPD);
				//сбрасываем положение энкодера
				encCount=1;
			}
		}

		//нажата кнопка 2 - старта засветки - флаг в прерывании INT1
		if (flagTact2)
		{
			//кнопка была нажата, отпустили
			flagTact2=0;
			encCount=1;
			//если уже не идет отсчет
			if (!(flagWork&(1<<WORK_TIME)))			
			{
				//если время не 0 (чтоб не моргало)
				if (ExpTime.hour|ExpTime.minute|ExpTime.second)
				{
					flagWork|=(1<<WORK_TIME);
					//что бы затереть ненужные флаги(LCD_CUR_TIME и хвост..),
					//но оставить отображение предустановки
					flagWork&=(1<<PRESET)|(1<<WORK_TIME);
					//запускаем таймер
					initTimer2();
					//запускаем MOSFET
					MOSFET_PORT|=(1<<MOSFET_GATE);
					//очистим экран, что бы убрать надпись saved если была
					//если выставлен флаг PRESET - название отобразится
					LCDclr();
					flagWork|=(1<<LCD_UPD);

				}
			}
			else
			{
				//если нажали в процессе засветки - отключаемся
				cli();
				//снимаем флаг
				flagWork&=~(1<<WORK_TIME);
				//отключаем транзистор
				MOSFET_PORT&=~(1<<MOSFET_GATE);
				//выключаем прерывание
				disableTimer2();
				sei();
				//отображаем текущее время, т.к. флаг выставляется при -1 сек, бывают артефакты
				flagWork|=(1<<LCD_UPD)|(1<<CUR_TIME);
				//отображаем надпись canceled
				LCDGotoXY(4,1);
				LCDstring((uint8_t*)(namePreset[4]),8);
				_delay_ms(500);

			}
			
		}
		
		//если время кончилось
		if (flagTimeStop)
		{
			cli();
			flagTimeStop=0;
			
			//обнуляем флаг экрана
			flagWork=0;
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
			//отображаем текущее время, т.к. флаг выставляется при -1 сек, бывают артефакты
			flagWork|=(1<<LCD_UPD)|(1<<CUR_TIME);
	
		}
		
/*LCD_UPD Блок ниже выполняется только при поднятов флаге*/
		//если выставлен флаг обновления экрана
		if (flagWork&(1<<LCD_UPD))
		{
			//снимаем его
			flagWork&=~(1<<LCD_UPD);
			//LCDclr();
			


/*WORK_TIME Засветка*/

			if (flagWork&(1<<WORK_TIME))
			{
				//отображаем текущее время
				showTime();
			}
			else
			{
//если не засветка, то обрабатываем остальное
/*PRESET Работа с предустановками*/
				if (flagWork&(1<<PRESET))
				{
				
					//Энкодер - Крутим по часовой
					if (EncStep>1)
					{	
						EncStep=0;
						//уменьшаем номер предустановки, если меньше 0 - снова 4
						if (--presetNameCur<0) presetNameCur=MAX_PRESET-1;
						//обновляем экран
						//flagWork|=(1<<LCD_UPD);
					}				

					//Энкодер - Крутим против часовой
					if (EncStep<-1)
					{
						EncStep=0;
						//увеличиваем номер предустановки, если больше4 - снова 0
						if (++presetNameCur==MAX_PRESET) presetNameCur=0;
					
						//LCDclr();
						//обновляем экран
						//flagWork|=(1<<LCD_UPD);

					}
				
					//загружаем время из пресета
					ExpTime=presetTime[presetNameCur];
					LCDclr();
					checkTime();
					showTime();
					//отображаем имя пресета
	/*
					LCDclr();
					LCDGotoXY(presetNameCur+5,1);
					LCDsendChar(0x57);
	*/
	/*
					LCDstring((uint8_t*)(PGM_P)pgm_read_word(&(namePreset[presetNameCur])),8);
				
	*/
					LCDGotoXY(4,1);
					LCDstring((uint8_t*)(namePreset[presetNameCur]),8);

				}
			
/*CUR_TIME Отображение текущего времени*/
				if (flagWork&(1<<CUR_TIME))
				{

					//Энкодер - по часовой
					if (EncStep>1)
					{
						EncStep=0;
						//меняем время. При первоначальном отображении после инициализации не реагирует
						//т.к. encCount=1
						switch (encCount&0x0E)
						{
							case 2: ExpTime.second--;break;
							case 4: ExpTime.minute--;break;
							case 8: ExpTime.hour--;
						
						}

						checkTime();
						//обновляем экран
						//flagWork|=(1<<LCD_UPD);
					}

					//Энкодер - против часовой
					if (EncStep<-1)
					{
						EncStep=0;

						switch (encCount&0x0E)
						{
							case 2: ExpTime.second++;break;
							case 4: ExpTime.minute++;break;
							case 8: ExpTime.hour++;
						
						}
						checkTime();
						//обновляем экран
						//flagWork|=(1<<LCD_UPD);

						
					}
					
					//отображаем текущее установленное время
					showTime();
					//если установка времени - включаем курсор под текущим разрядом
					if ((flagWork&(1<<CUR_TIME)) && (flagWork&((1<<BLINK_H)|(1<<BLINK_M)|(1<<BLINK_S))))
					{
						//отображаем мигающий курсор под изменяемой позицией
					
						pos=3*((flagWork&(1<<BLINK_H))>>BLINK_H)+3*((flagWork&(1<<BLINK_M))>>BLINK_M);
						LCDGotoXY(4+(8-pos),0);
						LCDcursorOnBlink();
					
					}



				}
			}
			
		}
	
    }
}