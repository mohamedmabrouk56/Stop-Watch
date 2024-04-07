/** ===================================================================================================
  *File Name: main.c
  *
  *Author: Mohamed Mabrouk
  *
  *Description: Mini Project Source file
  *
  *Date: 31/1/2024
  *
  *version: 1
    ==================================================================================================*/

/* ===================================================================================================
											      Includes
	===================================================================================================*/
#include"avr/interrupt.h"
#include"util/delay.h"

/* ===================================================================================================
									          Types Declaration
   ===================================================================================================*/

typedef unsigned char boolean;

typedef unsigned char         uint8;
typedef signed char           sint8;
typedef unsigned short        uint16;
typedef signed short          sint16;
typedef unsigned long         uint32;
typedef signed long           sint32;
typedef unsigned long long    uint64;
typedef signed long long      sint64;
typedef float 				  float32;
typedef double		   	      float64;
typedef long double			  float128;

/* EXTI */
typedef enum
{
	LOW_LEVEL,LOGICAL_CHANGE,FALLING_EDGE,RISING_EDGE
}Sense_Control;

/* GPIO */
typedef enum
{
	PIN0_ID,
	PIN1_ID,
	PIN2_ID,
	PIN3_ID,
	PIN4_ID,
	PIN5_ID,
	PIN6_ID,
	PIN7_ID
}GPIO_PIN;

typedef enum
{
	PORTA_ID,
	PORTB_ID,
	PORTC_ID,
	PORTD_ID
}GPIO_PORT;

typedef enum
{
	PIN_INPUT,PIN_OUTPUT
}GPIO_PinDirection;

typedef enum
{
	PORT_INPUT,PORT_OUTPUT=0xFF
}GPIO_PortDirection;

/* TIMERS */

typedef enum
{

	TIMER1_NO_SOURCE=0,
	TIMER1_NO_PRESCALER,
	TIMER1_PRESCALER_8,
	TIMER1_PRESCALER_64,
	TIMER1_PRESCALER_256,
	TIMER1_PRESCALER_1024,
	TIMER1_EXTSOURCE_FALLING,
	TIMER1_EXTSOURCE_RISING,

}PRESCALER;

/* ===================================================================================================
								   	          Macro Definition
   ===================================================================================================*/

#define NULL_PTR ((void *)0)
#define LOGIC_LOW        (0u)
#define LOGIC_HIGH       (1u)

/* Common Macros */
#define SET_BIT(REG,BIT) 		(REG|=(1<<BIT))
#define CLR_BIT(REG,BIT) 		(REG&=(~(1<<BIT)))
#define TOGGLE_BIT(REG,BIT)	    (REG^=1<<BIT)
#define GET_BIT(REG,BIT) 		((REG>>BIT)&1)
#define BIT_IS_SET(REG,BIT) 	(REG & (1<<BIT))
#define BIT_IS_CLEAR(REG,BIT) 	(!(REG & (1<<BIT)))

/* GPIO */

#define NUM_OF_PORTS            4
#define NUM_OF_PINS_PER_PORT    8
#define HIGH_REG 0xFF

/* Timers */

#define OC_DISCONNECTED 0
#define OC_TOOGLE 1
#define OC_CLEAR 2
#define OC_SET 3
#define TIMER1_CTC_MODE 		OC_DISCONNECTED
#define TIMER1_PRESCALLER 		TIMER1_PRESCALER_64

/* app */

#define SEGMENTS_ENABLE_PINS 0x3F
#define DECODER_PINS 0x0F
#define RESET_VALUE 0
#define SEC1_MAX_VALUE 10
#define SEC2_MAX_VALUE 5
#define MIN1_MAX_VALUE 9
#define MIN2_MAX_VALUE 5
#define HOUR1_MAX_VALUE 9
#define HOUR2_MAX_VALUE 9
/* ===================================================================================================
											  Global Variables
   ===================================================================================================*/
volatile uint8 Sec1 = RESET_VALUE;
volatile uint8 Sec2 = RESET_VALUE;
volatile uint8 Min1 = RESET_VALUE;
volatile uint8 Min2 = RESET_VALUE;
volatile uint8 Hour1 = RESET_VALUE;
volatile uint8 Hour2 = RESET_VALUE;

/* ===================================================================================================
									         Function Prototypes
   ===================================================================================================*/
/* GPIO */
void GPIO_SetPinDirection(GPIO_PORT port_num,GPIO_PIN pin_num,uint8 direction);
void GPIO_WritePin(GPIO_PORT port_num,GPIO_PIN pin_num,uint8 level);
uint8 GPIO_ReadPin(GPIO_PORT port_num,GPIO_PIN pin_num);

/* Timers */
void TIMER_init(void);

/* 7 Segments */
void NumberDisplayBySim(uint8 num);

/* EXTI */
void EXTI_INT0_init(void);
void EXTI_INT1_init(void);
void EXTI_INT2_init(void);

/* ===================================================================================================
									             Main Project
   ===================================================================================================*/

int main(void)
{
TIMER_init();
DDRA|=SEGMENTS_ENABLE_PINS;		/* Seting 7 Segments Enable Pins as Output */
DDRC|=DECODER_PINS; 			/* Seting Decoder Pins as Output */
EXTI_INT0_init();				/* initializing the Resume Button */
EXTI_INT1_init(); 				/* initializing the Pause Button  */
EXTI_INT2_init(); 				/* initializing the Reset Button  */

	for(;;)
	{
PORTA= (PORTA & 0xC0) | (0x01); /* enabling the 1st 7 Segments */
NumberDisplayBySim(Sec1);
_delay_us(1);
PORTA= (PORTA & 0xC0) | (0x02); /* enabling the 2nd 7 Segments */
NumberDisplayBySim(Sec2);
_delay_us(1);
PORTA= (PORTA & 0xC0) | (0x04); /* enabling the 3rd 7 Segments */
NumberDisplayBySim(Min1);
_delay_us(1);
PORTA= (PORTA & 0xC0) | (0x08); /* enabling the 4th 7 Segments */
NumberDisplayBySim(Min2);
_delay_us(1);
PORTA= (PORTA & 0xC0) | (0x10); /* enabling the 5th 7 Segments */
NumberDisplayBySim(Hour1);
_delay_us(1);
PORTA= (PORTA & 0xC0) | (0x20); /* enabling the 6th 7 Segments */
NumberDisplayBySim(Hour2);
_delay_us(1);
	}
}

/* ===================================================================================================
   										      Function Definition
   ===================================================================================================*/
/* GPIO */
void GPIO_SetPinDirection(GPIO_PORT port_num,GPIO_PIN pin_num,uint8 direction)
{
	if(port_num >= NUM_OF_PORTS || pin_num >= NUM_OF_PINS_PER_PORT)
	{
		/* Wrong Input*/
	}
	else
	{
		  switch(port_num)
	  {
		case PORTA_ID:
			if(direction==PIN_OUTPUT)
			{
				SET_BIT(DDRA,pin_num);/* Setting the Required pin on DDR Register to be output*/
			}
			else if(direction==PIN_INPUT)
			{
				CLR_BIT(DDRA,pin_num);/* Clearing the Required pin on DDR Register to be input*/
			}
			else
			{
				/*Wrong Direction Error*/
			}
			break;
		case PORTB_ID:
			if(direction==PIN_OUTPUT)
				{
					SET_BIT(DDRB,pin_num);/* Setting the Required pin on DDR Register to be output*/
				}
				else if(direction==PIN_INPUT)
				{
					CLR_BIT(DDRB,pin_num);/* Clearing the Required pin on DDR Register to be input*/
				}
				else
				{
					/*Wrong Direction Error*/
				}
				break;
		case PORTC_ID:
			if(direction==PIN_OUTPUT)
				{
					SET_BIT(DDRC,pin_num);/* Setting the Required pin on DDR Register to be output*/
				}
				else if(direction==PIN_INPUT)
				{
					CLR_BIT(DDRC,pin_num);/* Clearing the Required pin on DDR Register to be input*/
				}
				else
				{
					/*Wrong Direction Error*/
				}
				break;
		case PORTD_ID:
			if(direction==PIN_OUTPUT)
				{
					SET_BIT(DDRD,pin_num);/* Setting the Required pin on DDR Register to be output*/
				}
				else if(direction==PIN_INPUT)
				{
					CLR_BIT(DDRD,pin_num);/* Clearing the Required pin on DDR Register to be input*/
				}
				else
				{
					/*Wrong Direction Error*/
				}
					break;
	  }
	  }
	}

void GPIO_WritePin(GPIO_PORT port_num,GPIO_PIN pin_num,uint8 level)
{
	if(port_num >= NUM_OF_PORTS || pin_num >= NUM_OF_PINS_PER_PORT)
	{
		/* Wrong Input*/
	}
	else
	{
		  switch(port_num)
	  {
		case PORTA_ID:
			if(level == LOGIC_HIGH)
			{
				SET_BIT(PORTA,pin_num);/* Setting the Required pin on Port Register to Output a High logic 1 */
			}
			else
			{
				CLR_BIT(PORTA,pin_num);/* clearing the Required pin on Port Register to Output a low logic 0 */
			}
			break;
		case PORTB_ID:
			if(level == LOGIC_HIGH)
			{
				SET_BIT(PORTB,pin_num);/* Setting the Required pin on Port Register to Output a High logic 1 */
			}
			else
			{
				CLR_BIT(PORTB,pin_num);/* clearing the Required pin on Port Register to Output a low logic 0 */
			}
			break;
		case PORTC_ID:
			if(level == LOGIC_HIGH)
			{
				SET_BIT(PORTC,pin_num);/* Setting the Required pin on Port Register to Output a High logic 1 */
			}
			else
			{
				CLR_BIT(PORTC,pin_num);/* clearing the Required pin on Port Register to Output a low logic 0 */
			}
			break;
		case PORTD_ID:
			if(level == LOGIC_HIGH)
			{
				SET_BIT(PORTD,pin_num);/* Setting the Required pin on Port Register to Output a High logic 1 */
			}
			else
			{
				CLR_BIT(PORTD,pin_num);/* clearing the Required pin on Port Register to Output a low logic 0 */
			}
			break;
	  }
}
}

uint8 GPIO_ReadPin(GPIO_PORT port_num,GPIO_PIN pin_num)
{

	if(port_num >= NUM_OF_PORTS || pin_num >= NUM_OF_PINS_PER_PORT)
	{
		/* Wrong Input*/
	}
	else
	{
		  switch(port_num)
	  {
		case PORTA_ID:
			return GET_BIT(PINA,pin_num);
		case PORTB_ID:
			return GET_BIT(PINB,pin_num);
		case PORTC_ID:
			return GET_BIT(PINC,pin_num);
		case PORTD_ID:
			return GET_BIT(PIND,pin_num);
	  }
	}
	return -1;
}

/* Timers */
void TIMER_init(void)
{
			TCCR1B |= TIMER1_PRESCALLER;
			/* Choosing CTC Mode*/
	        CLR_BIT(TCCR1A, WGM10);
			CLR_BIT(TCCR1A, WGM11);
			SET_BIT(TCCR1B, WGM12);
			CLR_BIT(TCCR1B, WGM13);
			/*To Choose a non Pwm Mode*/
			SET_BIT(TCCR1A, FOC1A);
			/*Enabling the CtC Mode Interrupt*/
			SET_BIT(TIMSK, OCIE1A);
			/* TOP Value to Get 1 Sec*/
			OCR1A=15624;
			/*Enabling General Interrupt Enable*/
			SET_BIT(SREG, 7);
	#if   TIMER1_CTC_MODE  == OC_DISCONNECTED
			CLR_BIT(TCCR1A, COM1A1);
			CLR_BIT(TCCR1A, COM1A0);
			CLR_BIT(TCCR1A, COM1B1);
			CLR_BIT(TCCR1A, COM1B0);
	#elif TIMER1_CTC_MODE  == OC_TOOGLE
			CLR_BIT(TCCR1A, COM1A1);
			SET_BIT(TCCR1A, COM1A0);
			CLR_BIT(TCCR1A, COM1B1);
			SET_BIT(TCCR1A, COM1B0);
	#elif TIMER1_CTC_MODE  == OC_CLEAR
			SET_BIT(TCCR1A, COM1A1);
			CLR_BIT(TCCR1A, COM1A0);
			SET_BIT(TCCR1A, COM1B1);
			CLR_BIT(TCCR1A, COM1B0);
	#elif TIMER1_CTC_MODE  == OC_SET
			SET_BIT(TCCR1A, COM1A1);
			SET_BIT(TCCR1A, COM1A0);
			SET_BIT(TCCR1A, COM1B1);
			SET_BIT(TCCR1A, COM1B0);
	#endif
}
/* EXTI Initializtion*/

void EXTI_INT0_init(void)
{
	CLR_BIT(DDRD, PIN2_ID);				/* Setting the EXTI 0 PIN As An Input */
	SET_BIT(PORTD,PIN2_ID);			    /* Enabling the Internal Pull up */
	SET_BIT(SREG, PIN7_ID);				/* Setting the General Interrupt Enable */
	MCUCR |= FALLING_EDGE;				/* Choosing the Sense Control of EXTI 0 */
	SET_BIT(GICR, INT0);				/* Setting the Module Interrupt Enable of EXTI 0 */
}
void EXTI_INT1_init(void)
{
	CLR_BIT(DDRD, PIN3_ID);				/* Setting the EXTI 1 PIN As An Input */
	SET_BIT(SREG, PIN7_ID);				/* Setting the General Interrupt Enable */
	MCUCR |= (RISING_EDGE << ISC10);	/* Choosing the Sense Control of EXTI 1 */
	SET_BIT(GICR, INT1);				/* Setting the Module Interrupt Enable of EXTI 1 */
}
void EXTI_INT2_init(void)
{
	CLR_BIT(DDRB, PIN2_ID);				/* Setting the EXTI 2 PIN As An Input */
	SET_BIT(PORTB,PIN2_ID);		     	/* Enabling the Internal Pull up */
	SET_BIT(SREG, PIN7_ID);				/* Setting the General Interrupt Enable */
	SET_BIT(MCUCSR, ISC2);				/* Choosing the Sense Control of EXTI 2 */
	SET_BIT(GICR, INT2);				/* Setting the Module Interrupt Enable of EXTI 2 */
}

void NumberDisplayBySim(uint8 num)
{
PORTC&=0xF0;
PORTC|=(num & 0x0F);
}


/* ===================================================================================================
   										      Interrupts
   ===================================================================================================*/

ISR(INT0_vect)
{
	/* If Reset Button is Pressed Reset The Time Counter */
	Sec1=RESET_VALUE;
	Sec2=RESET_VALUE;
	Min1=RESET_VALUE;
	Min2=RESET_VALUE;
	Hour1=RESET_VALUE;
	Hour2=RESET_VALUE;
}
ISR(INT1_vect)
{
	/* If Pause Button is Pressed Disable the clock of the timer To stop Counting */
	TCCR1B = (TCCR1B & 0xF8) | (0x07 & TIMER1_NO_SOURCE);
}
ISR(INT2_vect)
{
	/* If Resume Button is Pressed Reenable the timer To Resume Counting */
	TCCR1B = (TCCR1B & 0xF8) | (0x07 & TIMER1_PRESCALLER);
}
ISR(TIMER1_COMPA_vect)
{
	Sec1++;
	if(Sec2 == SEC2_MAX_VALUE && Sec1 == SEC1_MAX_VALUE && Min1 == MIN1_MAX_VALUE && Min2 == MIN2_MAX_VALUE && Hour1 == HOUR1_MAX_VALUE && Hour2 == HOUR2_MAX_VALUE)
	{
	/*if 99 Hours is reached Reset the Stop watch */
	Sec1=RESET_VALUE;
	Sec2=RESET_VALUE;
	Min1=RESET_VALUE;
	Min2=RESET_VALUE;
	Hour1=RESET_VALUE;
	Hour2=RESET_VALUE;
	}
	else if(Sec2 == SEC2_MAX_VALUE && Sec1 == SEC1_MAX_VALUE && Min1 == MIN1_MAX_VALUE && Min2 == MIN2_MAX_VALUE && Hour1 == HOUR1_MAX_VALUE)
	{
	Hour1=RESET_VALUE;
	Min2=RESET_VALUE;
	Min1=RESET_VALUE;
	Sec2=RESET_VALUE;
	Sec1=RESET_VALUE;
	Hour2++;
	}
	else if(Sec2 == SEC2_MAX_VALUE && Sec1 == SEC1_MAX_VALUE && Min1 == MIN1_MAX_VALUE && Min2 == MIN2_MAX_VALUE)
	{
	Min2=RESET_VALUE;
	Min1=RESET_VALUE;
	Sec2=RESET_VALUE;
	Sec1=RESET_VALUE;
	Hour1++;
	}
	else if(Sec2 == SEC2_MAX_VALUE && Sec1 == SEC1_MAX_VALUE && Min1 == MIN1_MAX_VALUE)
	{
	 Min1=RESET_VALUE;
	 Sec2=RESET_VALUE;
	 Sec1=RESET_VALUE;
	 Min2++;
	}
	else if(Sec2 == SEC2_MAX_VALUE && Sec1 == SEC1_MAX_VALUE)
	{
	 Sec2=RESET_VALUE;
	 Sec1=RESET_VALUE;
	 Min1++;
	}
	else if(Sec1 == SEC1_MAX_VALUE)
	{
	 Sec1=RESET_VALUE;
	 Sec2++;
	}

}
