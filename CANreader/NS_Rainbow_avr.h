/*-------------------------------------------------------------------------

16Mhz 
ATmega8 or ATmega128


-------------------------------------------------------------------------*/

#ifndef __NULSOM_RAINBOW_H__
#define __NULSOM_RAINBOW_H__
#ifndef F_CPU
#define F_CPU 16000000
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>		
#include <stdlib.h>		

#if !(F_CPU == 16000000)
#error "On an AVR, this version of the NulSom Rainbow library only supports 16 MHz"
#endif


/* USE DEVICE SELECT */

//#define ATMEGA8
//#define ATMEGA128
#define ATMEGA328


#ifdef ATMEGA8			// ATmega8 
#define	PB	0x38			// PORT B
#define PC	0x35			// PORT C
#define PD	0x32			// PORT D

#else
#ifdef ATMEGA128		// ATmega128
#define PA	0x3B			// PORT A
#define PB	0x38			// PORT B 
#define PC	0x35			// PORT C
#define PD	0x32			// PORT D
#define PE	0x23			// PORT E
#define PF	0x82			// PORT F

#else
#ifdef ATMEGA328		// ATmega328
#define	PB	0x25			// PORT B
#define	PC	0x28			// PORT C
#define	PD	0x2B			// PORT D
#else
#error "Support Device ATmega8 or ATmega128 or ATmega328"

#endif
#endif
#endif

#define DEFALUT_BRIGHT		255

#define MAX_LED 			64
#define MAX_BYTE			(MAX_LED * 3)


/* initializer */
void NS_Rainbow_init(uint16_t _numled, volatile uint8_t _port, uint8_t _outbit);
void setPin(unsigned char port, unsigned char bit);


/* basic function */
void setColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
void setColor_RGB(uint16_t n, uint32_t c);

void show(void);
void clear(void);
void cell_clear(uint16_t nCell);

void setBrightness(uint8_t b);
uint32_t RGBtoColor(uint8_t r, uint8_t g, uint8_t b);
void rainbow(uint16_t interval);


/* delay function */
void rst_delay(void);					// Delay 50us
void delay(uint32_t _mstime);			// Delay _mstime ms




#endif
