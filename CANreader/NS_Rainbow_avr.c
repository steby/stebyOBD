/*-------------------------------------------------------------------------

16Mhz 
ATmega8 or ATmega128

-------------------------------------------------------------------------*/

#include "NS_Rainbow_avr.h"


volatile uint16_t	maxLED = MAX_LED;			
volatile uint16_t	maxLEDBytes = MAX_BYTE;		
volatile uint16_t 	nCells;						
volatile uint16_t 	nBytes;						
volatile uint8_t 	*cells;

volatile uint8_t *port;
volatile uint8_t maskPin;
volatile uint8_t brightness = 255;

void rst_delay(void) {_delay_us(50);}
void delay(uint32_t _mstime) {while(_mstime){_delay_loop_2(((F_CPU) / 4e3));_mstime--;}}


// Initializer Func
void NS_Rainbow_init(uint16_t _numled, volatile uint8_t _port, uint8_t _outbit)
{
	setPin(_port, _outbit);
	port = (volatile uint8_t *)_port;
	maskPin = ( 1 << _outbit );
	
	nCells		= _numled;
	nBytes		= maxLEDBytes;

	brightness = DEFALUT_BRIGHT;						
		
	if((cells = (uint8_t *)malloc(nBytes)))		// MAXLED(Default=64) Size LED Clear
		memset(cells, 0, nBytes);
	show();
	free(cells);
	_delay_ms(50);								// Stabilizer Delay

	nBytes = _numled * 3;
	if((cells = (uint8_t *)malloc(nBytes)))
		memset(cells, 0, nBytes);
	
}


void setPin(uint8_t _port, uint8_t _bit)
{
	*(volatile uint8_t *)(_port - 0x01) |= ( 1 << _bit );
	*(volatile uint8_t *)_port &= ~( 1 << _bit );
}


void setColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)		// LED 64EA = ( n = 0 ~ 63 )
{
	if(n < nCells)
	{
		if(brightness < 0xFF)
		{
			if(brightness > 0x01)
			{
				uint16_t cb = brightness + 1, t = 0;
    		
	    		t = (g|0x01) * cb;
	    		(t>0x80)?(g = (t-0x80) >> 8):(g = 0x00);
	    		t = (r|0x01) * cb;
	    		(t>0x80)?(r = (t-0x80) >> 8):(r = 0x00);
	    		t = (b|0x01) * cb;
	    		(t>0x80)?(b = (t-0x80) >> 8):(b = 0x00);
			}			
			else
			{
    			g = 0x00;
    			r = 0x00;
    			b = 0x00;
			}
		}
		uint8_t *p = &cells[n * 3];

		*p++ = g;
		*p++ = r;
		*p = b;
	}
}


void setColor_RGB(uint16_t n, uint32_t c)
{
	if(n < nCells)
	{
		uint8_t 
			r = (uint8_t)(c >> 16),
			g = (uint8_t)(c >>  8),
			b = (uint8_t)c;

		if(brightness < 0xFF)
		{
			if(brightness > 0x01)
			{
				uint16_t cb = brightness + 1, t = 0;
    		
	    		t = (g|0x01) * cb;
	    		(t>0x80)?(g = (t-0x80) >> 8):(g = 0x00);
	    		t = (r|0x01) * cb;
	    		(t>0x80)?(r = (t-0x80) >> 8):(r = 0x00);
	    		t = (b|0x01) * cb;
	    		(t>0x80)?(b = (t-0x80) >> 8):(b = 0x00);
			}			
			else
			{
    			g = 0x00;
    			r = 0x00;
    			b = 0x00;
			}
		}

		uint8_t *p = &cells[n * 3];

		*p++ = g;
		*p++ = r;
		*p = b;
	}
}


void show(void)
{
	rst_delay();								// 50us Delay
	asm volatile("cli");						// interrupt Disable

	volatile uint16_t loopcnt_i = nBytes;		// loop counter : led * 3 byte

	volatile uint8_t
		*ptr = cells,
		curbyte = *ptr++;

	volatile uint8_t next, bit, high, low;

	high = *port | maskPin;
	low = *port & ~maskPin;

	next = low;
	bit = 8;

	asm volatile(
		
		"run:"						"\n\t" 
		"st	%a[port], %[high]"		"\n\t"		
		"sbrc %[byte],	7"			"\n\t" 
		"mov  %[next],	%[high]"	"\n\t" 
		"st   %a[port],	%[next]"	"\n\t" 
		"mov  %[next],	%[low]"		"\n\t" 
		"dec  %[bit]"				"\n\t" 
		"breq nextByte"				"\n\t"
		"lsl  %[byte]"				"\n\t"		
		"rjmp  .+0"					"\n\t" 
		"nop"						"\n\t"
		"st   %a[port],	%[low]"		"\n\t" 
		"rjmp  .+0"					"\n\t" 
		"nop"						"\n\t"
		"rjmp run"					"\n\t"
		"nextByte:"					"\n\t" 
		"ld   %[byte],	%a[ptr]+"	"\n\t" 
		"st   %a[port],	%[low]"		"\n\t" 
		"ldi  %[bit],	8"			"\n\t" 
		"nop"						"\n\t" 
		"sbiw %[count],	1"			"\n\t" 
		"brne run"					"\n"   

		:[port]  "+e" (port),
		[byte]  "+r" (curbyte),
		[bit]   "+r" (bit),
		[next]  "+r" (next),
		[count] "+w" (loopcnt_i)
		: [ptr]   "e"  (ptr),
		[high]  "r"  (high),
		[low]   "r"  (low));

	asm volatile("sei");						// interrupt Enable
	rst_delay();								// 50us Delay	
}


void clear(void)
{
	memset(cells, 0, nBytes);
	show();
}


void cell_clear(uint16_t nCell)
{
	setColor(nCell, 0, 0, 0);
	show();
}



void setBrightness(uint8_t b)
{
	if( b != brightness )
	{
		uint8_t *ptr = cells;
		uint16_t nb = b + 1, cb = brightness + 1;
	
		if (b < 1)
		{
			for(uint16_t i = 0; i < nBytes; i++)
				(*ptr > 0) ? (*ptr++ = (((uint32_t)(*ptr + 1) << 16) / cb - 0x80) >> 8) : (*ptr++ = 0);
		}
		else if (b > 0xFE)
		{
			for (uint16_t i = 0; i < nBytes; i++)
				(*ptr > 0) ? (*ptr++ = (((uint32_t)(*ptr + 1) << 16) / cb - 0x80) >> 8) : (*ptr++ = 0);
		}
		else
		{
			for (uint16_t i = 0; i < nBytes; i++)
				(*ptr > 0) ? (*ptr++ = ((uint32_t)(*ptr + 1) * (nb << 8) / cb - 0x80) >> 8) : (*ptr++ = 0);
		}
		brightness = b;	
	}
}


uint32_t RGBtoColor(uint8_t r, uint8_t g, uint8_t b)
{
	return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}



void rainbow(uint16_t interval)
{
	uint16_t n = nCells;

	for(uint16_t j=0; j<255; j++) 
	{  // one cycle 
		for(uint16_t i=0; i<n; i++) 
		{
			uint8_t r_pos = (((i<<8) -  i) / n + j) % 0xFF;
			uint8_t sect = (r_pos / 0x55) % 0x03, pos = (r_pos % 0x55) * 0x03;

			switch(sect) 
			{
				case 0: 
					setColor_RGB(i, RGBtoColor(0xFF - pos, pos, 0x00)); break;

				case 1: 
					setColor_RGB(i, RGBtoColor(0x00, 0xFF - pos, pos)); break;

				case 2:
					setColor_RGB(i, RGBtoColor(pos, 0x00, 0xFF - pos)); break;
			}  
		}
	
		show();
		delay(interval);
	}
}

