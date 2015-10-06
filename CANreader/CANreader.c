#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "MCP_defines.h"		//Sparkfun Shield with MCP2515
#include "NS_Rainbow_avr.h"		//Nulsom LED Matrix 8x8

#define MCPSCK				PB5
#define MCPMISO				PB4
#define MCPMOSI				PB3
#define	MCPSS				PB2
#define LED_DATA_PIN		PC, 0

#define LED_BRIGHTNESS		5
#define LED_COUNT			64

#define BLUE		0x4444dd
#define TURQ		0x00EEFF
#define GREEN		0x69d025
#define AMBER		0xccbb33
#define ORANGE		0xff9900
#define RED			0xff0000
#define PINK		0xff9999

#define SPEED500			1

#define ENGINE_RPM			0x0C
#define MAF_SENSOR          0x10
#define VEHICLE_SPEED		0x0D
#define BAROMETER			0x33
#define ECU_VOLTAGE			0x42

#define PID_REQUEST         0x7DF
#define PID_REPLY			0x7E8


//comment out this #define to remove usb serial monitoring stuff
//#define USBOUTDEBUG

#ifdef USBOUTDEBUG
#include <stdio.h>
char BUFFER[255];
#endif


typedef struct CanFrame{
	uint16_t id;
	struct {
		int8_t rtr : 1;
		uint8_t length : 4;
	} header;
	uint8_t data[8];
} CanFrame;

#ifdef USBOUTDEBUG
void usbInit(uint32_t baud) {
	uint32_t ubrr = (F_CPU/16UL)/baud - 1;
	UBRR0H = (unsigned char)ubrr>>8;
	UBRR0L = (unsigned char)ubrr;
	
	// Enable Rx/Tx pins
	UCSR0B = (1<<RXEN0) | (1<<TXEN0);
	
	// Frame format: 8 data + 1 stop bit
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	UCSR0A &= ~(1<<U2X0);
	_delay_us(10);
}

void usbWrite(char *string) {
	while(*string) {
		while(!(UCSR0A & (1<<UDRE0))); // wait for empty buffer
		UDR0 = *string++; //write to buffer and sends
	}
}
#endif


uint8_t SPIputc(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1<<SPIF)));
	return SPDR;
}


uint8_t mcpReadRegister(uint8_t regAddr) {
	uint8_t data;
	
	PORTB &= ~(1<<MCPSS);
	SPIputc(SPI_READ);
	SPIputc(regAddr);
	data = SPIputc(0xFF);
	PORTB |= (1<<MCPSS);
	
	return data;
}



void mcpWriteRegister(uint8_t regAddr, uint8_t writeByte) {
	PORTB &= ~(1<MCPSS);
	SPIputc(SPI_WRITE);
	SPIputc(regAddr);
	SPIputc(writeByte);
	PORTB |= (1<<MCPSS);
}


uint8_t canInit(uint32_t speed) {
	// Uno SPI pins setup
	DDRB |= (1<<MCPSCK) | (1<<MCPMOSI) | (1<<MCPSS);
	DDRB &= ~(1<<MCPMISO);
	PORTB &= ~((1<<MCPSCK) | (1<< MCPMOSI) | (1<<MCPMISO));
	PORTB |= (1<<MCPSS);
	
	// Uno SPI config
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	SPSR = 0;
	
	// Soft Reset MCP2515 chip
	PORTB &= ~(1<<MCPSS);
	SPIputc(SPI_RESET);
	PORTB |= (1<<MCPSS);
	_delay_us(50);
	
	// MCP2515 config
	PORTB &= ~(1<<MCPSS);
	SPIputc(SPI_WRITE);
	SPIputc(CNF3);
	SPIputc(1<<PHSEG21);
	SPIputc((1<<BTLMODE) | (1<<PHSEG11));
	SPIputc(speed);
	SPIputc((1<<RX1IE) | (1<<RX0IE));
	PORTB |= (1<<MCPSS);
	
	if (mcpReadRegister(CNF1) != speed) {
		return 0; //exit, retry
	}
	
	// Buffer High-Z
	mcpWriteRegister(BFPCTRL, 0);
	// Transmit start
	mcpWriteRegister(TXRTSCTRL, 0);
	// Filters
	mcpWriteRegister(RXB0CTRL, (1<<RXM1) | (1<<RXM0));
	mcpWriteRegister(RXB1CTRL, (1<<RXM1) | (1<<RXM0));
	// Exit Config mode
	mcpWriteRegister(CANCTRL, 0);
	
	return 1;
}

void makeReqPacket(CanFrame *packet, uint8_t pid) {
	packet->id = PID_REQUEST;
	packet->header.rtr = 0;
	packet->header.length = 8;
	packet->data[0] = 0x02;
	packet->data[1] = 0x01;
	packet->data[2] = pid;
	packet->data[3] = 0;
	packet->data[4] = 0;
	packet->data[5] = 0;
	packet->data[6] = 0;
	packet->data[7] = 0;
}

uint8_t mcpReadStatus(uint8_t type) {
	uint8_t data;
	
	PORTB &= ~(1<<MCPSS);
	SPIputc(type);
	data = SPIputc(0xFF);
	PORTB |= (1<<MCPSS);
	
	return data;
}


uint8_t mcpSendCanFrame(CanFrame packet) {
	uint8_t status = mcpReadStatus(SPI_READ_STATUS);
	uint8_t addr, idx;
	uint8_t length = packet.header.length & 0x0F;
	
	if (bit_is_clear(status, 2))		//TXB0
		addr = 0x00;
	else if (bit_is_clear(status, 4))	//TXB1
		addr = 0x02;
	else if (bit_is_clear(status, 6))	//TXB2
		addr = 0x04;
	else
		return 0;						//buffers full
		
		
	PORTB &= ~(1<<MCPSS);
	SPIputc(SPI_WRITE_TX | addr);
	SPIputc(packet.id >> 3);
	SPIputc(packet.id << 5);
	SPIputc(0);
	SPIputc(0);
	
	SPIputc(length);
	
	for (idx = 0; idx < length; idx++) {
		SPIputc(packet.data[idx]);
	}
	
	PORTB |= (1<<MCPSS);
	_delay_us(5);
	
	PORTB &= ~(1<<MCPSS);
	addr = (addr == 0) ? 1 : addr;
	SPIputc(SPI_RTS | addr);
	PORTB |= (1<<MCPSS);
	
	return addr;
}


// Seemed to be always true?
// uint8_t mcpHasData() {
// 	//return 1;
// 	//return (PIND & (1<<2));
// }


void mcpBitModify(uint8_t addr, uint8_t mask, uint8_t data) {
	PORTB &= ~(1<<MCPSS);
	SPIputc(SPI_BIT_MODIFY);
	SPIputc(addr);
	SPIputc(mask);
	SPIputc(data);
	PORTB |= (1<<MCPSS);
}



uint8_t mcpRead(CanFrame *packet) {
	uint8_t status = mcpReadStatus(SPI_RX_STATUS);
	uint8_t addr, idx, length;
	
	if (bit_is_set(status, 6)) {
		// message in buffer 0
		addr = SPI_READ_RX;
	}
	else if (bit_is_set(status, 7)) {
		// message in buffer 1
		addr = SPI_READ_RX | 0x04;
	}
	else {
		// no message
		return 0;
	}
	
	PORTB &= ~(1<<MCPSS);
	SPIputc(addr);
	
	//read ID
	packet->id = (uint16_t)SPIputc(0xFF)<<3;
	packet->id |= SPIputc(0xFF)>>5;
	SPIputc(0xFF);
	SPIputc(0xFF);
	
	//read DLC
	length = SPIputc(0xFF) & 0x0F;
	packet->header.length = length;
	packet->header.rtr = (bit_is_set(status, 3)) ? 1 : 0;
	
	//read data
	for (idx = 0; idx < length; idx++) {
		packet->data[idx] = SPIputc(0xFF);
	}
	PORTB |= (1<<MCPSS);
	
	//clear interrupts
	if (bit_is_set(status, 6)) {
		mcpBitModify(CANINTF, (1<<RX0IF), 0);
	}
	else {
		mcpBitModify(CANINTF, (1<<RX1IF), 0);
	}
	
	return (status & 0x07) + 1;
}



uint16_t readPID(uint8_t pid) {
	CanFrame request = {};
	CanFrame reply = {};
	uint16_t timeout = 0;
	uint16_t result = 0;
	
	makeReqPacket(&request, pid);
	
	mcpBitModify(CANCTRL, (1<<REQOP2) | (1<<REQOP1) | (1<<REQOP0), 0);
	
	if(!mcpSendCanFrame(request)) {
		#ifdef USBOUTDEBUG
		usbWrite("no send ");
		#endif
		return 0;
	}
	
	while (++timeout < 1023) {
 		if (mcpRead(&reply)) {
			_delay_ms(10);
			#ifdef USBOUTDEBUG
			usbWrite("reply ");
			#endif
			if (reply.id == PID_REPLY && reply.data[2] == pid) {
				switch (reply.data[2]) {
					case ENGINE_RPM:
					result = ((reply.data[3] * 256) + reply.data[4]) /4;
					#ifdef USBOUTDEBUG
					sprintf(BUFFER, "%d RPM | ", result);
					usbWrite(BUFFER);
					#endif
					return result;
						
					case VEHICLE_SPEED:
					result = reply.data[3];
					#ifdef USBOUTDEBUG
					sprintf(BUFFER, "%d km/h | ", result);
					usbWrite(BUFFER);
					#endif
					return result;
						
// 					case MAF_SENSOR:
// 					result = ((reply.data[3] * 256) + reply.data[4]) /100;
// 					#ifdef USBOUTDEBUG
// 					sprintf(BUFFER, "%d g/s(MAF) | ", result);
// 					usbWrite(BUFFER);
// 					#endif
// 					return result;
// 						
// 					case BAROMETER:
// 					result = reply.data[3];
// 					#ifdef USBOUTDEBUG
// 					sprintf(BUFFER, "%d kPa(Baro) | ", result);
// 					usbWrite(BUFFER);
// 					#endif
// 					return result;
// 						
// 					case ECU_VOLTAGE:
// 					result = ((reply.data[3] * 256) + reply.data[4]);
// 					#ifdef USBOUTDEBUG
// 					sprintf(BUFFER, "%d mV ", result);
// 					usbWrite(BUFFER);
// 					#endif
// 					return result;
						
					default:
					break;
				}
			}
			return 0;
		}
	}
	timeout = 0;
	return 0;
}


void ledInit() {
	NS_Rainbow_init(LED_COUNT, LED_DATA_PIN); //change value in #defines
	setBrightness(LED_BRIGHTNESS);
}


int drawBar(uint8_t loc, uint16_t metric, uint16_t type) {
	//splits LEDs into 2 vertical bars, grows upwards
	//loc = 0/1

	//setColor_RGB(LED, RRGGBB);

	uint8_t idx, height;
	uint8_t led = 56 + (loc*5);
	uint32_t color = 0xFFFFFF;
	
	
	switch (type) {
		case ENGINE_RPM: //RPM
		// filter out invalid values
		metric = metric < 0 ? 0 : metric;
		metric = metric > 8000 ? 8000 : metric;
		
		if (metric < 1400) {
			color = BLUE;
			height = metric/56;
		}
		else if (metric < 2800) {
			color = TURQ;
			height = (metric-1400)/56;
		}
		else if (metric < 4200) {
			color = GREEN;
			height = (metric-2800)/56;
		}
		else if (metric < 5600) {
			color = AMBER;
			height = (metric-4200)/56;
		}
		else if (metric < 7000) {
			color = ORANGE;
			height = (metric-5600)/56;
		}
		else { // more than 7000
			color = RED;
			height = (metric-7000)/56;
		}
		break;
		
		
		case VEHICLE_SPEED: //Speed (km/h) (16km = 10mi)
		metric = metric < 0 ? 0 : metric;
		metric = metric > 167 ? 167 : metric;
				
		switch(metric / 24) {
			case 0:
				color = BLUE;
				break;
			case 1:
				color = TURQ;
				break;
			case 2:
				color = GREEN;
				break;
			case 3:
				color = AMBER;
				break;
			case 4:
				color = ORANGE;
				break;
			case 5:
				color = RED;
				break;
			case 6:
				color = PINK;
				break;
		}
		
		height = metric % 24;
		break;
		
		default:
		return 0; //exit routine, dont display anything
	}
	
	for(idx = 0; idx < (height/3); idx++) {
		setColor_RGB(led - (idx*8), color);
		setColor_RGB(led + 1 - (idx*8), color);
		setColor_RGB(led + 2 - (idx*8), color);
	}
	
	for(idx = 7; idx > (height/3); idx--) {
		setColor_RGB(led - (idx*8), 0);
		setColor_RGB(led + 1 - (idx*8), 0);
		setColor_RGB(led + 2 - (idx*8), 0);
	}
	
	if (height % 3 == 1) {
		setColor_RGB(led - ((height-1)/3)*8, 0);
		setColor_RGB(led + 1 - ((height-1)/3)*8, 0);
		setColor_RGB(led + 2 - ((height-1)/3)*8, color);
	}
	else if (height % 3 == 2) {
		setColor_RGB(led - ((height-2)/3)*8, 0);
		setColor_RGB(led + 1 - ((height-2)/3)*8, color);
		setColor_RGB(led + 2 - ((height-2)/3)*8, color);
	}

	
	show();
	return 1;
}



int main(void) {
	//uint16_t rpm, speed;
	
	#ifdef USBOUTDEBUG	
	int cycles = 1;
	usbInit(9600);
	usbWrite("Initializing...\n\r");
	#endif
	
	while (!canInit(SPEED500)) {
		#ifdef USBOUTDEBUG
		usbWrite("Error. Retrying...\n\r");
		#endif
	}

	#ifdef USBOUTDEBUG
	usbWrite("MCP2515 Initialized!\n\r");
	usbWrite("--------------------------------------\n\r");
	#endif
	
	_delay_ms(10);
	ledInit();
	_delay_ms(500);
	
	uint16_t rpmtemp = 500, speedtemp = 0;	//from min
//	uint16_t rpmtemp = 7200, speedtemp = 130; //from max
	
	while(1) {
		#ifdef USBOUTDEBUG
		sprintf(BUFFER, "[%d]: ", cycles++);
		usbWrite(BUFFER);
		#endif
		
				
// 		rpm = readPID(ENGINE_RPM);
// 		drawBar(0, rpm, ENGINE_RPM);
// 		
// 		speed = readPID(VEHICLE_SPEED);
// 		drawBar(1, speed, VEHICLE_SPEED);

		
///* Run through for diagnostics

// 		char buffa[99];
// 
 		rpmtemp += 20;
// 		usbWrite("RPM: ");
// 		usbWrite(itoa(rpmtemp, buffa, 10));
 		drawBar(0, rpmtemp, ENGINE_RPM);
// 
 		speedtemp += 1;
// 		usbWrite(" Speed: ");
// 		usbWrite(itoa(speedtemp, buffa, 10));
 		drawBar(1, speedtemp, VEHICLE_SPEED);
// 		
 		rpmtemp = rpmtemp > 7200 ? 500 : rpmtemp;
 		speedtemp = speedtemp > 130 ? 0 : speedtemp;
 		
//*/// ------ diagnostic

// NOT IMPLEMENTED	
// 		readPID(MAF_SENSOR);
// 		readPID(BAROMETER);
// 		readPID(ECU_VOLTAGE);
		
		_delay_ms(100);
		
		#ifdef USBOUTDEBUG
		//usbWrite("\n\n");
		usbWrite("\r");
		#endif
	}
}