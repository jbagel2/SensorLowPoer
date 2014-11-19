/*
 * Serial_USART.h
 *
 * Created: 10/18/2014 11:46:13 PM
 *  Author: jpagel
 */ 


#ifndef SERIAL_USART_H_
#define SERIAL_USART_H_

#include "definitions.h"

class Serial
{
	public:
	uint8_t UART_RX_Data_Waiting;
	void static sendChar(char c);
	void static sendString(const char *text);
	void getReceivedData();
	char static usart_receiveByte();
	void static begin();
	#if defined(__AVR_ATXmega128D3__)
	#include "ProgMemData.h"
	void SerialBeginXMEGA(baud_t baud);	
	#endif
	
	
	};


#endif /* SERIAL_USART_H_ */