/*
 * Serial_USART.cpp
 *
 * Created: 10/18/2014 11:45:25 PM
 *  Author: jpagel
 */ 


#include <avr/io.h>
#include "Serial_USART.h"
#include "ProgMemData.h"
#include <util/setbaud.h>
#include <string.h>
//#include <util//delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>



//circular buffer size
#define BUF_SIZE 128

static volatile uint8_t TxBuf[BUF_SIZE];
//buffer mem address (for circular buffer)
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;


#define UART_TX0_BUFFER_MASK ( BUF_SIZE - 1)




#ifndef F_CPU
#error You Must declare F_CPU
#endif

#ifndef BAUD
#error You must define BAUD to your desired BAUDrate
#endif

//int main(void)
//{
	//
//}

#if defined(__AVR_ATXmega128D3__) || defined(__AVR_ATXmega64A1U__) //Really any XMEGA Since they are all pretty much much compatible minus USB variations.

void Serial::sendChar(char c)
{
	
	while( !(USARTC0_STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
	
	USARTC0_DATA = c;
	
}

void Serial::sendString(char *text)
{
	while(*text)
	{
		sendChar(*text++);
	}
}

char Serial::receiveByte() //Blocking
{
	while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Interesting DRIF didn't work.
	return USARTC0_DATA;
}

void Serial::SerialBeginXMEGA(baud_t baud)
{
	//baud_t temp = baud;
	
	int16_t scaler = baud.Scaler;
	int32_t bsel = baud.Bsel;
	int16_t doubleSpeed = baud.DoubleSpeed;
	
	USARTC0_BAUDCTRLB = USARTC0.BAUDCTRLB = 0| (bsel >> 8) | (scaler << USART_BSCALE0_bp);
	USARTC0_BAUDCTRLA = bsel;
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	
	if(doubleSpeed)
	{
		USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm | USART_CLK2X_bm;
	}
	else
	{
		USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm | ~USART_CLK2X_bm;
	}


}

#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmegaA__) || defined(__AVR_ATmega168P__)

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)

//#undef UDR
//#undef UCSRA
//#undef UCSRB
//#undef UDRE
//#undef UBRRH
//#undef UBRRL
//#undef U2X
//#undef RXEN
//#undef TXEN
//#undef TXC
//#undef RXC
#define UDR UDR0
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UDRE UDRE0
#define UBRRH UBRR0H
#define UBRRL UBRR0L
#define U2X U2X0
#define RXEN RXEN0
#define TXEN TXEN0
#define TXC TXC0
#define RXC RXC0


#endif

void add_charToBuffer(uint8_t data)
{
	uint16_t tmphead;

	tmphead  = (UART_TxHead + 1) & UART_TX0_BUFFER_MASK;

	while ( tmphead == UART_TxTail );/* wait for free space in buffer */	

	TxBuf[tmphead] = data;
	UART_TxHead = tmphead;

	/* enable UDRE interrupt */
	UCSRB    |= (1<<UDRIE0);

} /* uart0_putc */
	
void uart0_puts(const char *s )
{
	while (*s) {
		add_charToBuffer(*s++);
	}
} /* uart0_puts */

void configureSerial()
{
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2X
	UCSRA |= (1<<U2X);
	#else
	UCSRA &= ~(1<<U2X);	
	#endif	
}

void startSerial()
{
	UART_TxHead = 0;
	UART_TxTail = 0;
	//UART_RxHead = 0;
	//UART_RxTail = 0;
	//Enables Both RX and TX
	UCSRB |= (1<<RXEN) | (1<<TXEN);
	
	//Enables RX Complete Interupt
	UCSR0B |= (1<<RXCIE0);
	sei();
}


void Serial::sendChar(char c) {

	
	while(!(UCSRA & (1<<UDRE)));
	UDR = c;

	 /* Wait until transmission ready. */
}

char *buf;
char *currentBuffer;
uint8_t bufSize;

uint8_t charPosition;


char *textBuf;
void Serial::sendString(const char *text)
{
	
	uart0_puts(text);
	sei();

	UCSR0B |= (1<<UDRIE0);
}

char receiveByte() {
	loop_until_bit_is_set(UCSRA, RXC); /* Wait until data exists. */
	return UDR;
}




void Serial::begin()
{
	configureSerial();
	startSerial();
	
}


#endif


	

uint8_t lastAddress =0;
//ISR(USART_UDRE_vect)
//{
////UCSR0B &= ~(1<<UDRIE0);	
	//if(TxBuf[lastAddress] != '\0')
	//{
		//
		//
		//UDR = TxBuf[lastAddress];
		//lastAddress++;
		////charPosition++;
		////UCSR0B |= (1<<UDRIE0);	
	//}
	//else
	//{
		//cli();
		//lastAddress =0;
		////UCSR0B &= ~(1<<UDRIE0);		
	//}
	//
	//
	//
//}

#define UART0_TRANSMIT_INTERRUPT USART_UDRE_vect


ISR(USART_RX_vect)
{
	

	uint8_t incomming;
	cli();
	UCSRA &= ~(1<<RXC);
	
	incomming = UDR;
	UCSR0A &= ~(1<<RXC0);
	incomming = UDR0;


	add_charToBuffer(incomming);
	add_charToBuffer('\r');
	sleep_disable();
	
	
}



ISR(UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
	uint16_t tmptail;

 	if ( UART_TxHead != UART_TxTail) {
		/* calculate and store new buffer index */
		tmptail = (UART_TxTail + 1) & UART_TX0_BUFFER_MASK;
		UART_TxTail = tmptail;
		/* get one byte from buffer and write it to UART */
		UDR = TxBuf[tmptail];  /* start transmission */
		} else {
		/* tx buffer empty, disable UDRE interrupt */
		UCSRB &= ~(1<<UDRIE0);		
	}
}