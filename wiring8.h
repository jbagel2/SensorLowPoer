/* 
* wiring44A.h
*
* Created: 8/14/2014 9:34:25 AM
* Author: jpagel
*/

#include "definitions.h"


#ifndef WIRING8_H_
#define WIRING8_H_


#include <avr/io.h>
//#include <stdlib.h>

extern "C"{
	

	#define HIGH 0x1
	#define LOW  0x0

	#define INPUT 0x0
	#define OUTPUT 0x1

	#define true 0x1
	#define false 0x0

	#define PI 3.1415926535897932384626433832795
	#define HALF_PI 1.5707963267948966192313216916398
	#define TWO_PI 6.283185307179586476925286766559
	#define DEG_TO_RAD 0.017453292519943295769236907684886
	#define RAD_TO_DEG 57.295779513082320876798154814105

	#define SERIAL  0x0
	#define DISPLAY 0x1

	#define LSBFIRST 0
	#define MSBFIRST 1

	#define CHANGE 1
	#define FALLING 2
	#define RISING 3

	#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	#define INTERNAL1V1 2
	#define INTERNAL2V56 3
	#else
	#define INTERNAL 3
	#endif
	#define DEFAULT 1
	#define EXTERNAL 0

	// undefine stdlib's abs if encountered
	#ifdef abs
	#undef abs
	#endif


	//Fuse Masks
	


	//#define min(a,b) ((a)<(b)?(a):(b))
	//#define max(a,b) ((a)>(b)?(a):(b))
	//#define abs(x) ((x)>0?(x):-(x))
	//#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
	//#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
	//#define radians(deg) ((deg)*DEG_TO_RAD)
	//#define degrees(rad) ((rad)*RAD_TO_DEG)
	//#define sq(x) ((x)*(x))
//
	//#define interrupts() sei()
	//#define noInterrupts() cli()

	#define clockCyclesPerMicrosecond() ( F_CPU / 1000000UL )
	#define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
	#define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )

	#define lowByte(w) ((uint8_t) ((w) & 0xff))
	#define highByte(w) ((uint8_t) ((w) >> 8))

	#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
	#define bitSet(value, bit) ((value) |= (1UL << (bit)))
	#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
	#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))


	//#define digitalWrite(portChar, pinNum, STATE) ((PORT ##))


	typedef unsigned int word;

	#define bit(b) (1UL << (b))

	typedef uint8_t boolean;
	typedef uint8_t byte;

	void init(void);

	//void pinMode(uint8_t, uint8_t);
	//void digitalWrite(uint8_t pin, uint8_t val);
	//int digitalRead(uint8_t);
	//int analogRead(uint8_t);
	//void analogReference(uint8_t mode);
	//void analogWrite(uint8_t, int);

	unsigned long millis(void);
	unsigned long micros(void);
	void delay(unsigned long);
	void delayMicroseconds(unsigned int us);
	//unsigned long pulseIn(uint8_t pin, uint8_t state, unsigned long timeout);

	//void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val);
	//uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);

	//void attachInterrupt(uint8_t, void (*)(void), int mode);
	//void detachInterrupt(uint8_t);

	//void setup(void);
	//void loop(void);
	
//#define digitalPinToBitMask(P) ( pgm_read_byte( digital_pin_to_bit_mask_PGM + (P) ) )
//#define digitalPinToTimer(P) ( pgm_read_byte( digital_pin_to_timer_PGM + (P) ) )
//#define analogInPinToBit(P) (P)
//#define portOutputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_output_PGM + (P))) )
//#define portInputRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_input_PGM + (P))) )
//#define portModeRegister(P) ( (volatile uint8_t *)( pgm_read_word( port_to_mode_PGM + (P))) )

	//#ifdef __cplusplus
} // extern "C"
//#endif

uint16_t makeWord(uint16_t w);
uint16_t makeWord(byte h, byte l);

#define word(...) makeWord(__VA_ARGS__)

#endif
