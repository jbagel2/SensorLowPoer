/*
* wiring44A.cpp
*
* Created: 8/14/2014 9:34:25 AM
* Author: jpagel
*/


#include "wiring8.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/fuse.h>


// the prescaler is set so that timer0 ticks every 64 clock cycles, and the
// the overflow handler is called every 256 ticks.
#define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))

// the whole number of milliseconds per timer0 overflow
#define MILLIS_INC (MICROSECONDS_PER_TIMER0_OVERFLOW / 1000)

// the fractional number of milliseconds per timer0 overflow. we shift right
// by three to fit these numbers into a byte. (for the clock speeds we care
// about - 8 and 16 MHz - this doesn't lose precision.)
#define FRACT_INC ((MICROSECONDS_PER_TIMER0_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer0_overflow_count = 0;
volatile unsigned long timer0_millis = 0;
volatile unsigned char timer0_fract = 0;



#if defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
ISR(TIM0_OVF_vect)
#else
ISR(TIMER0_OVF_vect)
#endif
{
	// copy these to local variables so they can be stored in registers
	// (volatile variables must be read from memory on every access)
	unsigned long m = timer0_millis;
	unsigned char f = timer0_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m += 1;
	}

	timer0_fract = f;
	timer0_millis = m;
	timer0_overflow_count++;
}

unsigned long millis()
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	cli();
	m = timer0_millis;
	SREG = oldSREG;
	sei();
	return m;
}

unsigned long micros() {
	unsigned long m;
	uint8_t oldSREG = SREG, t;
	
	cli();
	m = timer0_overflow_count;
	#if defined(TCNT0)
	t = TCNT0;
	#elif defined(TCNT0L)
	t = TCNT0L;
	#else
	#error TIMER 0 not defined
	#endif

	
	#ifdef TIFR0
	if ((TIFR0 & _BV(TOV0)) && (t < 255))
	m++;
	#else
	if ((TIFR & _BV(TOV0)) && (t < 255))
	m++;
	#endif

	SREG = oldSREG;
	
	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

void delay(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();

	while (ms > 0) {
		if (((uint16_t)micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

/* Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. */
/* Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. */
void delayMicroseconds(unsigned int us)
{
	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
	#if F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply wait 2 cycle and return. The overhead
	// of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
	"nop" "\n\t"
	"nop"); //just waiting 2 cycle
	if (--us == 0)
	return;

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us<<2) + us; // x5 us

	// account for the time taken in the preceeding commands.
	us -= 2;

	#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
	return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;
	#else
	// for the 8 MHz internal clock on the ATmega168

	// for a one- or two-microsecond delay, simply return.  the overhead of
	// the function calls takes more than two microseconds.  can't just
	// subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
	return;
	if (--us == 0)
	return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1;
	
	// partially compensate for the time taken by the preceeding commands.
	// we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
	#endif

	// busy wait
	__asm__ __volatile__ (
	"1: sbiw %0,1" "\n\t" // 2 cycles
	"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}

void init()
{
	// this needs to be called before setup() or some functions won't
	// work there
	sei();
	
	///#if defined(__AVR_ATMEGA328P__) 
	//Setting up timer
	//328
	#if defined(TCCR0A) && defined(WGM01)
	TCCR0A |= (1<<WGM01) | (1<<WGM00);
	//#if F_CPU==(16000000UL)
	TCCR0B |= (1<<CS01) | (1<<CS00);
	//#elif F_CPU==(1000000UL)
	//TCCR0B |= (0<<CS01) | (1<<CS00);
	//#endif
	
	//Enabling Timer OVF Interupt
	TIMSK0 |= (1<< TOIE0);

	#endif
	// set timer 0 prescale factor to 64

	#if defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the standard 168/328/1280/2560
	TCCR0 |= (1<<CS01) | (1<<CS00);
	//sbi(TCCR0B, CS01);
	//sbi(TCCR0B, CS00);
	#elif defined(TCCR0) && defined(CS01) && defined(CS00)
	// this combination is for the __AVR_ATmega645__ series
	TCCR0A |= (1<<CS01) | (1<<CS00);
	//sbi(TCCR0A, CS01);
	//sbi(TCCR0A, CS00);

	#endif

	// enable timer 0 overflow interrupt
	#if defined(TIMSK) && defined(TOIE0)
	//sbi(TIMSK, TOIE0);
	//#if defined(TIMSK0) && defined(TOIE0)
	TIMSK |= (1<<TOIE0);
	//sbi(TIMSK0, TOIE0);
	#else	
	#endif
}

unsigned int makeWord(unsigned int w) { return w; }
unsigned int makeWord(unsigned char h, unsigned char l) { return (h << 8) | l; }
// Get fuse Data
