/*
 * PowerSave.h
 *
 * Created: 11/4/2014 10:47:10 PM
 *  Author: jpagel
 */ 


#ifndef POWERSAVE_H_
#define POWERSAVE_H_
#include "wiring8.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

const static uint8_t WDT_Multiplyer = 4;


void PowerReduction()
{
	PRR |= (1<<PRTIM2) | (1<<PRTWI);
}

void SetINT0Int()
{
	EIMSK |= (1<<INT1);
	EIFR |= (1<<INTF1);
}

void GotToSleepAndWaitForWork()
{
	_delay_ms(250);
	cli();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	sleep_enable();
	sei();

	sleep_cpu();
	cli();
	sleep_disable();

	_delay_ms(12);
	sei();
}

void WDTPowerSave_RunAtSystemStart()
{
	cli();
	
	wdt_enable(WDTO_8S);
	WDTCSR |= (1<<WDCE) | (1<<WDIE);
	//WDTCSR |= (1<<WDCE) | (1<<WDP0) | (1<<WDP3);
	
	//wdt_disable();
	//wdt_reset();
	
	//MCUSR = 0;
	//sei();
	
}

void SetWDT()
{
	wdt_enable(WDTO_8S);	
}

void Reset_WDT_Timer()
{
	wdt_reset();
	
}

volatile uint8_t wdt_triggered = 0;


uint8_t WDTTriggerCount = 1;

ISR(WDT_vect)
{
	
	cli();
	
	//MCUSR = 0;
	WDTCSR |= (1<<WDCE) | (1<<WDIE);
	WDTCSR &= ~(1<<WDIF);
	
	//wdt_reset();
	//WDTCSR |= (1<<WDIE);
	//WDTCSR = (0<<WDIF);
	wdt_triggered = 1;
	if(WDTTriggerCount < WDT_Multiplyer)
	{
		WDTTriggerCount++;
		GotToSleepAndWaitForWork();
	}
	if(WDTTriggerCount >= WDT_Multiplyer)
	{
		WDTTriggerCount = 1;
	}
	sei();
	//_delay_ms(150);
}


#endif /* POWERSAVE_H_ */