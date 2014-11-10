/*
 * Senors.h
 *
 * Created: 11/5/2014 1:39:42 PM
 *  Author: jpagel
 */ 


#ifndef SENORS_H_
#define SENORS_H_

#include "definitions.h"

volatile uint8_t motionStatus = 0;


int ReadMotion()
{
	motionStatus = 0;
	 //Just incase set motion pin to input
	 if(MOTION_PIN & (1<<MOTION_PIN_NUM))
	 {
		 motionStatus = 1;
	 }	
	return motionStatus;	
}

#if MOTION_ENABLE_ISR
volatile uint8_t MotionDetected = 0;
void configureMotionISR()
{
	cli();
	EICRA |= (1<<ISC10) | (1<<ISC11); //Set for Rising Edge trigger
	EIMSK |= (1<<INT1);
	
}


//ISR for Motion INT1
ISR(INT1_vect)
{
	cli();
	motionStatus = 1;
	MotionDetected = 1;
	EIMSK &= ~(1<<INT1); //Disable Interupt untill it has a chance to transmit, is re-enabled after transmit
	sleep_disable();
	sei();
	
}

#endif

#endif /* SENORS_H_ */