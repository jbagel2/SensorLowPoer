/*
 * SensorLowPower.cpp
 *
 * Created: 11/4/2014 9:20:40 PM
 *  Author: jpagel
 *  
 *  
 *  Current iteration uses watchdog timer to wake up from sleep ever 8Sec x WDT_Multiplyer Count
 *  
 */ 

#include "stdio.h"

#include "definitions.h"
#include <util/delay.h>
#include <avr/io.h>
#include "dht22New.h"
#include "Serial_USART.h"
#include "Analog.h"
#include "PowerSave.h"
#include "Senors.h"


char buffer[28];
Serial serial;

uint8_t justTriggeredByMotionISR = 0;

void CheckMotion()
{
	if(MotionDetected)
	{
		//MotionDetected = 0;
		justTriggeredByMotionISR = 1;
		sprintf(buffer,"M:%d\r\n",1);
		serial.sendString(buffer);
		MotionDetected = 0;
		_delay_ms(150); // wait for transmission to finish		
		EIMSK |= (1<<INT1);
	}
	
}



int main(void)
{
	
	
	DDRD &= ~(1<<3); //set motion Pin as input
	PORTD &= ~(1<<3); //Make sure pullup is disabled.
	WDTPowerSave_RunAtSystemStart();
	//cli();
	PowerReduction();
	init(); //Starts timer for millis and micro and delay()
	delay(250);
	
	
	serial.begin();
	uint16_t light = 0;
	uint8_t motionData = 0;
	serial.sendString("Boot Complete\r\n");
	double Ftemp = 0.0;
	//cli();
	//SetWDT();
	DHT22_DATA_t sensor_values;
	configureMotionISR();
	//DHT22_ERROR_t error;
    for(;;)
	{		
		if(!justTriggeredByMotionISR)
		{ 
		 cli();
		 readDHT22(&sensor_values);		 
		 sei();
		 light = ADCsingleREAD(0);
		 motionData = ReadMotion();
		 Ftemp = (((float)sensor_values.raw_temperature / 10.0) * 1.8) + 32;
		 sprintf(buffer,"F:%.2f,H:%.2f,L:%d,M:%d\r\n", Ftemp,(double)sensor_values.raw_humidity / 10, light,motionData);
		 serial.sendString(buffer);
		}
		else
		{
			justTriggeredByMotionISR = 0;
		}
		 Reset_WDT_Timer();
		 GotToSleepAndWaitForWork();
		 CheckMotion();
				
	}
}

