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
#define USE_NRF24L01 0
#define USE_ESP8266 0



#include "stdio.h"

#include "definitions.h"
#include <util/delay.h>
#include <avr/io.h>
#include "dht22New.h"
#include "Serial_USART.h"
#include "Analog.h"
#include "PowerSave.h"
#include "Senors.h"
#include "VirtualWireTxShrunk.h"
#include <string.h>

#if USE_NRF24L01
#include "nrf24.h"
#else

#endif


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
		vw_send((uint8_t *)buffer, strlen(buffer));
		MotionDetected = 0;
		_delay_ms(150); // wait for transmission to finish		
		EIMSK |= (1<<INT1);
	}
	
}

//NRF24 references

//NRF24 references

int main(void)
{
	#if USE_NRF24L01
	uint8_t temp;
	uint8_t q = 0;
	uint8_t data_array[4];
	uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
	uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
	
	nrf24_init();
	nrf24_config(2,10);
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);
	#endif
	
	
	DDRD &= ~(1<<3); //set motion Pin as input
	PORTD &= ~(1<<3); //Make sure pullup is disabled.
	WDTPowerSave_RunAtSystemStart();
	PowerReduction();
	init(); //Starts timer for millis and micro and delay()
	delay(250);
	vw_setup(1500);
	
	serial.begin();
	uint16_t light = 0;
	uint8_t motionData = 0;
	serial.sendString("Boot Complete\r\n");
	double Ftemp = 0.0;
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
		 sprintf(buffer,"F:%.2f,H:%.2f,L:%d\r\n", Ftemp,(double)sensor_values.raw_humidity / 10, light);
		 serial.sendString(buffer);
		 vw_send((uint8_t *)buffer, strlen(buffer));
		 //vw_wait_tx();
		 #if USE_NRF24L01
		 //Convert data to byte array for nrf24
		 nrf24_send(data_array);
		 
		 /* Wait for transmission to end */
		 while(nrf24_isSending());
		 temp = nrf24_lastMessageStatus();
		 temp = nrf24_retransmissionCount();
		 nrf24_powerUpRx();
		 #endif
		 
		 /* Or you might want to power down after TX */
		 // nrf24_powerDown();
		 
		 
		 
		}
		//if(serial.UART_RX_Data_Waiting)
		//{
			////serial.RxBuf
			////serial.sendString(serial.getReceivedData());
		//}
		else
		{
			justTriggeredByMotionISR = 0;
		}
		 Reset_WDT_Timer();
		 GotToSleepAndWaitForWork();
		 CheckMotion();
				
	}
}

