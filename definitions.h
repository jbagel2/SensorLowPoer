/*
 * definitions.h
 *
 * Created: 11/4/2014 9:27:10 PM
 *  Author: jpagel
 */ 


#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

#include <avr/io.h>

#define BAUD 57600
#define F_CPU 8000000UL


//Motion Sensor
#define MOTION_PIN_NUM PD3
#define MOTION_PIN PIND
#define MOTION_PORT PORTD
#define MOTION_DDR DDRD

#define MOTION_ENABLE_ISR 1






//Light Sensor


// DHT22 (Temp/Humid)



// Wifi



// 433TxRx



//Baromentric presssure


#endif /* DEFINITIONS_H_ */