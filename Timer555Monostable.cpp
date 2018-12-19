/*
  Timer555Monostable.cpp v.01 - 
  Version: 0.0.1
  Author: Jerome Drouin
  https://github.com/JeromeDrouin/Timer555Monostable
  Capacitive Meter Library for 'duino / Wiring
  Capacitance is derived via 555 Timer IC set via a Multivibrator in Monostable mode, and one Resistor R1
  Capacitance by default is expressed in NanoFarads 
	
	C = a.b.T / (1,1 . R) ; with T in seconds and C in nF
	a = 1E9		: 1,000,000,000 nano Farads in 1 Farad (FARADS_TO_NANOFARADS)
	b = 1E-6 	: 1,000,000 microseconds in one second (SECONDS_TO_MICROS)
  
  Credits: Library initially inspired by/ derived from "CapacitiveSensor.h" by Paul Bagder & Paul Stoffregen. 
	- Direct I/O through registers and bitmask (from OneWire library)
	- Thanks.
  Copyright (c) 2018 Jerome Drouin  All rights reserved.
  
*/


#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#include "WConstants.h"
#endif

#include "Timer555Monostable.h"

// Constructor /////////////////////////////////////////////////////////////////
// Function that handles the creation and setup of instances

Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t R1, float Biais)
{
	// initialize this instance's variables
	error = 1;

	#ifdef NUM_DIGITAL_PINS
	if (TriggerPin >= NUM_DIGITAL_PINS) error = -1;
	if (OutputPin >= NUM_DIGITAL_PINS) error = -1;
	#endif

	pinMode(TriggerPin, OUTPUT);					// TriggerPin to OUTPUT
	pinMode(OutputPin, INPUT);					// OutputPin to INPUT
	digitalWrite(TriggerPin, HIGH);					// Set Trigger HIGH to avoid trigger

	//Objects Parameters
	Resist_R1 = R1;
	Biais_Correction = Biais;
	
	//Reset
	Capacitance = 0;
	Period = 0;

	TriggerPin = _TriggerPin;
	OutputPin = _OutputPin;


	// get pin mapping and port for TriggerPin - from PinMode function in Wiring.c 
	sBit = PIN_TO_BITMASK(TriggerPin);					// get Trigger pin's ports and bitmask
	sReg = PIN_TO_BASEREG(TriggerPin);					// get pointer to output register

	// get pin mapping and port for OutPutPin - from digital pin functions in Wiring.c
	rBit = PIN_TO_BITMASK(OutputPin);					// get OutPut pin's ports and bitmask
	rReg = PIN_TO_BASEREG(OutputPin);					// get pointer to output register

}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

float Timer555Monostable::GetCapacitanceValue(uint8_t samples)
{
	// Set results to zero before start of read
	Frequency       = 0;
	Capacitance     = 0;
	Capacitance_Raw = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    	// loop for samples parameter - simple lowpass filter
		if (SingleRead() < 0)  return -2;   	// variable over timeout
	}
	// Capacitance is the average of all reads
	return Capacitance/samples;

}

float Timer555Monostable::GetCapacitanceValueRaw(uint8_t samples)
{
	// Set results to zero before start of read
	Frequency       = 0;
	Capacitance     = 0;
	Capacitance_Raw = 0;
	if (samples == 0) return 0;
	if (error < 0) return -1;                  	// bad pin - this appears not to work

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    	// loop for samples parameter - simple lowpass filter
		if (SingleRead() < 0)  return -2;   	// variable over timeout
	}
	// Capacitance_Raw is the average of all reads
	return Capacitance_Raw/samples;
}

float Timer555Monostable::GetLastFrequency(void)
{
	// Returns the last available calculated frequency
	return Frequency;
}


uint32_t Timer555Monostable::GetLastPeriod(void)
{
	// Returns the last available calculated Period in microseconds
	return Period;
}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

int Timer555Monostable::SingleRead(void) {
    noInterrupts();
	DIRECT_MODE_INPUT(rReg, rBit);	// OutputPin to input (pullups are off)
	DIRECT_MODE_OUTPUT(rReg, rBit); // OutputPin to OUTPUT
	DIRECT_WRITE_LOW(rReg, rBit);	// OutputPin is now LOW AND OUTPUT -> Cap discharges
	delayMicroseconds(10);
    
    	//Pulse Trigger Low
    	StartTimer = micros();	
    	DIRECT_WRITE_LOW(sReg, sBit);	// TriggerPin Register low -> 555 Trigger
    	delayMicroseconds(1);
    	DIRECT_WRITE_HIGH(sReg, sBit);	// TriggerPin Register high -> Stop Trigger pulse
    interrupts();
 
    //int total = 0;
    // while Output pin is HIGH 
    while (DIRECT_READ(rReg, rBit)) {
        //total++;
    }
    StopTimer   = micros()-1;	// empirical
        	
    // Calculate Capacitance	
    Period      	= StopTimer - StartTimer;
    Capacitance 	+= FARADS_TO_NANOFARADS/SECONDS_TO_MICROS * (Period*Biais_Correction) / (1.0986 * Resist_R1);
    Capacitance_Raw 	+= FARADS_TO_NANOFARADS/SECONDS_TO_MICROS * (Period) / (1.0986 * Resist_R1);
    Frequency   	= 1 / (Period/SECONDS_TO_MICROS);
    
    /*	
    Serial.print("\Period:");	
    Serial.print(Period);	
    Serial.print("\ttotal:");	
    Serial.print(total);	
    Serial.print("\t");	
    */

    //Return
    return 1;
}

// /////////////////////////////////////////////////////////////////////////////

