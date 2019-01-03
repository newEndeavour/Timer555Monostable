/*
  Timer555Monostable.cpp v.01 - 
  Version: 0.0.1
  Author: Jerome Drouin
  https://github.com/newEndeavour/Timer555Monostable
  Capacitive Meter Library for 'duino / Wiring
  Capacitance is derived via 555 Timer IC set via a Multivibrator in Monostable mode, and one Resistor R1
  Capacitance by default is expressed in NanoFarads 
	
	C = (a x b x T) / (1.1 x R1) ; with T in seconds and C in nF
	C		: Capacitance in nF
	R1		: Resistance in Ohms
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

Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _Biais)
{
	// initialize this instance's variables
	error = 1;

	#ifdef NUM_DIGITAL_PINS
	if (_TriggerPin >= NUM_DIGITAL_PINS) error = -1;
	if (_OutputPin >= NUM_DIGITAL_PINS) error = -1;
	#endif

	//Pins (Not required as we use registers)
	//TriggerPin 		=  _TriggerPin;
	//OutputPin 		=  _OutputPin;

	//Auto Calibration
	AutoCal_Millis 		= 20000;

	//Objects Parameters
	Resist_R1 		= _R1;
	Biais_Correction 	= _Biais;
		
	// get pin mapping and port for TriggerPin - from PinMode function in Wiring.c 
	sBit = PIN_TO_BITMASK(_TriggerPin);					// get Trigger pin's ports and bitmask
	sReg = PIN_TO_BASEREG(_TriggerPin);					// get pointer to output register

	// get pin mapping and port for OutPutPin - from digital pin functions in Wiring.c
	rBit = PIN_TO_BITMASK(_OutputPin);					// get OutPut pin's ports and bitmask
	rReg = PIN_TO_BASEREG(_OutputPin);					// get pointer to output register

	//Set Pins before start 
	DIRECT_MODE_OUTPUT(sReg, sBit); 				// TriggerPin to OUTPUT
	DIRECT_MODE_INPUT(rReg, rBit); 					// OutputPin to INPUT
    	DIRECT_WRITE_HIGH(sReg, sBit);					// TriggerPin high -> No unwanted Trigger pulse

}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

float Timer555Monostable::GetCapacitanceValue(uint8_t samples)
{
	// Set results to zero before start of read
	Period		= 0;
	Frequency       = 0;
	Capacitance     = 0;

	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    	// loop for samples parameter - simple lowpass filter
		if (OneCycle() < 0)  return -2;   	// variable over timeout
	}

	// Capacitance is the average of all reads
	return Capacitance/samples;

}


float Timer555Monostable::GetLastCapacitanceValueRaw()
{
	// Returns the last available calculated Capacitance Raw
	return Capacitance/Biais_Correction;
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

void Timer555Monostable::set_Biais_Correction(float _Biais_Correction)
{
	Biais_Correction = _Biais_Correction;
}


void Timer555Monostable::set_Autocal_Millis(unsigned long _AutoCal_Millis)
{
	AutoCal_Millis = _AutoCal_Millis;
}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

int Timer555Monostable::OneCycle(void) {
    
    noInterrupts();
    	//Pulse Trigger Low
    	DIRECT_WRITE_LOW(sReg, sBit);	// TriggerPin Register low -> 555 Trigger
    	//delayMicroseconds(1);		// Not required: DIRAC pulse duration approx 200ns without 1us delay
    	DIRECT_WRITE_HIGH(sReg, sBit);	// TriggerPin Register high -> Stop Trigger pulse    
    interrupts();
 
    StartTimer = micros();		// Start Timer
    //int total = 0;			// Note: We don't have to count cycles: wait for Flip-Flop to invert Voltage and O/P Pin to get Low
    // while Output pin is HIGH 
    while (DIRECT_READ(rReg, rBit)) {
        //total++;			
    }
    StopTimer   = micros();		// Stop Timer
        	
    // Calculate Capacitance	
    Period      	= StopTimer - StartTimer;
    Capacitance 	+= FARADS_TO_NANOFARADS/SECONDS_TO_MICROS * (Period*Biais_Correction) / (LOGNEPERIEN * Resist_R1);
    Frequency   	= 1 / (Period/SECONDS_TO_MICROS);
    
    /*	
    //DEBUG
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

