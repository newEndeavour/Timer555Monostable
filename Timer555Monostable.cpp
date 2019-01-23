/*
  File:         Timer555Monostable.cpp
  Version:      0.0.5
  Date:         19-Dec-2018
  Revision:     23-Jan-2019
  Author:       Jerome Drouin (jerome.p.drouin@gmail.com)

  Editions:	Please go to Timer555Monostable.h for Edition Notes.

  https://github.com/newEndeavour/Timer555Monostable

  Capacitive &/or Resistance Meter Library for 'duino / Wiring
  Capacitance &/or Resistance is derived via 555 Timer IC in Monostable mode, Resistor R1 (Cap Meter mode) 
  or Capacitance C1 (Res Meter mode). 
  Capacitance by default is expressed in NanoFarads. 
  Resistance by default is expressed in Ohms. 
	
	C = (a / b x T) / (1.1 x R1) ; with T in seconds and C in nF
	C		: Capacitance in nF
	R1		: Resistance in Ohms
	a = 1E9		: 1,000,000,000 nano Farads in 1 Farad (FARADS_TO_NANOFARADS)
	b = 1E6 	: 1,000,000 microseconds in one second (SECONDS_TO_MICROS)
  
	R = (c / b x T) / (1.1 x C1) ; with T in seconds and C in pF
	R		: Resistance in Ohms
	C1		: Capacitance in pF
	c = 1E12	: 1,000,000,000,000 pico Farads in 1 Farad (FARADS_TO_PICOFARADS)

  Credits: 
        - Library initially inspired by/ derived from "CapacitiveSensor.h" by 
  	  Paul Bagder & Paul Stoffregen. Thanks.
	- Direct I/O through registers and bitmask (from OneWire library)

  Copyright (c) 2018-2019 Jerome Drouin  All rights reserved.
  
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res)
{
	// initialize this instance's variables
	error = 1;

	#ifdef NUM_DIGITAL_PINS
	if (_TriggerPin >= NUM_DIGITAL_PINS) error = -1;
	if (_OutputPin >= NUM_DIGITAL_PINS) error = -1;
	#endif

	//Auto Calibration
	AutoCal_Millis 		= 20000;

	//Objects Parameters
	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= _Baseline_Cap;			// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= _Baseline_Res;			// if not in use, input 0.0
		
	// get pin mapping and port for TriggerPin - from PinMode function in Wiring.c 
	sBit = PIN_TO_BITMASK(_TriggerPin);				// get Trigger pin's ports and bitmask
	sReg = PIN_TO_BASEREG(_TriggerPin);				// get pointer to output register

	// get pin mapping and port for OutPutPin - from digital pin functions in Wiring.c
	rBit = PIN_TO_BITMASK(_OutputPin);				// get OutPut pin's ports and bitmask
	rReg = PIN_TO_BASEREG(_OutputPin);				// get pointer to output register

	hasDischargePin = 0;

	//Set Pins before start 
	DIRECT_MODE_OUTPUT(sReg, sBit); 				// TriggerPin to OUTPUT
	DIRECT_MODE_INPUT(rReg, rBit); 					// OutputPin to INPUT
    	DIRECT_WRITE_HIGH(sReg, sBit);					// TriggerPin high -> No unwanted Trigger pulse

}


Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint8_t _DischargePin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res)
{
	// initialize this instance's variables
	error = 1;

	#ifdef NUM_DIGITAL_PINS
	if (_TriggerPin >= NUM_DIGITAL_PINS) error = -1;
	if (_OutputPin >= NUM_DIGITAL_PINS) error = -1;
	#endif

	//Auto Calibration
	AutoCal_Millis 		= 20000;

	//Objects Parameters
	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= _Baseline_Cap;			// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= _Baseline_Res;			// if not in use, input 0.0
		
	// get pin mapping and port for TriggerPin - from PinMode function in Wiring.c 
	sBit = PIN_TO_BITMASK(_TriggerPin);				// get Trigger pin's ports and bitmask
	sReg = PIN_TO_BASEREG(_TriggerPin);				// get pointer to output register

	// get pin mapping and port for OutPutPin - from digital pin functions in Wiring.c
	rBit = PIN_TO_BITMASK(_OutputPin);				// get OutPut pin's ports and bitmask
	rReg = PIN_TO_BASEREG(_OutputPin);				// get pointer to output register

	//Set Pins before start 
	DIRECT_MODE_OUTPUT(sReg, sBit); 				// TriggerPin to OUTPUT
	DIRECT_MODE_INPUT(rReg, rBit); 					// OutputPin to INPUT
    	DIRECT_WRITE_HIGH(sReg, sBit);					// TriggerPin high -> No unwanted Trigger pulse

	hasDischargePin = 1;

	// get pin mapping and port for DischargePin - from digital pin functions in Wiring.c
	dBit = PIN_TO_BITMASK(_DischargePin);				// get Discharge pin's ports and bitmask
	dReg = PIN_TO_BASEREG(_DischargePin);				// get pointer to output register
	DIRECT_MODE_INPUT(dReg, dBit); 					// DischargePin to INPUT -> Cap. charging possible

}


// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries
// Returns Calculated Capacitance
float Timer555Monostable::GetCapacitance(uint8_t samples)
{

	// Set results to zero before start of read
	Period		= 0;
	Frequency       = 0;
	Capacitance     = 0;
	Total	= 0;

	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    	// loop for samples parameter - simple lowpass filter
		if (OneCycle_Capacitance() < 0)  return -2;   	// variable over timeout
	}

	//Updates other variables
	Total 		= (uint32_t)(Total / samples);
	Frequency	= Frequency / samples;
	Period		= Period / samples;
	Capacitance	= (Capacitance / samples) - Baseline_Cap;

	// Capacitance is the average of all reads
	return Capacitance;

}

// Returns Calculated Resistance
float Timer555Monostable::GetResistance(uint8_t samples)
{

	// Set results to zero before start of read
	Period		= 0;
	Frequency       = 0;
	Resistance      = 0;
	Total	= 0;

	if (samples == 0) return 0;
	if (error < 0) return -1;            // bad pin

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    	// loop for samples parameter - simple lowpass filter
		if (OneCycle_Resistance() < 0)  return -2;   	// variable over timeout
	}

	//Updates other variables
	Total 		= (uint32_t)(Total / samples);
	Frequency	= Frequency / samples;
	Period		= Period / samples;
	Resistance	= (Resistance / samples) - Baseline_Res;

	// Resistance is the average of all reads
	return Resistance;

}


float Timer555Monostable::GetBaseline_Cap()
{
	// Returns the Baseline_Cap Parameters
	return Baseline_Cap;

}

float Timer555Monostable::GetBaseline_Res()
{
	// Returns the Baseline_Res Parameters
	return Baseline_Res;

}

float Timer555Monostable::GetLastCapacitance()
{
	// Returns the last available calculated Capacitance
	return Capacitance;

}

float Timer555Monostable::GetLastResistance()
{
	// Returns the last available calculated Resistance
	return Resistance;

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

uint32_t Timer555Monostable::GetLastTotal(void)
{
	// Returns the last available calculated GetLastTotal
	return Total;
}


void Timer555Monostable::set_Autocal_Millis(unsigned long _AutoCal_Millis)
{
	AutoCal_Millis = _AutoCal_Millis;
}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library
long Timer555Monostable::RunTimer(void) {

    noInterrupts();
	if (hasDischargePin) {
		//---- Discharging Capacitor --------
		DIRECT_MODE_INPUT(dReg, dBit);	// DischargePin to INPUT (pullups are off)
		DIRECT_MODE_OUTPUT(dReg, dBit); // DischargePin to OUTPUT
		DIRECT_WRITE_LOW(dReg, dBit);	// pin is now LOW AND OUTPUT
		delayMicroseconds(2);		// usually, takes no more than a few nanoSeconds: 2us is plenty of time...
		DIRECT_MODE_INPUT(dReg, dBit);	// DischargePin to INPUT (pullups are off)
	}
	
	//---- Pulse ------------------------	
    	DIRECT_WRITE_LOW(sReg, sBit);	// TriggerPin Register low -> 555 Trigger
    					// No add. delay required: DIRAC pulse duration approx 200ns
    	//Stop Pulse
	DIRECT_WRITE_HIGH(sReg, sBit);	// TriggerPin Register high -> Stop Trigger pulse    
    interrupts();
 
    StartTimer 		= micros();		// Start Timer 
    //while (condition==true) {instructions} approach
    while (DIRECT_READ(rReg, rBit)) {	// while Output pin is HIGH
        Total++;				
    }
    StopTimer   	= micros();		// Stop Timer
        	
    // Calculate and return Timer Duration	
    return ((StopTimer - StartTimer) - RISEFALL_ADJUST);
}


// Charge-Discharge code for Capacitance
int Timer555Monostable::OneCycle_Capacitance(void) {
long Dur;    

    Dur			 = RunTimer();
    Period		+= Dur;
    Frequency   	+= 1 / (Dur/SECONDS_TO_MICROS);
    Capacitance 	+= UNITADJUST_CAP * Dur / (LOGNEPERIEN_3 * Resist_R1);
    
    //Return
    return 1;
}


// Charge-Discharge code for Resistance
int Timer555Monostable::OneCycle_Resistance(void) {
long Dur;    

    Dur			 = RunTimer();
    Period		+= Dur;
    Frequency   	+= 1 / (Dur/SECONDS_TO_MICROS);
    Resistance	 	+= UNITADJUST_RES * Dur / (LOGNEPERIEN_3 * Capacit_C1);
    
    //Return
    return 1;

}
// /////////////////////////////////////////////////////////////////////////////

