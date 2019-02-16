/*
  File:         Timer555Monostable.cpp
  Version:      0.0.6
  Date:         19-Dec-2018
  Revision:     16-Feb-2019
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
Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _C1)
{

	//Objects Parameters
	en_debug = 0;							//Debug disabled by default

	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= 0;					// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= 0;					// if not in use, input 0.0

	//Pre Calculations
	UnitLn3_R1		= UCAP_LN_3 / (float)Resist_R1;
	UnitLn3_C1		= URES_LN_3 / (float)Capacit_C1;
	
	//Digital Pins
	TriggerPin		= _TriggerPin;
	OutputPin		= _OutputPin;
	DischargePin		= 0;
	ObjecthasDischargePin 	= 0;					// Object Setup without discharge Pin

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

	//Check for errors
	ResetErrors();
}


Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res)
{

	//Objects Parameters
	en_debug = 0;							//Debug disabled by default

	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= _Baseline_Cap;			// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= _Baseline_Res;			// if not in use, input 0.0
		
	//Pre Calculations
	UnitLn3_R1		= UCAP_LN_3 / (float)Resist_R1;
	UnitLn3_C1		= URES_LN_3 / (float)Capacit_C1;

	//Digital Pins
	TriggerPin		= _TriggerPin;
	OutputPin		= _OutputPin;
	DischargePin		= 0;
	ObjecthasDischargePin 	= 0;					// Object Setup without discharge Pin

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

	//Check for errors
	ResetErrors();
}


Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint8_t _DischargePin, uint32_t _R1, float _C1)
{

	//Objects Parameters
	en_debug = 0;							//Debug disabled by default

	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= 0.0;					// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= 0.0;					// if not in use, input 0.0

	//Pre Calculations
	UnitLn3_R1		= UCAP_LN_3 / (float)Resist_R1;
	UnitLn3_C1		= URES_LN_3 / (float)Capacit_C1;
		
	//Digital Pins
	TriggerPin		= _TriggerPin;
	OutputPin		= _OutputPin;
	DischargePin		= _DischargePin;
	ObjecthasDischargePin 	= 1;					// Object Setup WITH discharge Pin

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

	// get pin mapping and port for DischargePin - from digital pin functions in Wiring.c
	dBit = PIN_TO_BITMASK(_DischargePin);				// get Discharge pin's ports and bitmask
	dReg = PIN_TO_BASEREG(_DischargePin);				// get pointer to output register
	DIRECT_MODE_INPUT(dReg, dBit); 					// DischargePin to INPUT -> Cap. charging possible

	//Check for errors
	ResetErrors();
}


Timer555Monostable::Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint8_t _DischargePin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res)
{

	//Objects Parameters
	en_debug = 0;							//Debug disabled by default

	Resist_R1 		= _R1;
	Capacitance     	= 0;
	Baseline_Cap		= _Baseline_Cap;			// if not in use, input 0.0

	Capacit_C1		= _C1;					// if not in use, input 0.0
	Resistance      	= 0;
	Baseline_Res		= _Baseline_Res;			// if not in use, input 0.0

	//Pre Calculations
	UnitLn3_R1		= UCAP_LN_3 / (float)Resist_R1;
	UnitLn3_C1		= URES_LN_3 / (float)Capacit_C1;
		
	//Digital Pins
	TriggerPin		= _TriggerPin;
	OutputPin		= _OutputPin;
	DischargePin		= _DischargePin;
	ObjecthasDischargePin 	= 1;					// Object Setup WITH discharge Pin

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

	// get pin mapping and port for DischargePin - from digital pin functions in Wiring.c
	dBit = PIN_TO_BITMASK(_DischargePin);				// get Discharge pin's ports and bitmask
	dReg = PIN_TO_BASEREG(_DischargePin);				// get pointer to output register
	DIRECT_MODE_INPUT(dReg, dBit); 					// DischargePin to INPUT -> Cap. charging possible

	//Check for errors
	ResetErrors();
}



// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries
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


float Timer555Monostable::GetLastPeriod(void)
{
	// Returns the last available calculated Period in microseconds
	return Period;
}

uint32_t Timer555Monostable::GetLastDuration(void)
{
	// Returns the last available calculated Duration in microseconds
	return Duration;
}


uint32_t Timer555Monostable::GetLastTotal(void)
{
	// Returns the last available calculated GetLastTotal
	return Total;
}

void  Timer555Monostable::EnableDebug(void)
{
	//Enable debug
	en_debug = 1;
}

void  Timer555Monostable::DisableDebug(void)
{
	//Disable debug
	en_debug = 0;
}



//------------------------------------------------------------------------
//
//
//
//
/*

uint32_t Timer555Monostable::GetLastDuration1(void)
{
	return Duration1;
}
uint32_t Timer555Monostable::GetLastDuration2(void)
{
	return Duration2;
}
uint32_t Timer555Monostable::GetLastDuration3(void)
{
	return Duration3;
}
uint32_t Timer555Monostable::GetLastDuration4(void)
{
	return Duration4;
}
uint32_t Timer555Monostable::GetLastDuration5(void)
{
	return Duration5;
}
uint32_t Timer555Monostable::GetLastDuration6(void)
{
	return Duration6;
}
uint32_t Timer555Monostable::GetLastDuration7(void)
{
	return Duration7;
}
uint32_t Timer555Monostable::GetLastDuration8(void)
{
	return Duration8;
}
*/
//
//
//
//
//------------------------------------------------------------------------


// Returns Calculated Capacitance
float Timer555Monostable::GetCapacitance(uint8_t samples)
{

	if (samples <= 0) return 0;
	if (error < 0) return -1;            			// construction error check

	// Set results to zero before start of read
	Duration	= 0;
	Total		= 0;

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    		// loop for samples parameter - simple lowpass filter
		// if (OneCycle_Capacitance() < 0)  return -2;  // variable over timeout
		Duration += RunTimer();				// no test -> faster for now (timeout not implemented yet)
	}

	// Update variables
	Period		= Duration / samples;			// Average Period
	Frequency   	= 1 / (Period/SECONDS_TO_MICROS);	// Frequency
	Capacitance 	= UnitLn3_R1 * Period;  		// UNITADJUST_CAP * Period / (LN_3 * Resist_R1);
	Capacitance	= Capacitance - Baseline_Cap; 		// Capacitance 

	// Return
	return Capacitance;

}

// Returns Calculated Resistance
float Timer555Monostable::GetResistance(uint8_t samples)
{

	if (samples <= 0) return 0;
	if (error < 0) return -1;            			// construction error check

	// Set results to zero before start of read
	Duration	= 0;
	Total		= 0;

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    		// loop for samples parameter - simple lowpass filter
		// if (OneCycle_Resistance() < 0)  return -2;   // variable over timeout
		Duration += RunTimer();				// no test -> faster for now (timeout not implemented yet)
	}

	// Update variables
	Period		= Duration / samples;			// Average Period
	Frequency   	= 1 / (Period/SECONDS_TO_MICROS);	// Frequency
	Resistance	= UnitLn3_C1 * Period; 			// UNITADJUST_RES * Period / (LN_3 * Capacit_C1);
	Resistance	= Resistance - Baseline_Res;		// Resistance

	// Return
	return Resistance;

}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library
// Charge-Discharge code for Capacitance and for Resistance
unsigned long Timer555Monostable::RunTimer(void) {

    noInterrupts();			// Disable interrupts
    
    //---- Discharging Capacitor --------
    if (ObjecthasDischargePin) {
	DIRECT_MODE_INPUT(dReg, dBit);	// DischargePin to INPUT (pullups are off)
	DIRECT_MODE_OUTPUT(dReg, dBit); // DischargePin to OUTPUT
	DIRECT_WRITE_LOW(dReg, dBit);	// pin is now LOW AND OUTPUT
	delayMicroseconds(2);		// usually, takes no more than a few nanoSeconds: 2us is plenty of time...
	DIRECT_MODE_INPUT(dReg, dBit);	// DischargePin to INPUT (pullups are off)
    }
	

    //---- Pulse ------------------------	
    DIRECT_WRITE_LOW(sReg, sBit);	// TriggerPin Register low -> 555 Trigger
    					// No add. delay required: DIRAC pulse duration approx 200ns
    DIRECT_WRITE_HIGH(sReg, sBit);	// TriggerPin Register high -> Stop Trigger pulse    
    
    interrupts();			// Restore interrupts
 
    //---- RC Read ----------------------	
    StartTimer 		= micros();	// Start Timer 
    while (DIRECT_READ(rReg, rBit)) {	// while Output pin is HIGH
        Total++;			// Count loops -> Total variable	
    }
    StopTimer   	= micros();	// Stop Timer
        	
    // Calculate and return Timer Duration	
    return ((StopTimer - StartTimer) - RISEFALL_ADJUST);
}


int Timer555Monostable::OneCycle_Capacitance(void) {

    Duration		+= RunTimer();
    
    //Return
    return 1;
}


int Timer555Monostable::OneCycle_Resistance(void) {

    Duration		+= RunTimer();
    
    //Return
    return 1;

}

// Error flag handling
void Timer555Monostable::ResetErrors(void)
{
	// initialize this instance's variables
	error = 1;

	#ifdef NUM_DIGITAL_PINS
	if (TriggerPin >= NUM_DIGITAL_PINS) error = -2;
	if (OutputPin >= NUM_DIGITAL_PINS) error = -2;
	if (ObjecthasDischargePin)
		if (DischargePin >= NUM_DIGITAL_PINS) error = -2;
	#endif

	if (Resist_R1<0) 	error =-1;
	if (Capacit_C1<0) 	error =-1;

}

// /////////////////////////////////////////////////////////////////////////////

