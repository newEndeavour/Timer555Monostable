/*
  File:         Timer555Monostable.cpp
  Version:      0.1.4
  Date:         19-Dec-2018
  Revision:     16-Jan-2021
  Author:       Jerome Drouin (jerome.p.drouin@gmail.com)

  Editions:	Please go to Timer555Monostable.h for Important Edition Notes.

  https://github.com/newEndeavour/Timer555Monostable

  Capacitive &/or Resistance Meter Library for 'duino / Wiring
  Capacitance &/or Resistance is derived via 555 Timer IC in Monostable mode, Resistor R1 (Cap Meter mode) 
  or Capacitance C1 (Res Meter mode). 
  Capacitance by default is expressed in NanoFarads. 
  Resistance by default is expressed in Ohms. 
	
	C = (a / b x T) / (1.1 x R1) ; with T in seconds and C in pF
	C		: Capacitance in pF
	R1		: Resistance in Ohms
	a = 1E12	: 1,000,000,000,000 pico Farads in 1 Farad (FARADS_TO_PICOFARADS)
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
	DischargePin		= -1;
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
	
	Calibrate_SysTickParams();

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
	DischargePin		= -1;
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

	Calibrate_SysTickParams();

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

	Calibrate_SysTickParams();

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

	Calibrate_SysTickParams();

	//Check for errors
	ResetErrors();
}



// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

// Returns the Version
String Timer555Monostable::GetVersion(void)
{
	return VER_Timer555Monostable;
}

//Returns the Last Version Date
String Timer555Monostable::GetReleaseDate(void)
{
	return REL_Timer555Monostable;
}

// Returns the Board Type
String Timer555Monostable::GetBoardType(void)
{
	return TIMER555MONOSTABLE_BOARD_TYPE;
}


// Returns the Method for Timing T=RC
String Timer555Monostable::GetTimingMethod(void)
{
	return TIMER555MONOSTABLE_TIMING_METH;
}


void Timer555Monostable::DisplayObjectSetup(void)
{
	Serial.print("\n");
	Serial.print("\nTimer555Monostable:");
	Serial.print("\nVersion	   : ");Serial.print(GetVersion());
	Serial.print("\nRelease	   : ");Serial.print(GetReleaseDate());
	Serial.print("\nBoard  	   : ");Serial.print(GetBoardType());
	Serial.print("\n- Pins");
	Serial.print("\nTrigger	   : ");Serial.print(GetTriggerPin());
	Serial.print("\nOutPut 	   : ");Serial.print(GetOutputPin());
	Serial.print("\nDischrg    : ");
	if (ObjecthasDischargePin) Serial.print(GetDischargePin());else Serial.print("N/A");

	Serial.print("\n- Main Params");
	Serial.print("\nResist R1  : ");Serial.print(GetResist_R1());Serial.print(" Ohms");
	Serial.print("\nCapac  C1  : ");Serial.print(GetSysTickLOAD());Serial.print(" pF");
	Serial.print("\nTiming     : ");Serial.print(GetTimingMethod());

	Serial.print("\n- Scanning");
	Serial.print("\nDefault    : ");Serial.print(DEFAULT_SCANS_PER_CYCLE);Serial.print(" Scans/Cycle");
	#if defined(ENABLE_FREQ_JITTER)
	Serial.print("\nJitter     : ENABLED");
	Serial.print("\nDelay      : 0-");Serial.print(FREQ_JITTER_MICROS);Serial.print("uS (random)");
	#else
	Serial.print("\nJitter     : DISABLED");	
	#endif

	#if defined(TIMER_USE_SYSTICK)
	Serial.print("\n- Timer Method:");Serial.print(TIMER555MONOSTABLE_TIMING_METH);

	Serial.print("\n- SysTick Params");
	Serial.print("\nBase       : ");Serial.print(GetSysTickBase());
	Serial.print("\nLOAD       : ");Serial.print(GetSysTickLOAD());
	Serial.print("\nLOAD fac   : ");Serial.print(GetSysTickLOADFac(),6);
	#endif

	#if defined(TIMER_USE_MICROS)
	Serial.print("\n- Timer Method:");Serial.print(TIMER555MONOSTABLE_TIMING_METH);
	#endif

	Serial.print("\n- Error");
	Serial.print("\nFlag       : ");Serial.print(GetError());

	Serial.print("\n");
}


// Returns the TriggerPin Parameter
uint8_t	Timer555Monostable::GetTriggerPin(void)
{
	return TriggerPin;
}


// Returns the OutputPin Parameter
uint8_t	Timer555Monostable::GetOutputPin(void)
{
	return OutputPin;
}


// Returns the DischargePin Parameter
uint8_t	Timer555Monostable::GetDischargePin(void)
{
	return DischargePin;
}


// Returns the Resist_R1 Parameter
uint32_t Timer555Monostable::GetResist_R1(void)
{
	return Resist_R1;
}


// Returns the Capacit_C1 Parameter
uint32_t Timer555Monostable::GetCapacit_C1(void)
{
	return Capacit_C1;
}


// Returns the Baseline_Cap Parameter
float Timer555Monostable::GetBaseline_Cap(void)
{
	return Baseline_Cap;
}

// Returns the Baseline_Res Parameters
float Timer555Monostable::GetBaseline_Res(void)
{
	return Baseline_Res;
}


// Returns the last available calculated AvgFrequency
float Timer555Monostable::GetAvgFrequency(void)
{
	return AvgFrequency;
}

// Returns the last available calculated Average AvgPeriod in microseconds
float Timer555Monostable::GetAvgPeriod(void)
{
	return AvgPeriod;
}

// Returns the AvgPeriod Type 
int Timer555Monostable::GetAvgPeriodType(void)
{
	#if defined(AVGPERIOD_AS_FLOAT)
		return AVGPERIOD_AS_FLOAT;
	#endif
	#if defined(AVGPERIOD_AS_INT)
		return AVGPERIOD_AS_INT;
	#endif
}

// Returns the last available calculated Duration in microseconds
//uint32_t Timer555Monostable::GetDuration(void)
float Timer555Monostable::GetDuration(void)
{
	return Duration;
}


// Returns the last available calculated Total
uint32_t Timer555Monostable::GetTotal(void)
{
	return Total;
}


// Returns the last available SysTickBase
int Timer555Monostable::GetSysTickBase(void)
{
	return SysTickBase;
}


// Returns the last available SysTickLOAD
int Timer555Monostable::GetSysTickLOAD(void)
{
	return SysTickLOAD;
}


// Returns the last available SysTickLOADFac
float Timer555Monostable::GetSysTickLOADFac(void)
{
	return SysTickLOADFac;
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


int  Timer555Monostable::GetError(void)
{
	return error;
}



// Returns Calculated Capacitance using the default param
float Timer555Monostable::GetCapacitance(void)
{
	return GetCapacitance(DEFAULT_SCANS_PER_CYCLE);
}


// Returns Calculated Capacitance
float Timer555Monostable::GetCapacitance(uint8_t samples)
{

	if (samples <= 0) return ERROR_FLAG_INVALID_SAMPLE;
	if (error < 0) return error;            		// construction error flag

	// Set results to zero before start of read
	Duration	= 0;
	Total		= 0;

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    		// loop for samples parameter - simple lowpass filter
		#if defined(TIMER_USE_MICROS)
			#if defined(ENABLE_FREQ_JITTER)
				delayMicroseconds(random(FREQ_JITTER_MICROS));
			#endif
			Duration += RunTimer_Micros();		// (timeout not implemented yet)

		#elif defined(TIMER_USE_SYSTICK)
			#if defined(ENABLE_FREQ_JITTER)
				#if defined(TIMER_USE_SYSTICK)
					delayMicroseconds_SysTick(random(FREQ_JITTER_MICROS));
				#elif defined(TIMER_USE_MICROS)
					delayMicroseconds(random(FREQ_JITTER_MICROS));
				#endif
			#endif
			Duration += RunTimer_SysTick();		// (timeout not implemented yet)
		#endif

		//Serial.print("\n-->loop Duration :");	
    		//Serial.print(Duration,6);	

	}

	// Update variables
	#if defined(AVGPERIOD_AS_FLOAT)
		AvgPeriod	= (float)Duration / (float)samples;	// Average AvgPeriod - Returns a float
	#endif

	#if defined(AVGPERIOD_AS_INT)
		AvgPeriod	= Duration / samples;			// Average AvgPeriod - Returns an int
	#endif

	AvgFrequency   	= 1 / (AvgPeriod/SECONDS_TO_MICROS);	// AvgFrequency
	Capacitance 	= UnitLn3_R1 * AvgPeriod;  		// UNITADJUST_CAP * AvgPeriod / (LN_3 * Resist_R1);
	Capacitance	= Capacitance - Baseline_Cap; 		// Capacitance 

	/*
	Serial.print("\n---- GetCapacitance ----");	
	Serial.print("\n-->Duration :");	
    	Serial.print(Duration,6);	
	Serial.print("\n-->AvgPeriod :");	
    	Serial.print(AvgPeriod,6);	
	Serial.print("\n-->AvgFrequency :");	
    	Serial.print(AvgFrequency,6);	
	Serial.print("\n-->Capacitance (1E3) :");	
    	Serial.print(Capacitance *1000,6);	
	//*/

	// Return
	return Capacitance;

}


// Returns Calculated Resistance using the default param
float Timer555Monostable::GetResistance(void)
{
	return GetResistance(DEFAULT_SCANS_PER_CYCLE);
}


// Returns Calculated Resistance
float Timer555Monostable::GetResistance(uint8_t samples)
{

	if (samples <= 0) return ERROR_FLAG_INVALID_SAMPLE;
	if (error < 0) return error;            		// construction error flag

	// Set results to zero before start of read
	Duration	= 0;
	Total		= 0;

	// capacitance read: we read the capacitor 'sample' times
	for (uint8_t i = 0; i < samples; i++) {    		// loop for samples parameter - simple lowpass filter
		#if defined(TIMER_USE_MICROS)
			#if defined(ENABLE_FREQ_JITTER)
				delayMicroseconds(random(FREQ_JITTER_MICROS));
			#endif
			Duration += RunTimer_Micros();		// (timeout not implemented yet)

		#elif defined(TIMER_USE_SYSTICK)
			#if defined(ENABLE_FREQ_JITTER)				
				//delayMicroseconds_SysTick(random(FREQ_JITTER_MICROS));
			#endif
			Duration += RunTimer_SysTick();		// (timeout not implemented yet)
		#endif
	}

	// Update variables
	#if defined(AVGPERIOD_AS_FLOAT)
		AvgPeriod	= (float)Duration / (float)samples;	// Average AvgPeriod - Returns a float
	#endif

	#if defined(AVGPERIOD_AS_INT)
		AvgPeriod	= Duration / samples;			// Average AvgPeriod - Returns an int
	#endif

	AvgFrequency   	= 1 / (AvgPeriod/SECONDS_TO_MICROS);	// AvgFrequency
	Resistance	= UnitLn3_C1 * AvgPeriod; 			// UNITADJUST_RES * AvgPeriod / (LN_3 * Capacit_C1);
	Resistance	= Resistance - Baseline_Res;		// Resistance

	// Return
	return Resistance;

}



// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library
// Charge-Discharge code for Capacitance and for Resistance

float Timer555Monostable::RunTimer_Micros(void) {

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
     
    interrupts();			// Restore interrupts ---> Transfered below as a test

    //---- T=RC Read --------------------
    StartTimer 	= micros();	// Start Timer 
    while (DIRECT_READ(rReg, rBit)) {	// while Output pin is HIGH
        #if defined(ENABLE_TOTAL_CALC)
		Total++;			// Count loops -> Total variable	
	#endif
    }
    StopTimer   	= micros();	// Stop Timer

    /* 
    Serial.print("\nTimerStart:");	
    Serial.print(StartTimer);	
    Serial.print("\nTimerStop:");	
    Serial.print(StopTimer);	
    Serial.print("\nDuration :");	
    Serial.print(StopTimer - StartTimer);	
    //*/
        	
    // Calculate and return Timer Duration
    return ((StopTimer - StartTimer));
		
}


float Timer555Monostable::RunTimer_SysTick(void) {

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
     
    //---- T=RC Read --------------------
    #if defined(TIMER_USE_MICROS)
    	StartTimer 	= micros();	// Start Timer 
    	while (DIRECT_READ(rReg, rBit)) {	// while Output pin is HIGH
        		#if defined(ENABLE_TOTAL_CALC)
			Total++;			// Count loops -> Total variable	
		#endif
    	}
    	StopTimer   	= micros();	// Stop Timer

    #elif defined(TIMER_USE_SYSTICK)
    	StartTimer 	= SysTick->VAL;	// Start Timer 
    	while (DIRECT_READ(rReg, rBit)) {	// while Output pin is HIGH
        		#if defined(ENABLE_TOTAL_CALC)
			Total++;			// Count loops -> Total variable	
		#endif
    	}
    	StopTimer   	= SysTick->VAL;	// Stop Timer
    	if (StartTimer<StopTimer) StartTimer += SysTickLOAD;	

    #endif

    interrupts();			// Restore interrupts

    /* 
    Serial.print("\nTimerStart:");	
    Serial.print(StartTimer);	
    Serial.print("\nTimerStop:");	
    Serial.print(StopTimer);	
    Serial.print("\nSysTickBase:");	
    Serial.print(SysTickBase);	
    Serial.print("\nSysTickLOADFac:");	
    Serial.print(SysTickLOADFac,8);	

    Serial.print("\nTicks:");	
    Serial.print(StartTimer - StopTimer - SysTickBase);	
    Serial.print("\nDuration :");	
    Serial.print((StartTimer - StopTimer - SysTickBase) * SysTickLOADFac,6);	
    //*/

    // Calculate and return Timer Duration	
    return (StartTimer - StopTimer - SysTickBase) * SysTickLOADFac;	//WARNING!: Start - Stop  as SysTick counter is down
		
}


int Timer555Monostable::OneCycle_Capacitance(void) {

    Duration		+= RunTimer_Micros();
    
    //Return
    return 1;
}


int Timer555Monostable::OneCycle_Resistance(void) {

    Duration		+= RunTimer_Micros();
    
    //Return
    return 1;

}


// Error flag handling
void Timer555Monostable::ResetErrors(void)
{
	// initialize this instance's variables
	error = ERROR_FLAG_NO_ERROR;

	#ifdef NUM_DIGITAL_PINS
		if (TriggerPin >= NUM_DIGITAL_PINS) 	error = ERROR_FLAG_INVALID_PINSET;
		if (OutputPin >= NUM_DIGITAL_PINS) 	error = ERROR_FLAG_INVALID_PINSET;
		if (ObjecthasDischargePin)
		if (DischargePin >= NUM_DIGITAL_PINS) 	error = ERROR_FLAG_INVALID_PINSET;
	#endif

	if (Resist_R1<0) 	error = ERROR_FLAG_INVALID_RESIS;
	if (Capacit_C1<0) 	error = ERROR_FLAG_INVALID_CAPAC;

}


// Calibrates SysTickBase
void Timer555Monostable::Calibrate_SysTickParams(void)
{
    noInterrupts();			// Disable interrupts
    
    #if defined(TIMER_USE_MICROS)
	StartTimer 	= micros();	// Start Timer 
    	StopTimer   	= micros();	// Stop Timer
    	SysTickBase    	= StartTimer - StopTimer;

    	SysTickLOAD	= 0;
    	SysTickLOADFac	= 0;
    #elif defined(TIMER_USE_SYSTICK)
	StartTimer 	= SysTick->VAL;	// Start Timer 
    	StopTimer   	= SysTick->VAL;	// Stop Timer
    	SysTickBase    	= StartTimer - StopTimer;

    	SysTickLOAD	= SysTick->LOAD;
    	SysTickLOADFac	= (float)1000/((float)SysTickLOAD+1);
    #endif    

    interrupts();			// Restore interrupts
}


// /////////////////////////////////////////////////////////////////////////////

