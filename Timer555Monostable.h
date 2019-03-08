/*
  File:         Timer555Monostable.h
  Version:      0.1.2
  Date:         19-Dec-2018
  Revision:     08-Mar-2019
  Author:       Jerome Drouin (jerome.p.drouin@gmail.com)

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


  Editions:
  - 0.0.1	: First version
  - 0.0.2	: Additional member access methods
  - 0.0.3	: Added DischargePin to allow for a full discharge of the Capacitor. 
		  In certain high frequency cases, the cap has no time to discharge fully
		  a new trigger occurs therefore shortening the time to 2/3.Vcc.
		  Modification of the OneCycle method. 
  - 0.0.4	: DischargePin is now optional (new constructor)
  - 0.0.5	: Added Resistance Meter Capabilities from a Reference Capacitor C1
		  Removed bias_correction factor		  
		  Constructor modified to handle new Resistance Meter Capabilities
  - 0.0.6	: Added a debug enable mode when debug must be done in specific areas of code only
		  Added Duration as output parameter
		  Modify way the Capacitance / Resistance is calculated from the Average Period for increased speed
		  and decommissioned OneCycle_Capacitance to shorten return time (I think you got the picture that speed is
		  important, so will not mention it again, but let me just say that speed is crucial in this application - There,
		  I said it again...).
		  Changed Period into AvgPeriod=float from uint_32 to avoid casting during calcs (costly).
  - 0.1.1	: Implementation of a sub-microseconds approach to RC timing: SysTick.
		  Added supporting variables (SysTickBase, SysTickLOAD, SysTickLOADFac) and associated methods.
		  Note that this approach is valid only for T=RC under 1ms.
		  Please observe the following references and Credits:
		  "https://stackoverflow.com/questions/27885330/systick-load-vs-systick-calib"
		  "http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Bhcjegci.html"
  - 0.1.2	: GetCapacitance returns result in pF to be in line with GetResistance.
		  modified Supporting defined parameters and retired nano_farads conversion factors
  
*/


// ensure this library description is only included once
#ifndef Timer555Monostable_h
#define Timer555Monostable_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

// Direct I/O through registers and bitmask (from OneWire library)

#if defined(__AVR__)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask), (*((base)+2)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))
#define TIMER555MONOSTABLE_BOARD_TYPE "__AVR__"

#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (1)
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (*((base)+512))
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+640) = 0)
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+640) = 1)
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+256) = 1)
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+128) = 1)

//Board Type
#if defined(__MK20DX128__) 
	#define TIMER555MONOSTABLE_BOARD_TYPE "__MK20DX128__"
#elif defined(__MK20DX256__) 
	#define TIMER555MONOSTABLE_BOARD_TYPE "__MK20DX256__"
#elif defined(__MK66FX1M0__) 
	#define TIMER555MONOSTABLE_BOARD_TYPE "__MK66FX1M0__"
#elif defined(__MK64FX512__)
	#define TIMER555MONOSTABLE_BOARD_TYPE "__MK64FX512__"
#endif

#elif defined(__MKL26Z64__)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         ((*((base)+16) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+20) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+20) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+8) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+4) = (mask))
#define TIMER555MONOSTABLE_BOARD_TYPE "__MKL26Z64__"

#elif defined(__SAM3X8E__)
#define PIN_TO_BASEREG(pin)             (&(digitalPinToPort(pin)->PIO_PER))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*((base)+15)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+5)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+4)) = (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+13)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+12)) = (mask))
#define TIMER555MONOSTABLE_BOARD_TYPE "__SAM3X8E__"

#elif defined(__PIC32MX__)
#define PIN_TO_BASEREG(pin)             (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+4)) & (mask)) ? 1 : 0)  //PORTX + 0x10
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+2)) = (mask))            //TRISXSET + 0x08
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) = (mask))            //TRISXCLR + 0x04
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+8+1)) = (mask))          //LATXCLR  + 0x24
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+8+2)) = (mask))          //LATXSET + 0x28
#define TIMER555MONOSTABLE_BOARD_TYPE "__PIC32MX__"

#elif defined(ARDUINO_ARCH_ESP8266)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+6)) & (mask)) ? 1 : 0)    //GPIO_IN_ADDRESS
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+5)) = (mask))              //GPIO_ENABLE_W1TC_ADDRESS
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+4)) = (mask))              //GPIO_ENABLE_W1TS_ADDRESS
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+2)) = (mask))              //GPIO_OUT_W1TC_ADDRESS
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+1)) = (mask))              //GPIO_OUT_W1TS_ADDRESS
#define TIMER555MONOSTABLE_BOARD_TYPE "ARDUINO_ARCH_ESP8266"

#elif defined(__SAMD21G18A__)
// runs extremely slow/unreliable on Arduino Zero - help wanted....
#define PIN_TO_BASEREG(pin)             portModeRegister(digitalPinToPort(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*((base)+8)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+2)) = (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+5)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+6)) = (mask))
#define TIMER555MONOSTABLE_BOARD_TYPE "__SAMD21G18A__"

#elif defined(RBL_NRF51822)
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             (pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, pin)          nrf_gpio_pin_read(pin)
#define DIRECT_WRITE_LOW(base, pin)     nrf_gpio_pin_clear(pin)
#define DIRECT_WRITE_HIGH(base, pin)    nrf_gpio_pin_set(pin)
#define DIRECT_MODE_INPUT(base, pin)    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL)
#define DIRECT_MODE_OUTPUT(base, pin)   nrf_gpio_cfg_output(pin)
#define TIMER555MONOSTABLE_BOARD_TYPE "RBL_NRF51822"

#elif defined(__arc__)

#include "scss_registers.h"
#include "portable.h"
#include "avr/pgmspace.h"

#define GPIO_ID(pin)			(g_APinDescription[pin].ulGPIOId)
#define GPIO_TYPE(pin)			(g_APinDescription[pin].ulGPIOType)
#define GPIO_BASE(pin)			(g_APinDescription[pin].ulGPIOBase)
#define DIR_OFFSET_SS			0x01
#define DIR_OFFSET_SOC			0x04
#define EXT_PORT_OFFSET_SS		0x0A
#define EXT_PORT_OFFSET_SOC		0x50

/* GPIO registers base address */
#define PIN_TO_BASEREG(pin)		((volatile uint32_t *)g_APinDescription[pin].ulGPIOBase)
#define PIN_TO_BITMASK(pin)		pin
#define IO_REG_TYPE				uint32_t
#define IO_REG_ASM

static inline __attribute__((always_inline))
IO_REG_TYPE directRead(volatile IO_REG_TYPE *base, IO_REG_TYPE pin)
{
    IO_REG_TYPE ret;
    if (SS_GPIO == GPIO_TYPE(pin)) {
        ret = READ_ARC_REG(((IO_REG_TYPE)base + EXT_PORT_OFFSET_SS));
    } else {
        ret = MMIO_REG_VAL_FROM_BASE((IO_REG_TYPE)base, EXT_PORT_OFFSET_SOC);
    }
    return ((ret >> GPIO_ID(pin)) & 0x01);
}

static inline __attribute__((always_inline))
void directModeInput(volatile IO_REG_TYPE *base, IO_REG_TYPE pin)
{
    if (SS_GPIO == GPIO_TYPE(pin)) {
        WRITE_ARC_REG(READ_ARC_REG((((IO_REG_TYPE)base) + DIR_OFFSET_SS)) & ~(0x01 << GPIO_ID(pin)),
			((IO_REG_TYPE)(base) + DIR_OFFSET_SS));
    } else {
        MMIO_REG_VAL_FROM_BASE((IO_REG_TYPE)base, DIR_OFFSET_SOC) &= ~(0x01 << GPIO_ID(pin));
    }
}

static inline __attribute__((always_inline))
void directModeOutput(volatile IO_REG_TYPE *base, IO_REG_TYPE pin)
{
    if (SS_GPIO == GPIO_TYPE(pin)) {
        WRITE_ARC_REG(READ_ARC_REG(((IO_REG_TYPE)(base) + DIR_OFFSET_SS)) | (0x01 << GPIO_ID(pin)),
			((IO_REG_TYPE)(base) + DIR_OFFSET_SS));
    } else {
        MMIO_REG_VAL_FROM_BASE((IO_REG_TYPE)base, DIR_OFFSET_SOC) |= (0x01 << GPIO_ID(pin));
    }
}

static inline __attribute__((always_inline))
void directWriteLow(volatile IO_REG_TYPE *base, IO_REG_TYPE pin)
{
    if (SS_GPIO == GPIO_TYPE(pin)) {
        WRITE_ARC_REG(READ_ARC_REG(base) & ~(0x01 << GPIO_ID(pin)), base);
    } else {
        MMIO_REG_VAL(base) &= ~(0x01 << GPIO_ID(pin));
    }
}

static inline __attribute__((always_inline))
void directWriteHigh(volatile IO_REG_TYPE *base, IO_REG_TYPE pin)
{
    if (SS_GPIO == GPIO_TYPE(pin)) {
        WRITE_ARC_REG(READ_ARC_REG(base) | (0x01 << GPIO_ID(pin)), base);
    } else {
        MMIO_REG_VAL(base) |= (0x01 << GPIO_ID(pin));
    }
}

#define DIRECT_READ(base, pin)		directRead(base, pin)
#define DIRECT_MODE_INPUT(base, pin)	directModeInput(base, pin)
#define DIRECT_MODE_OUTPUT(base, pin)	directModeOutput(base, pin)
#define DIRECT_WRITE_LOW(base, pin)	directWriteLow(base, pin)
#define DIRECT_WRITE_HIGH(base, pin)	directWriteHigh(base, pin)
#define TIMER555MONOSTABLE_BOARD_TYPE "__arc__"

#endif


// DEFINES /////////////////////////////////////////////////////////////
#define VER_Timer555Monostable	"0.1.2"		// 
#define REL_Timer555Monostable	"08Mar2019"	//

//#define AVGPERIOD_AS_INT	0		// AvgPeriod = Duration / Sample returns an int (without decimals)
#define AVGPERIOD_AS_FLOAT	1		// AvgPeriod = Duration / Sample returns a float (with decimals)

//
//
//	TIMER DEFINITION: WARNING !
//	ONLY CHANGE IF YOU KNOW WHAT YOU ARE DOING !!!!
//
//
//#define TIMER_USE_MICROS	  1		// Timer uses the Micros Approach for timing T=RC
#define TIMER_USE_SYSTICK	1		// Timer uses the SysTick Register Approach for timing T=RC
						// WARNING!!the SYSTICK Method valid only 
						// if Period < 1ms (1000uS). 
						// Nothing is done to manage Switch over in case Time is longer 
//
//
//
//
//



#if defined (TIMER_USE_MICROS)
#define TIMER555MONOSTABLE_TIMING_METH "MICROS"
#elif defined(TIMER_USE_SYSTICK)
#define TIMER555MONOSTABLE_TIMING_METH "SYSTICK"
#endif

//#define ENABLE_TOTAL_CALC	  1		// Object Enables the Calculation of Total = Number of Loops inside Timer	
						// Note: this is an expensive feature (time), so here for those who need it
						// uncomment this line to enable the function

//#define FARADS_TO_NANOFARADS 	1E9		// RETIRED
#define SECONDS_TO_MICROS 	1E6
#define FARADS_TO_PICOFARADS 	1E12

#define UNITADJUST_CAP	 	1E6 		// = FARADS_TO_PICOFARADS/SECONDS_TO_MICROS
#define UNITADJUST_RES	 	1E6		// = FARADS_TO_PICOFARADS/SECONDS_TO_MICROS
#define LN_3			1.098612289	// Neperien Logarithm of 3.0

#define UCAP_LN_3		910239.2264	// UNITADJUST_CAP / LN_3
#define URES_LN_3		910239.2264	// UNITADJUST_RES / LN_3

#define	RISEFALL_ADJUST		1		//Timer Adjustment - Rise&Fall in microsecond to adjust the Timer
						//in reality this is a variable depending on R1 and C1...



// library interface description ////////////////////////////////////////
class Timer555Monostable
{
  // user-accessible "public" interface
  public:
  // methods
	Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _C1);
	Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res);

	Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint8_t _DischargePin, uint32_t _R1, float _C1);
	Timer555Monostable(uint8_t _TriggerPin, uint8_t _OutputPin, uint8_t _DischargePin, uint32_t _R1, float _C1, float _Baseline_Cap, float _Baseline_Res);

	float 		GetCapacitance(uint8_t samples);
	float 		GetResistance(uint8_t samples);

	float 		GetCapacitance();
	float 		GetResistance();

	float 		GetBaseline_Cap();
	float 		GetBaseline_Res();

	float 		GetAvgFrequency(void);
	float 		GetAvgPeriod(void);
	uint32_t 	GetTotal(void);
	//uint32_t 	GetDuration(void);
	float 		GetDuration(void);

	/*
	uint32_t 	GetDuration1(void);
	uint32_t 	GetDuration2(void);
	uint32_t 	GetDuration3(void);
	uint32_t 	GetDuration4(void);
	uint32_t 	GetDuration5(void);
	uint32_t 	GetDuration6(void);
	uint32_t 	GetDuration7(void);
	uint32_t 	GetDuration8(void);
	*/

	void  		EnableDebug();
	void  		DisableDebug();
	int 		GetAvgPeriodType(void);

	int		GetSysTickBase();
	int		GetSysTickLOAD();
	float		GetSysTickLOADFac();

	String 		GetVersion();
	String 		GetReleaseDate();
	String 		GetBoardType();
	String 		GetTimingMethod();


  // library-accessible "private" interface
  private:
  // variables
	int error;
	int en_debug;

	uint8_t	TriggerPin;		//Trigger-Pulse Pin: connect to Pin2 of 555 Timer
	uint8_t	OutputPin;		//Output-Signal Pin: connect to Pin3 of 555 Timer
	uint8_t	DischargePin;		//

	int	SysTickBase;		//
	int	SysTickLOAD;		//
	float	SysTickLOADFac;		//


	/*
	uint32_t Duration1;		//in uS
	uint32_t Duration2;		//in uS
	uint32_t Duration3;		//in uS
	uint32_t Duration4;		//in uS
	uint32_t Duration5;		//in uS
	uint32_t Duration6;		//in uS
	uint32_t Duration7;		//in uS
	uint32_t Duration8;		//in uS
	*/

	unsigned long StartTimer;	//in uS	
	unsigned long StopTimer;	//in uS
	//unsigned long Duration;		//in uS
	float 	 Duration;		//in uS

	uint32_t Resist_R1;		//in Ohms
	uint32_t Capacit_C1;		//in pF
	uint32_t Total;			//in loop cycles

	float	 AvgPeriod;		//in uS
	float 	 AvgFrequency;		//in Hz
	float 	 Capacitance;		//in nF
	float 	 Resistance;		//in Ohms
	float 	 Baseline_Cap;		//in nF
	float 	 Baseline_Res;		//in Ohms

	float 	 UnitLn3_R1;		//pre-Calculated variable for Capacitance
	float 	 UnitLn3_C1;		//pre-Calculated variable for Resistance

	
	IO_REG_TYPE sBit;   	// Trigger pin's ports and bitmask
	volatile IO_REG_TYPE *sReg;

	IO_REG_TYPE rBit;    	// Output pin's ports and bitmask
	volatile IO_REG_TYPE *rReg;

	IO_REG_TYPE dBit;    	// Discharge pin's ports and bitmask
	volatile IO_REG_TYPE *dReg;
	int 	 ObjecthasDischargePin;

  // methods
	int 	OneCycle_Capacitance(void);
	int 	OneCycle_Resistance(void);
	//unsigned long RunTimer(void);
	float   RunTimer_Micros(void);
	float   RunTimer_SysTick(void);
	void 	ResetErrors(void);
	void 	Calibrate_SysTickParams();
};

#endif
