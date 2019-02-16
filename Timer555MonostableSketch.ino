/*
  File:         Timer555Monostable
  Version:      0.0.6
  Date:         19-Dec-2018
  Revision:     16-Feb-2019
  Author:       Jerome Drouin
  
  https://github.com/newEndeavour/Timer555Monostable
  Capacitive &/or Resistance Meter Library for 'duino / Wiring
  Capacitance &/or Resistance is derived via 555 Timer IC in Monostable mode, Resistor R1 (Cap Meter mode) 
  or Capacitance C1 (Res Meter mode). 
  Capacitance by default is expressed in NanoFarads. 
  Resistance by default is expressed in Ohms. 
  
  C = (a / b x T) / (1.1 x R1) ; with T in seconds and C in nF
  C   : Capacitance in nF
  R1    : Resistance in Ohms
  a = 1E9   : 1,000,000,000 nano Farads in 1 Farad (FARADS_TO_NANOFARADS)
  b = 1E6   : 1,000,000 microseconds in one second (SECONDS_TO_MICROS)
  
  R = (c / b x T) / (1.1 x C1) ; with T in seconds and C in pF
  R   : Resistance in Ohms
  C1    : Capacitance in pF
  c = 1E12  : 1,000,000,000,000 pico Farads in 1 Farad (FARADS_TO_PICOFARADS)

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
#include <Timer555Monostable.h>

#define TRIGGER_FREQ_mS        1000          
#define CAP_BASELINE            0.0   // Baseline for Empty Capacitance in nF/Enter 0.0 if not in use
#define RES_BASELINE            100   // Baseline for Empty Resistance in Ohms/Enter 0.0 if not in use
#define SAMPLESIZE_1       2          // 
#define SAMPLESIZE_2      10          //
#define R1    240000                  // in Ohms  : Reference Resistance in Capacitor Meter Mode
#define C1    80.0                    // in pF    : Reference Capacitor in Resistance Meter Mode

//Pins
const int8_t Output555    = 0;        // Connected to 555Timer IC Pin 3 (Monostable)
const int8_t Trigger555   = 1;        // Connected to 555Timer IC Pin 2 (Monostable)

Timer555Monostable Timer555(Trigger555,Output555,R1,C1,CAP_BASELINE,RES_BASELINE);

int LoopCount=0;

// --- Setup -----------------------------------------------------------
void setup() {
  Serial.begin(115200);

  while (!Serial);

  pinMode(Output555, INPUT);              // PIN 3 = OUTPUT OF 555 CONNECTED HERE
  pinMode(Trigger555, OUTPUT);            // CONNECTED TO PIN 2 = TRIGGER OF 555
  digitalWrite(Trigger555, HIGH);         // High to Avoid triggering 555 accidentally 
}


// --- Loop -----------------------------------------------------------
void loop() {
  LoopCount++;

  float Cap1 = Timer555.GetCapacitance(SAMPLESIZE_1);
  float Res1 = Timer555.GetResistance(SAMPLESIZE_1);

  //Default unit = nF
  int display_factor = 1;
  String display_unit_Cap = "nF";
  String display_unit_Res = "Ohms";
  
  if (Cap1<1) {
    display_factor = 1000;
    display_unit_Cap = "pF";
  }
  if (Cap1>1000) {
    display_factor = 1/1000;
    display_unit_Cap = "uF";
  }
  if (Cap1>1000000) {
    display_factor = 1/1000000;
    display_unit_Cap = "mF";
  }

  Serial.print(LoopCount);
  Serial.print(") ");
  
  //SAMPLE SIZE 1
  Serial.print("\t Cap[");
  Serial.print(SAMPLESIZE_1);
  Serial.print("]\t\t:");
  Serial.print(Cap1*display_factor,3);
  Serial.print(display_unit_Cap);
  
  Serial.print("\t Res\t:");
  Serial.print(Res1,1);
  Serial.print(display_unit_Res);

  Serial.print("\tLoop\t:");
  Serial.print(Timer555.GetLastTotal());
  Serial.print("");

  Serial.print("\tFreq\t:");
  Serial.print(Timer555.GetLastFrequency(),3);
  Serial.print("Hz");

  Serial.print("\tPeriod\t:");
  Serial.print(Timer555.GetLastPeriod());
  Serial.print("us");

  Serial.print("\tDuration\t:");
  Serial.print(Timer555.GetLastDuration());
  Serial.print("us");

  //SAMPLE SIZE 2
  float Cap2 = Timer555.GetCapacitance(SAMPLESIZE_2);
  float Res2 = Timer555.GetResistance(SAMPLESIZE_2);

  Serial.print("\n\t Cap[");
  Serial.print(SAMPLESIZE_2);
  Serial.print("]\t:");
  Serial.print(Cap2*display_factor,3);
  Serial.print(display_unit_Cap);
  
  Serial.print("\t Res2\t:");
  Serial.print(Res2,1);
  Serial.print(display_unit_Res);
  
  Serial.print("\tLoop\t:");
  Serial.print(Timer555.GetLastTotal());
  Serial.print("");

  Serial.print("\tFreq\t:");
  Serial.print(Timer555.GetLastFrequency(),3);
  Serial.print("Hz");

  Serial.print("\tPeriod\t:");
  Serial.print(Timer555.GetLastPeriod());
  Serial.print("us");

  Serial.print("\tDuration\t:");
  Serial.print(Timer555.GetLastDuration());
  Serial.print("us");

  Serial.print("\n");
  delay(TRIGGER_FREQ_mS);
  
}
