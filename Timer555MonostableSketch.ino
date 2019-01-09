/*
  File:         Timer555Monostable
  Version:      0.0.2
  Date:         19-Dec-2018
  Revision:     09-Jan-2019
  Author:       Jerome Drouin
  https://github.com/newEndeavour/Timer555Monostable
  Capacitive Meter Library for 'duino / Wiring
  Capacitance is derived via 555 Timer IC in Monostable mode, and one Resistor R1
  Capacitance by default is expressed in NanoFarads 
  
  C = a.b.T / (1,1 . R) ; with T in seconds and C in nF (Note: precise RC Constant is 1.0986 = ln(3))
  a = 1E9   : 1,000,000,000 nano Farads in 1 Farad (FARADS_TO_NANOFARADS)
  b = 1E-6  : 1,000,000 microseconds in one second (SECONDS_TO_MICROS)
  
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

#define CAP_BASELINE            0.370   // Baseline for Empty Capacitance in nF/Enter 0.0 is not in use

#define FARADS_TO_NANOFARADS    1E9   // 1E9 = 1,000,000,000 (1Farad = 1bn Nano Farad)
#define SECONDS_TO_MICROS       1E6   // 1E6 = 1,000,000 (1Second = 1000millis = 1000000 Microseconds)
#define SAMPLESIZE_1       1
#define SAMPLESIZE_2      33

#define R1    1000000
float Biais_Correction = 0.97;

//Pins
const int8_t Output555 = 0;  // Attached to Interrupt 0
const int8_t Trigger555 = 2;

Timer555Monostable Timer555(Trigger555,Output555,R1,Biais_Correction,CAP_BASELINE);

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
  float Cap1R = Timer555.GetLastCapacitanceRaw();
  float Cap2 = Timer555.GetCapacitance(SAMPLESIZE_2);
  float Cap2R = Timer555.GetLastCapacitanceRaw();
  
  //Default unit = nF
  int display_factor = 1;
  String display_unit = "nF";
  
  if (Cap1<1) {
    display_factor = 1000;
    display_unit = "pF";
  }
  if (Cap1>1000) {
    display_factor = 1/1000;
    display_unit = "uF";
  }
  if (Cap1>1000000) {
    display_factor = 1/1000000;
    display_unit = "mF";
  }

  Serial.print(LoopCount);
  Serial.print(") ");
  
  //SAMPLE SIZE 1
  Serial.print("\t Cap(");
  Serial.print(SAMPLESIZE_1);
  Serial.print("):");
  Serial.print(Cap1*display_factor,3);
  Serial.print(display_unit);
  
  Serial.print("\t(Raw:");
  Serial.print(Cap1R*display_factor,3);
  Serial.print(display_unit);
  Serial.print(")");

  Serial.print("\tLoop:");
  Serial.print(Timer555.GetLastTotal());
  Serial.print("");

  Serial.print("\tFreq:");
  Serial.print(Timer555.GetLastFrequency(),3);
  Serial.print("Hz");

  Serial.print("\tPeriod:");
  Serial.print(Timer555.GetLastPeriod());
  Serial.print("us");

  //SAMPLE SIZE 2
  Serial.print("\n\t Cap(");
  Serial.print(SAMPLESIZE_2);
  Serial.print("):");
  Serial.print(Cap2*display_factor,3);
  Serial.print(display_unit);
  
  Serial.print("\t(Raw:");
  Serial.print(Cap2R*display_factor,3);
  Serial.print(display_unit);
  Serial.print(")");
  
  Serial.print("\tLoop:");
  Serial.print(Timer555.GetLastTotal());
  Serial.print("");

  Serial.print("\tFreq:");
  Serial.print(Timer555.GetLastFrequency(),3);
  Serial.print("Hz");

  Serial.print("\tPeriod:");
  Serial.print(Timer555.GetLastPeriod());
  Serial.print("us");

  Serial.print("\n");
  delay(500);
  
}
