/*
  Timer555Monostable- 
  Version: 0.0.1
  Date: 19-Dec-2018
  Author: Jerome Drouin
  https://github.com/JeromeDrouin/Timer555Monostable
  Capacitive Meter Library for 'duino / Wiring
  Capacitance is derived via 555 Timer IC set via a Multivibrator in Monostable mode, and one Resistor R1
  Capacitance by default is expressed in NanoFarads 
  
  C = a.b.T / (1,1 . R) ; with T in seconds and C in nF (Note: precise RC Constant is 1.0986 = ln(3))
  a = 1E9   : 1,000,000,000 nano Farads in 1 Farad (FARADS_TO_NANOFARADS)
  b = 1E-6  : 1,000,000 microseconds in one second (SECONDS_TO_MICROS)
  
  Copyright (c) 2018 Jerome Drouin  All rights reserved.  
*/
#include <Timer555Monostable.h>
 
#define FARADS_TO_NANOFARADS    1E9   // 1E9 = 1,000,000,000 (1Farad = 1bn Nano Farad)
#define SECONDS_TO_MICROS       1E6   // 1E6 = 1,000,000 (1Second = 1000millis = 1000000 Microseconds)
#define R1    1000000
float Biais_Correction = 0.97;

//Pins
const int8_t Output555 = 0;  // Attached to Interrupt 0
const int8_t Trigger555 = 2;

Timer555Monostable Timer555(Trigger555,Output555,R1,Biais_Correction);

int LoopCount=0;

// --- Setup -----------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(Output555, INPUT);              // PIN 3 = OUTPUT OF 555 CONNECTED HERE
  pinMode(Trigger555, OUTPUT);            // CONNECTED TO PIN 2 = TRIGGER OF 555
  digitalWrite(Trigger555, HIGH);         // High to Avoid triggering 555 accidentally 
}


// --- Loop -----------------------------------------------------------
void loop() {
  LoopCount++;

  float Cap1 = Timer555.GetCapacitanceValue(1);
  float Cap2 = Timer555.GetCapacitanceValue(10);
  float Cap3 = Timer555.GetCapacitanceValueRaw(1);
  
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
  Serial.print("  Cap1:");
  Serial.print(Cap1*display_factor,3);
  Serial.print(display_unit);
  Serial.print("  Cap2:");
  Serial.print(Cap2*display_factor,3);
  Serial.print(display_unit);
  Serial.print("  Cap3:");
  Serial.print(Cap3*display_factor,3);
  Serial.print(display_unit);
  
  Serial.print("  Freq:");
  Serial.print(Timer555.GetLastFrequency(),3);
  Serial.print("Hz");

  Serial.print("  Period:");
  Serial.print(Timer555.GetLastPeriod());
  Serial.print("us");

  Serial.print("\n");
  delay(500);
  
}
