/*
  DynamicArray.h v.01 - Library for 'duino
  https://github.com/JeromeDrouin/DynamicArray
  http://playground.arduino.cc/Main/DynamicArray

  Copyright (c) 2018 Jerome Drouin  All rights reserved.

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


// ensure this library description is only included once
#ifndef DynamicArray_h
#define DynamicArray_h

#include "Arduino.h"

// defines
#define MAX_ARRAY_SIZE 		10000
#define MAX_FLOAT_VALUE 	0xFFFFFFFF
#define MIN_FLOAT_VALUE 	-3.4028235E+38


// library interface description
class DynamicArray
{
  // user-accessible "public" interface
  public:
  // methods
	DynamicArray(void);
	explicit DynamicArray(uint8_t _arraySize);
	~DynamicArray();

	void  clearArray();
	void  PopulateArray(float lastObservation);
	int   PushArgument(uint8_t pos, float Observation);
	float PullArgument(uint8_t pos);

	float GetAverage(void);
	float GetLastAverage(void);
	float GetVariance(void);
	float GetStdDeviation(void);
	float GetMin(void);
	float GetMax(void);
	float GetLastMin(void);
	float GetLastMax(void);

  // library-accessible "private" interface
  //private:

  protected:
  // variables
	int 	error;
	uint8_t size;		// Size of Array
	uint8_t count;		// current Size of Array
	float   average;	// current average. Calculated automatically after each Populate
	float   cmin;		// current minimum. Calculated automatically after each Populate
	float   cmax;		// current maximum. Calculated automatically after each Populate
	float *	arg;		// Array values
	
  // methods

};

#endif
// END OF FILE