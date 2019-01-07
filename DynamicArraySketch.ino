/*
 * DynamicArray
 *  - 
 * Author : Jerome Drouin
 * Date   : Dec 20, 2018
 * 
 */

#include <DynamicArray.h>

const int ArraySize = 10;               // Declare a Dynamic Array with Size 10 elements
DynamicArray dynamicArray(ArraySize);

float Observation;
int LoopCount = 0;

//---- setup ----------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
    
  for (int i=0;i<100;i++) {
    Observation        = (1.0) *random(0,100)/100;    
    dynamicArray.PopulateArray(Observation);
  }

  //Statistics  
  Serial.print("\n\nInitial Array:\n");

  for (int i=0;i<ArraySize;i++) {
    Serial.print("Arg[");
    Serial.print(i);
    Serial.print("]:");
    Serial.print(dynamicArray.PullArgument(i+1),4);
    Serial.print("\n");
  }
  
  //Replace second and third observation with arbitrary data
  dynamicArray.PushArgument(2,22);  //22 takes the 2nd position
  dynamicArray.PushArgument(3,33);  //33 takes the 3rd position
  
  //Push additional argument at position 20 == Outside range 
  //and so therefore place this element at the end of range switching 
  //all previous arguments one place.
  dynamicArray.PushArgument(20,111);
    
  //Statistics  
  Serial.print("\n\nAfter Push Procedures:\n");
  for (int i=0;i<ArraySize;i++) {
    Serial.print("Arg[");
    Serial.print(i);
    Serial.print("]:");
    Serial.print(dynamicArray.PullArgument(i+1),4);
    Serial.print("\n");
  }

  Serial.print("\n\nStatistics:");
  Serial.print("\nAverage:");
  Serial.print(dynamicArray.GetAverage(),5);

  Serial.print("\nVariance:");
  Serial.print(dynamicArray.GetVariance(),5);

  Serial.print("\nStdDeviation:");
  Serial.print(dynamicArray.GetStdDeviation(),5);

  Serial.print("\nMin:");
  Serial.print(dynamicArray.GetMin(),5);

  Serial.print("\nMax:");
  Serial.print(dynamicArray.GetMax(),5);

  //
  dynamicArray.PushArgument(1,22);  //22 takes the 2nd position
  dynamicArray.PushArgument(ArraySize,111);  //22 takes the 2nd position
  Serial.print("\n\nLast Consistency Check:");
  Serial.print("\nAverage:");
  Serial.print(dynamicArray.GetLastAverage(),5);
  Serial.print("\nCalcAverage:");
  Serial.print(dynamicArray.GetAverage(),5);

  //Remove this instruction if you require Loop example
  while(1);  
}


//---- loop ----------------------------------------------------
void loop() {

  LoopCount++;
  
  Observation        = random(0,100);    

  //Running Average
  dynamicArray.PopulateArray(Observation);

  //Statistics  
  Serial.print("\n\n");
  Serial.print(LoopCount);
  Serial.print(")");

  Serial.print("\tAvg:");
  Serial.print(dynamicArray.GetLastAverage(),5);

  Serial.print("\tMin:");
  Serial.print(dynamicArray.GetLastMin(),5);

  Serial.print("\tMax:");
  Serial.print(dynamicArray.GetLastMax(),5);

  delay(500);
  
}
