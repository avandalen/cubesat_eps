#include <Wire.h>
#include <I2C.h>
#include "LTC2943.h"

int16_t sizeOfDataArray;
float receivedVoltages [6] = {0};
float receivedCurrents [5] = {0};
float receivedCharge   [1] = {0};
float receivedTemps    [3] = {0};

void setup() {
  Wire.begin(8);                // join I2C bus with address #8
  Wire.setClock(400000);        // set I2C clock freq to 400kHz
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  // delay(100);
}

// executes whenever data is received from master
void receiveEvent(int numberOfBytesToReceive) {

  sizeOfDataArray = (Wire.available())/4;     // the amount of bytes in a transfer /4 to give the number of floats being transferred

  switch(sizeOfDataArray) {
    case 6: // array of voltages
      assembleFloatsfromBytes(receivedVoltages, sizeOfDataArray);

      /*
      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedVoltages[i]);         // print the array
      }
      */
      Serial.print("v5V_1: ");         
      Serial.println(receivedVoltages[0]);         // print the array
      Serial.print("v5V_2: ");
      Serial.println(receivedVoltages[1]);         // print the array
      Serial.print("v3.3V_1: ");   
      Serial.println(receivedVoltages[2]);         // print the array
      Serial.print("v3.3V_2: ");   
      Serial.println(receivedVoltages[3]);         // print the array
      Serial.print("vBatt: ");   
      Serial.println(receivedVoltages[4]);         // print the array
      Serial.print("vCell: ");   
      Serial.println(receivedVoltages[5]);         // print the array
      Serial.println(" ");
      break;
    case 5: // array of currents
      assembleFloatsfromBytes(receivedCurrents, sizeOfDataArray);
      /*
      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedCurrents[i]);         // print the array
      }
      */
      Serial.print("iBatt: ");         
      Serial.println(receivedCurrents[0], 4);         // print the array
      Serial.print("i5V_1: ");
      Serial.println(receivedCurrents[1], 4);         // print the array
      Serial.print("i5V_2: ");   
      Serial.println(receivedCurrents[2], 4);         // print the array
      Serial.print("i3.3V_1: ");   
      Serial.println(receivedCurrents[3], 4);         // print the array
      Serial.print("i3.3V_2: ");   
      Serial.println(receivedCurrents[4], 4);         // print the array
      Serial.println(" ");
      break;
    case 3: // array of temperatures
      assembleFloatsfromBytes(receivedTemps, sizeOfDataArray);
      /*
      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedTemps[i]);         // print the array
      }
      */
      Serial.print("tInternal: ");         
      Serial.println(receivedTemps[0], 4);         // print the array
      Serial.print("tBatt: ");
      Serial.println(receivedTemps[1], 4);         // print the array
      Serial.print("tExternal2: ");   
      Serial.println(receivedTemps[2], 4);         // print the array
      Serial.println(" ");
      break;

    case 1: // battery charge
    assembleFloatsfromBytes(receivedCharge, sizeOfDataArray);
      Serial.print("battCharge: ");         
      Serial.println(receivedCharge[0], 4);         // print the array
      Serial.println(" ");
      break;
  }
}

void assembleFloatsfromBytes(float * values, int16_t sizeOfArray) {
  // create a union which will contain the float variable (f) in the same address space as four bytes (byte[4])
  // the bytes are thus the four bytes of the float and can be received over I2C
  // they are then accessed as a float
  union {                    
    float f;
    int8_t bytes[4];
  } Union;

  for (int16_t i = 0; i < sizeOfArray; i++) {
    Union.bytes[3] = Wire.read(); // MSB
    Union.bytes[2] = Wire.read();
    Union.bytes[1] = Wire.read();
    Union.bytes[0] = Wire.read(); // LSB

    values[i] = Union.f;
  }
}
