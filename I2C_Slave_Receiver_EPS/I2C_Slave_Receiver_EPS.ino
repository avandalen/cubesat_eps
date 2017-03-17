#include <Wire.h>

int16_t sizeOfDataArray;
float receivedVoltages [4] = {0};
float receivedCurrents [8] = {0};
float receivedTemps    [3] = {0};

void setup() {
  Wire.begin(8);                // join I2C bus with address #8
  Wire.setClock(400000);        // set I2C clock freq to 400kHz
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(100);
}

// executes whenever data is received from master
void receiveEvent(int numberOfBytesToReceive) {

  sizeOfDataArray = (Wire.available())/4;     // the amount of bytes in a transfer /4 to give the number of floats being transferred

  switch(sizeOfDataArray) {
    case 4: // array of voltages
      assembleFloatsfromBytes(receivedVoltages, sizeOfDataArray);

      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedVoltages[i]);         // print the array
      }
      Serial.println("v conversion complete");
      break;
    case 8: // array of currents
      assembleFloatsfromBytes(receivedCurrents, sizeOfDataArray);

      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedCurrents[i]);         // print the array
      }
      Serial.println("i conversion complete");
      break;
    case 3: // array of temperatures
      assembleFloatsfromBytes(receivedTemps, sizeOfDataArray);

      for (int16_t i = 0; i < sizeOfDataArray; ++i) {
        Serial.println(receivedTemps[i]);         // print the array
      }
      Serial.println("t conversion complete");
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
