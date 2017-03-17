#include <Wire.h>

// send:
//power consumption
//power availability in battery
//voltges
//currents
//temperatures

float voltage1 = 39.67;
float voltage2 = 39.68;
float voltage3 = 39.69;
float voltage4 = 39.70;

float current1 = 38.67;
float current2 = 38.68;
float current3 = 38.69;
float current4 = 38.70;
float current5 = 38.67;
float current6 = 38.68;
float current7 = 38.69;
float current8 = 38.70;

float temp1 = 37.67;
float temp2 = 37.68;
float temp3 = 37.69;

float voltageArray  [4] = {0};
float currentArray  [8] = {0};
float tempArray     [3] = {0};

void setup() {
  Wire.begin(8);            // join i2c bus (address optional for master)
  Wire.setClock(400000);    // set I2C clock freq to 400kHz
  Serial.begin(9600);
}
 

void loop() {
  voltageArray[0] = voltage1;
  voltageArray[1] = voltage2;
  voltageArray[2] = voltage3;
  voltageArray[3] = voltage4;

  currentArray[0] = current1;
  currentArray[1] = current2;
  currentArray[2] = current3;
  currentArray[3] = current4;
  currentArray[4] = current5;
  currentArray[5] = current6;
  currentArray[6] = current7;
  currentArray[7] = current8;

  tempArray[0] = temp1;
  tempArray[1] = temp2;
  tempArray[2] = temp3;

  Wire.beginTransmission(8);  // transmit to device #8

  // splitFloatsIntoBytesAndSend(voltageArray, 4);
  splitFloatsIntoBytesAndSend(currentArray, 8);
  // splitFloatsIntoBytesAndSend(tempArray, 3);

  Wire.endTransmission();    // stop transmitting

  delay(1000);
}

void splitFloatsIntoBytesAndSend(float * values, uint8_t sizeOfArray) {
  // create a union which will contain the float variable (f) in the same address space as four bytes (byte[4])
  // the float can be accessed as individual bytes which can be sent over I2C then reassembled on the other side 
  // using another union to reverse the process
  union {
    float f;
    uint8_t bytes[4];
  } Union;

  for (int16_t i = 0; i < sizeOfArray; i++) {
    Union.f = values[i];

    Wire.write(Union.bytes[3]); // MSB
    Wire.write(Union.bytes[2]);
    Wire.write(Union.bytes[1]);
    Wire.write(Union.bytes[0]); // LSB
  }
}



