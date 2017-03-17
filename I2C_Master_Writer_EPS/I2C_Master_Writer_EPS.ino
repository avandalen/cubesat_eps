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

float voltageArray[4] = {0};

void setup() {
  Wire.begin(8);    // join i2c bus (address optional for master)
  Serial.begin(9600);
}
 

void loop() {
  voltageArray[0] = voltage1;
  voltageArray[1] = voltage2;
  voltageArray[2] = voltage3;
  voltageArray[3] = voltage4;

  Wire.beginTransmission(8);  // transmit to device #8

  splitFloatsIntoBytesAndSend(voltageArray, 4);

  Wire.endTransmission();    // stop transmitting

  delay(1000);
}

void splitFloatsIntoBytesAndSend(float * values, uint8_t sizeOfArray) {
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



