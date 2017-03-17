#include <Wire.h>

const int16_t sizeOfDataArray        = 4;
// const int16_t numberOfBytesToReceive = sizeOfDataArray*4;  // each float is split into 4 bytes to be sent over I2C

float receivedVoltage1 = 0;
float receivedVoltage2 = 0;
float receivedVoltage3 = 0;
float receivedVoltage4 = 0;

float receivedVoltageArray[sizeOfDataArray] = {0};

void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int numberOfBytesToReceive) {

  assembleFloatsfromBytes(receivedVoltageArray, sizeOfDataArray);

  for (int16_t i = 0; i < sizeOfDataArray; ++i) {
    Serial.println(receivedVoltageArray[i]);         // print the array
  }

  Serial.println("conversion complete");
}

void assembleFloatsfromBytes(float * values, int16_t sizeOfArray) {
  union {
    float f;
    int8_t bytes[4];
  } Union;

  for (int16_t i = 0; i < sizeOfArray; i++) {
    Union.bytes[3] = Wire.read(); //0x41; // MSB
    Union.bytes[2] = Wire.read(); //0x70;
    Union.bytes[1] = Wire.read(); //0x00;
    Union.bytes[0] = Wire.read(); //0x00;

    values[i] = Union.f;
  }
}
