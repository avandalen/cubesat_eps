#include <I2C.h>
#include "LTC2943.h"

uint8_t fullCharge[2] = {0x72, 0xB5};   // Qbat/Qlsb = 29365 Qlsb's in full charge. 29365 = 0x72B5

void setup() {
  
  // Start Serial, I2c and LCD
  Serial.begin(9600);
  I2c.begin();
  I2c.setSpeed(100000);
  
  // B[7:6] - 01  manual mode -> initiate conversion from microcontroller , 00 sleep mode -> use just for config in void setup
  // B[5:3] - 100 prescaler of 256, 011 prescaler of 64
  // B[2:1] - 10  ALCC' pin in alert mode
  // B[0]   -     1 to shut down analogue part of chip to program in charge in battery. 0 to switch it back on

  // (uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes)
  I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B00011001);
  I2c.write(LTC2943_ADDRESS, LTC_ACCU_CHARGE_MSB, fullCharge, 2);  // when starting up, program in full charge
  I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B00011000);
}

void loop() {

  // Initiate conversion by setting B[7:6] to 01 then wait for 50ms + before reading from registers
  I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B01100100);
  delay(50);
  
  // Read two bytes form LTC_ACCU_CHARGE_MSB and convert to Accumulated Charge
  int16_t LTC2943AccuChargeRaw = readReg16(LTC_ACCU_CHARGE_MSB);
  float LTC2943AccuCharge = 0.08854  * LTC2943AccuChargeRaw;
  
  // Read two bytes from LTC_VOLTAGE_MSB and convert to Voltage
  int16_t LTC2943VoltageRaw = readReg16(LTC_VOLTAGE_MSB);
  float LTC2943Voltage = 23.600 * LTC2943VoltageRaw / 65535.000;
  
  // // Read two bytes from LTC_TEMP_MSB and convert to Celsius
  // uint16_t LTC2943TempRaw = readReg16(LTC_TEMP_MSB);
  // float LTC2943Temp = (510.00 * LTC2943TempRaw / 65535.00) - 273.15;
  
  // Read two bytes from LTC_CURRENT_MSB and convert to current
  uint16_t LTC2943CurrentRaw = readReg16(LTC_CURRENT_MSB);
  float LTC2943Current = 0.060 / senseResistor * ( ( LTC2943CurrentRaw - 32767.000 ) / 32767.000 ) * 1000;
  if (LTC2943Voltage == 0) {
    LTC2943Current = 0;
  }
  

  Serial.print("Qbat: ");
  Serial.print(LTC2943AccuCharge);
  Serial.println("mAh  ");
  
  Serial.print("Vbat: ");
  Serial.print(LTC2943Voltage);
  Serial.println("V  ");

  
  Serial.print("Ibat: ");
  Serial.print(LTC2943Current);
  Serial.println("mA  ");  

  delay(750);
}

uint8_t readReg8(uint16_t regAddress) {
  uint8_t data = 0;

  // Point to status register regAddress and receive one byte of data
  I2c.read(LTC2943_ADDRESS,regAddress,1);
  data = I2c.receive();
  
  return(data);
}

uint16_t readReg16(uint16_t regAddress) {
  uint8_t dataH, dataL;
  uint16_t data = 0;

  // Point to status register regAddress and receive two bytes of data
  I2c.read(LTC2943_ADDRESS,regAddress,2);
  dataH = I2c.receive();
  dataL = I2c.receive();  
  // Combine MSB and LSB to word
  data = word(dataH, dataL);

  return(data);
}
