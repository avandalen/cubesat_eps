#include <MAX11300.h>
#include <MAX11300registers.h>
#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Wire.h>
#include "avr/pgmspace.h"
#include <I2C.h>
#include "LTC2943.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  //clear
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   //set

uint8_t convertPin = 9;
uint8_t selectPin  = 6;
uint16_t MAX_ID    = 0;

// create array for all non-temperature ADC results to reside in
const uint8_t numberOfSingEndSensors = 11;
// const uint8_t numberOfDiffSensors    = 8; // 1/2 the number of channels that act as differential

//voltages
float voltageArray[6] = {0};

// currents
float currentArray[5] = {0};

// temperatures
double tExternal2     = 0;
double tBatt          = 0;
double tInternal      = 0;
float tempArray[3]    = {0};

// battery charge
float battCharge[1]   = {0};

uint8_t fullCharge[2] = {0x72, 0xB5};   // Qbat/Qlsb = 29365 Qlsb's in full charge. 29365 = 0x72B5

MAX11300 MAX11300(&SPI, convertPin, selectPin);

// Define Tasks
void TaskIVSamples      (void *pvParameters);
void TaskTempSamples    (void *pvParameters);
void TaskHeaterControl  (void *pvParameters);
void TaskPowerControl   (void *pvParameters);

void setup() {

  // PDM MOSFET gate pins
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);

  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  // Third argument is the number of words (not bytes!) to allocate for use as the task's stack.
  // ORDER THE TASKS BASED ON RUNTIME
  xTaskCreate(TaskIVSamples,      (const portCHAR *) "IVSamples",     512, NULL, 0, NULL);
  xTaskCreate(TaskTempSamples,    (const portCHAR *) "TempSamples",   128, NULL, 1, NULL);
  xTaskCreate(TaskHeaterControl,  (const portCHAR *) "HeaterControl", 128, NULL, 2, NULL);
  xTaskCreate(TaskPowerControl,   (const portCHAR *) "PowerControl",  128, NULL, 3, NULL);

  Wire.begin();             // join I2C bus (address optional for master)
  Wire.setClock(100000);    // set I2C clock freq to 400kHz
  I2c.begin();
  I2c.setSpeed(100000);

  MAX11300.begin();
  
  Serial.begin(9600);
  
  sei();                    // global enable interrupts
  setupTimerInterrupts();
  setupExternalInterrupts();

  // setup MCU as I2C slave so that the onboard computer can request power over I2C // slave sender, master reader, Sends data as an I2C/TWI slave device
  // also have the ground Arduino request data as a master

  // master writer, slave receiver -> master sends, slave prints out received message       <- for the EPS to 'ground' Arduino
  // master reader, slave sender   -> master requests, slave sends what has been requested  <- for the EPS to onboard computer
  
  // Identify shield by ID register  
  MAX_ID = MAX11300.checkID();     
  if (MAX_ID == 0x0424 ) {
  Serial.print("Found MAX module ID: 0x");
  Serial.println(MAX_ID, HEX);
  }
  
  // set ADC mode
  // Continuous sweep through the specified pins
  MAX11300.setADCmode(ContinuousSweep);
  if (MAX11300.getADCmode() == ContinuousSweep){Serial.println("ADC mode set to ContinuousSweep");}
  
  // set conversion rate
  // decrease this and increase the averaging amount to get
  // 400ksps -> 200ksps
  MAX11300.setConversionRate(rate200ksps);
  if (MAX11300.getConversionRate() == rate200ksps){Serial.println("ADC set to rate200ksps");}
  
  // set pin mode and voltage per pin

  // Positive analog input to single-ended ADC -> for PDM voltages: v5V_1, v5V_2, v3.3V_1, v3.3V_2
  MAX11300.setPinModeMAX(0, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(0) == analogIn){Serial.println("pin 0 set to ADC");} if(MAX11300.getPinADCRange(0) == ADCZeroTo10){Serial.println("pin0 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(1, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(1) == analogIn){Serial.println("pin 1 set to ADC");} if(MAX11300.getPinADCRange(1) == ADCZeroTo10){Serial.println("pin1 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(2, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(2) == analogIn){Serial.println("pin 2 set to ADC");} if(MAX11300.getPinADCRange(2) == ADCZeroTo10){Serial.println("pin2 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(3, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(3) == analogIn){Serial.println("pin 3 set to ADC");} if(MAX11300.getPinADCRange(3) == ADCZeroTo10){Serial.println("pin3 set ADCZeroTo10");}
  
  // Positive analog input to single-ended ADC -> for system currents: iBatt, i5V_1, i5V_2, i3.3V_i, i3.3V_2
  MAX11300.setPinModeMAX(4, MAX_FUNCID_ADC, ADCZeroTo2_5);
  if (MAX11300.getPinModeMAX(4) == analogIn){Serial.println("pin 4 set to ADC");} if(MAX11300.getPinADCRange(4) == ADCZeroTo2_5){Serial.println("pin4 set ADCZeroTo2_5");}
  MAX11300.setPinModeMAX(5, MAX_FUNCID_ADC, ADCZeroTo2_5);
  if (MAX11300.getPinModeMAX(5) == analogIn){Serial.println("pin 5 set to ADC");} if(MAX11300.getPinADCRange(5) == ADCZeroTo2_5){Serial.println("pin5 set ADCZeroTo2_5");}
  MAX11300.setPinModeMAX(6, MAX_FUNCID_ADC, ADCZeroTo2_5);
  if (MAX11300.getPinModeMAX(6) == analogIn){Serial.println("pin 6 set to ADC");} if(MAX11300.getPinADCRange(6) == ADCZeroTo2_5){Serial.println("pin6 set ADCZeroTo2_5");}
  MAX11300.setPinModeMAX(7, MAX_FUNCID_ADC, ADCZeroTo2_5);
  if (MAX11300.getPinModeMAX(7) == analogIn){Serial.println("pin 7 set to ADC");} if(MAX11300.getPinADCRange(7) == ADCZeroTo2_5){Serial.println("pin7 set ADCZeroTo2_5");}
  MAX11300.setPinModeMAX(8, MAX_FUNCID_ADC, ADCZeroTo2_5);
  if (MAX11300.getPinModeMAX(8) == analogIn){Serial.println("pin 8 set to ADC");} if(MAX11300.getPinADCRange(8) == ADCZeroTo2_5){Serial.println("pin8 set ADCZeroTo2_5");}
  
  // Positive analog input to single-ended ADC -> for battery and solar cell voltages: vBatt, vCell
  MAX11300.setPinModeMAX(9, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(9) == analogIn){Serial.println("pin 9 set to ADC");} if(MAX11300.getPinADCRange(9) == ADCZeroTo10){Serial.println("pin9 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(10, MAX_FUNCID_ADC, ADCZeroTo10);
  if (MAX11300.getPinModeMAX(10) == analogIn){Serial.println("pin 10 set to ADC");} if(MAX11300.getPinADCRange(10) == ADCZeroTo10){Serial.println("pin10 set ADCZeroTo10");}
  
  // MAX11300.setPinModeMAX(11, MAX_FUNCID_ADC, ADCZeroTo10);
  // if (MAX11300.getPinModeMAX(11) == analogIn){Serial.println("pin 11 set to ADC");} if(MAX11300.getPinADCRange(11) == ADCZeroTo10){Serial.println("pin11 set ADCZeroTo10");}
  
  // Bidirection Level Translator SDA 5V <-> SDA 3.3V
  MAX11300.setPinModeMAX(12, MAX_FUNCID_LVL_TRANS_BI);
  if (MAX11300.getPinModeMAX(12) == biderectionalTrans){Serial.println("pin 12 set to biderectionalTrans");}
  if (MAX11300.getPinModeMAX(13) == highImpedance){Serial.println("pin 13 set to highImpedance");}

    // Bidirection Level Translator SCL 5V <-> SCL 3.3V
  MAX11300.setPinModeMAX(14, MAX_FUNCID_LVL_TRANS_BI);
  if (MAX11300.getPinModeMAX(14) == biderectionalTrans){Serial.println("pin 14 set to biderectionalTrans");}
  if (MAX11300.getPinModeMAX(15) == highImpedance){Serial.println("pin 15 set to highImpedance");}
  
  /*
  MAX11300.setPinModeMAX(13, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(13) == analogIn){Serial.println("pin 13 set to ADC");} if(MAX11300.getPinADCRange(13) == ADCZeroTo10){Serial.println("pin13 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(14, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(14) == analogIn){Serial.println("pin 14 set to ADC");} if(MAX11300.getPinADCRange(14) == ADCZeroTo10){Serial.println("pin14 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(15, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(15) == analogIn){Serial.println("pin 15 set to ADC");} if(MAX11300.getPinADCRange(15) == ADCZeroTo10){Serial.println("pin15 set ADCZeroTo10");} 
  MAX11300.setPinModeMAX(16, MAX_FUNCID_ADC, ADCZeroTo10);
  // if (MAX11300.getPinModeMAX(16) == analogIn){Serial.println("pin 16 set to ADC");} if(MAX11300.getPinADCRange(16) == ADCZeroTo2_5){Serial.println("pin16 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(17, MAX_FUNCID_ADC, ADCZeroTo10);
  // if (MAX11300.getPinModeMAX(17) == analogIn){Serial.println("pin 17 set to ADC");} if(MAX11300.getPinADCRange(17) == ADCZeroTo2_5){Serial.println("pin17 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(18, MAX_FUNCID_ADC, ADCZeroTo10);
  // if (MAX11300.getPinModeMAX(18) == analogIn){Serial.println("pin 18 set to ADC");} if(MAX11300.getPinADCRange(18) == ADCZeroTo2_5){Serial.println("pin18 set ADCZeroTo10");}
  MAX11300.setPinModeMAX(19, MAX_FUNCID_ADC, ADCZeroTo10);
  // if (MAX11300.getPinModeMAX(19) == analogIn){Serial.println("pin 19 set to ADC");} if(MAX11300.getPinADCRange(19) == ADCZeroTo2_5){Serial.println("pin19 set ADCZeroTo10");}
  */

  // Positive analog input to differential ADC 
  // setPinModeMAX(uint8_t pin, pinMode_t mode, ADCRange_t ADCrange, uint8_t differentialPin);

  // MAX11300.setPinModeMAX(7, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 8);
  /*
  if (MAX11300.getPinModeMAX(7) == analogIn){Serial.println("pin 7 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(7)); Serial.println(" is pin 7's differential partner");  
  if(MAX11300.getPinADCRange(7) == ADCZeroTo10){Serial.println("pin 7 set zero to 10");}
  if(MAX11300.getPinADCRange(8) == ADCZeroTo10){Serial.println("pin 8 set zero to 10");}
  */
  
  // MAX11300.setPinModeMAX(9, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 10);
  /*
  if (MAX11300.getPinModeMAX(9) == analogIn){Serial.println("pin 9 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(9)); Serial.println(" is pin 9's differential partner");  
  if(MAX11300.getPinADCRange(9) == ADCZeroTo10){Serial.println("pin 9 set zero to 10");}
  if(MAX11300.getPinADCRange(10) == ADCZeroTo10){Serial.println("pin 10 set zero to 10");}
  */

  // MAX11300.setPinModeMAX(11, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 12);
  /*
  if (MAX11300.getPinModeMAX(11) == analogIn){Serial.println("pin 11 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(11)); Serial.println(" is pin 11's differential partner");  
  if(MAX11300.getPinADCRange(11) == ADCZeroTo10){Serial.println("pin 11 set zero to 10");}
  if(MAX11300.getPinADCRange(12) == ADCZeroTo10){Serial.println("pin 12 set zero to 10");}
  */

  // MAX11300.setPinModeMAX(13, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 14);
  /*
  if (MAX11300.getPinModeMAX(13) == analogIn){Serial.println("pin 13 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(13)); Serial.println(" is pin 13's differential partner");  
  if(MAX11300.getPinADCRange(13) == ADCZeroTo10){Serial.println("pin 13 set zero to 10");}
  if(MAX11300.getPinADCRange(14) == ADCZeroTo10){Serial.println("pin 14 set zero to 10");}
  */

  // MAX11300.setPinModeMAX(15, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 16);
  /*
  if (MAX11300.getPinModeMAX(15) == analogIn){Serial.println("pin 15 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(15)); Serial.println(" is pin 15's differential partner");  
  if(MAX11300.getPinADCRange(15) == ADCZeroTo10){Serial.println("pin 15 set zero to 10");}
  if(MAX11300.getPinADCRange(16) == ADCZeroTo10){Serial.println("pin 16 set zero to 10");}
  */

  // set voltage reference per pin
  // internal
  for (uint8_t i = 0; i < 20; i++){
    MAX11300.setPinADCref(i, ADCInternal);
    if (MAX11300.getPinADCref(i) == ADCInternal){
        Serial.print(i);
        Serial.println(" pin ref ADCInternal");
      }
  }

  // set averaging per pin
  // set to average 128 samples before loading into ADC reg for pin
  // NB/ Since one conversion per ADC-configured port is performed per sweep, 
  // many sweeps may be required before refreshing the data register of a given ADC-configured port that utilizes the averaging function.
  for (uint8_t i = 0; i < 20; i++){
    MAX11300.setPinAveraging (i, 128);
    Serial.println(MAX11300.getPinAveraging(i));
  }
  
  // enable required temperature sensors
  // 3 sensors can be enabled
  // ext2 ext1 internal 
  // count up in 3 bits
  // ie// mode 5 -> 101 -> ext2 enabled and internal enables
  MAX11300.setTempSensorEnableMode(mode7);
  if (MAX11300.getTempSensorEnableMode() == mode7){Serial.println("temp mode7");}
  
  // set each temperature sensor averaging
  // set to 32 samples for a more represenstative reading when sampling as register read rate is very slow
  MAX11300.setTempAveraging(internal, 32);
  // Serial.print("internal temp monitor averaging mode: ");
  // Serial.println(MAX11300.getTempAveraging(internal));
  MAX11300.setTempAveraging(external1, 32);
  // Serial.print("external1 temp monitor averaging mode: ");
  // Serial.println(MAX11300.getTempAveraging(external1));
  MAX11300.setTempAveraging(external2, 32);
  // Serial.print("external2 temp monitor averaging mode: ");
  // Serial.println(MAX11300.getTempAveraging(external2));

}
void loop(){
  // Empty. Things are done in Tasks.
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskIVSamples(void *pvParameters) {
  (void) pvParameters;
  // Perform an action every 30 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  // NB/ ADC data format is straight binary in single-ended mode, and two’s complement in differential and pseudo- differential modes.
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  // initialise arrays to contain ADC data
  uint16_t rawResultsSing[numberOfSingEndSensors]    = {0};
  // uint16_t  rawResultsDiff[16]                        = {0};
  float    scaledResultsSing[numberOfSingEndSensors] = {0};
  // float    scaledResultsDiff[8]                       = {0};

  // B[7:6] - 01  manual mode -> initiate conversion from microcontroller , 00 sleep mode -> use just for config in void setup
  // B[5:3] - 100 prescaler of 256, 011 prescaler of 64
  // B[2:1] - 10  ALCC' pin in alert mode
  // B[0]   -     1 to shut down analogue part of chip to program in charge in battery. 0 to switch it back on

  // (uint8_t address, uint8_t registerAddress, uint8_t *data, uint8_t numberBytes)
  I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B00011001);
  I2c.write(LTC2943_ADDRESS, LTC_ACCU_CHARGE_MSB, fullCharge, 2);  // when starting up, program in full charge
  I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B00011000);

  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    
    taskENTER_CRITICAL();

    MAX11300.burstAnalogRead(0, rawResultsSing, numberOfSingEndSensors);

    // Initiate conversion by setting B[7:6] to 01 then wait for 33ms + before reading from registers
    I2c.write(LTC2943_ADDRESS, LTC_CONTROL, B01100100);
    vTaskDelay(2);

    // Read two bytes form LTC_ACCU_CHARGE_MSB and convert to Accumulated Charge
    int16_t LTC2943AccuChargeRaw = readReg16(LTC_ACCU_CHARGE_MSB);
    float LTC2943AccuCharge = 0.08854  * LTC2943AccuChargeRaw;

    // Read two bytes from LTC_VOLTAGE_MSB and convert to Voltage
    int16_t LTC2943VoltageRaw = readReg16(LTC_VOLTAGE_MSB);
    float LTC2943Voltage = 23.600 * LTC2943VoltageRaw / 65535.000;

    for (uint8_t i = 0; i < numberOfSingEndSensors; i++) {
      scaledResultsSing[i] = ((float)(rawResultsSing[i]))/4096.0000; 
    }

    // fill voltage array
    // ADCZeroTo10
    voltageArray[0] = 10.0000 * scaledResultsSing[0]; // v5V_1
    voltageArray[1] = 10.0000 * scaledResultsSing[1]; // v5V_2
    voltageArray[2] = 10.0000 * scaledResultsSing[2]; // v3.3V_1
    voltageArray[3] = 10.0000 * scaledResultsSing[3]; // v3.3V_2
    
    // fill current array
    // ADCZeroTo2_5
    // current = (approx(~)2.42 - voltage reading) / 0.1 -> 100mV per Amp -> ~2.42 = voltage given when no current is sensed
    // in this wiring config of the sensors the output voltage decreases with an increase in current
    // currentArray[0] = (2.4200 - (2.5000 * scaledResultsSing[4])) / 0.1000; // iCell
    currentArray[0] = (2.500 - (2.5000 * scaledResultsSing[4])) / 0.1000; // iBatt
    currentArray[1] = (2.500 - (2.5000 * scaledResultsSing[5])) / 0.1000; // i5V_1
    currentArray[2] = (2.500 - (2.5000 * scaledResultsSing[6])) / 0.1000; // i5V_2
    currentArray[3] = (2.500 - (2.5000 * scaledResultsSing[7])) / 0.1000; // i3.3V_1
    currentArray[4] = (2.500 - (2.5000 * scaledResultsSing[8])) / 0.1000; // i3.3V_2

    // fill rest of voltage array
    // ADCZeroTo10
    voltageArray[4] = LTC2943Voltage;                 // vBatt
    voltageArray[5] = 10.0000 * scaledResultsSing[9]; // vCell

    // fill battery charge variable
    battCharge[0] = LTC2943AccuCharge;

    Serial.print("Qbat: ");
    Serial.print(LTC2943AccuCharge);
    Serial.println("mAh  ");

    taskEXIT_CRITICAL();

    Wire.beginTransmission(8);  // transmit to device #8
    splitFloatsIntoBytesAndSend(voltageArray, 6);
    Wire.endTransmission();    // stop transmitting
  
    Wire.beginTransmission(8);  // transmit to device #8
    splitFloatsIntoBytesAndSend(currentArray, 5);
    Wire.endTransmission();    // stop transmitting

    Wire.beginTransmission(8);  // transmit to device #8
    splitFloatsIntoBytesAndSend(battCharge, 1);
    Wire.endTransmission();    // stop transmitting


    /*
    // Differential results -> 4 to 19
    taskENTER_CRITICAL();
    MAX11300.burstAnalogRead(4, rawResultsDiff, 16);
    taskEXIT_CRITICAL();
    for (uint8_t i = 0; i < 8; i++) {
      scaledResultsDiff[i] = (10*((float)(rawResultsDiff[(2*i)+1]))/4096) - (10*((float)(rawResultsDiff[2*i]))/4096);
      //Serial.println(scaledResultsDiff[i]);
    }
    */

    /*
    // REMEMBER TO PASS AN int ARRAY NOT uint IF YOU WANT TO USE THIS GUY
    // Differentially configured results
    MAX11300.burstAnalogDiffRead(7, rawResultsDiff, 1);
    scaledResultsDiff[0] = (10*((float)(rawResultsDiff[0]))/2048); // for some reason differential reading is 2.67 ish volts out
    Serial.println(scaledResultsDiff[0]);
    */
  }
}

void TaskTempSamples(void *pvParameters) {
  (void) pvParameters;
  // Perform an action every 1000 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  for( ;; ) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    // check internal and battery temperature 
    taskENTER_CRITICAL();

    tInternal = MAX11300.readInternalTemp();
    tBatt = MAX11300.readExternalTemp1();
    tExternal2 = MAX11300.readExternalTemp2();

    taskEXIT_CRITICAL();

    // fill temp array
    tempArray[0] = tInternal;
    tempArray[1] = tBatt;
    tempArray[2] = tExternal2;

    Wire.beginTransmission(8);  // transmit to device #8
    splitFloatsIntoBytesAndSend(tempArray, 3);
    Wire.endTransmission();    // stop transmitting
  }
}

void TaskHeaterControl(void *pvParameters) {
  (void) pvParameters;
  // make a load of states for the ASM
  // maybe do PID control if you get into a state that needs const temp maintainance
  // run every time period that is not that quick as temperature changes slowly
  // Perform an action every 1000 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 50;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  double tBattFiltered = MAX11300.readExternalTemp1();
  double a = 0.1;      // filter coefficient
  double setPoint = 25;
  double hysteresis = 0.5;

  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    Serial.println("heater control");

    // if temp is below 5 degrees, heat. else don't heat??
    // PID use less power?
  
    tBatt = MAX11300.readExternalTemp1();
    tBattFiltered += a * (tBatt - tBattFiltered) ;                                      // first order low pass digital filter

    Serial.println(tBattFiltered);

    if (tBattFiltered >= (setPoint + hysteresis)) {                                     // above upper hysteresis threshold so stop heating
      digitalWrite(8, HIGH);
    }
    else if (tBattFiltered < (setPoint - hysteresis)) {                                 // below lower hysteresis threshold so start heating
      analogWrite(8, 0);  // duty cycle input is opposite with P-FET -> higher duty input = lower duty output
    }
    else if ((tBattFiltered < setPoint) && (tBattFiltered > (setPoint - hysteresis))) { // inside hysteresis so heat slower
      analogWrite(8, 10);
    }
  }
}

void TaskPowerControl(void *pvParameters) {
  (void) pvParameters;
  // make a load of states for the ASM
  // maybe do PID control if you get into a state that needs const temp maintainance
  // run every time period that is not that quick as temperature changes slowly
  // Perform an action every 1000 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );

    /*
    Power arbitration algorithm goes here
    */
    digitalWrite(9, LOW);
    // vTaskDelay(2);
    // digitalWrite(8, HIGH);
    // vTaskDelay(2);

    digitalWrite(10, LOW);
    // vTaskDelay(2);
    // digitalWrite(8, HIGH);
    // vTaskDelay(2);

    digitalWrite(11, LOW);
    // vTaskDelay(2);
    // digitalWrite(8, HIGH);
    // vTaskDelay(2);
  }
}

void receiveEvent(int numberOfBytesToReceive) {

}

ISR(TIMER1_COMPA_vect) { // Interrupt Vectors in ATmega328/P
  //period = 4.194 s
  //remember to make variables used in interrupts volatile
}

void PIN2_ISR() { // ISR for pin 2
  Serial.println("pin2 ISR");
}

void PIN3_ISR() { // ISR for pin 3
  Serial.println("pin3 ISR");
}

void setupTimerInterrupts() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 65534;     // must be 2 less than maximum of 65534
              
  // compare match register 16*10^6/(1024*65536) = 0.238 Hz, period = 4.194 s

  //TCCR1A –Timer/Counter Control Register A
  //TCCR1A[7:6]: Compare Output Mode for Channel
  //10 -> Clear OC1A on Compare Match 
  sbi (TCCR1A, COM1A1); //1
  cbi (TCCR1A, COM1A0); //0 

  //TCCR1A/B –Timer/Counter Control Register A/B
  //WGM[13:10]: Waveform Generation Mode  
  //CTC - Clear timer on compare-match of OCR1A mode. Mode 4
  //Combined with the WGM22 bit found in the TCCR2B Register, these bits control the counting sequence of the counter, the source for maximum (TOP) counter value, and what type of waveform generation to be used
  //0001 -> Mode 1  / Phase Correct PWM

  cbi (TCCR1B, WGM13); //0 
  sbi (TCCR1B, WGM12); //1
  cbi (TCCR1A, WGM11); //0  
  cbi (TCCR1A, WGM10); //0
  
  //TCCR1B –Timer/Counter Control Register B
  //CS[12:10]: Clock Select 
  //100 -> 1024 prescaler
  sbi (TCCR1B, CS12); //1
  cbi (TCCR1B, CS11); //0
  sbi (TCCR1B, CS10); //1

  //TIMSK1 -Timer/Counter 1 Interrupt Mask Register
  //OCIEA: Output Compare A Match Interrupt Enable
  sbi (TIMSK1, OCIE1A); //1
}

void setupExternalInterrupts() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  digitalWrite(2, HIGH);    // Enable pullup resistor
  digitalWrite(3, HIGH);    // Enable pullup resistor

  // done this way as using ISR(INT1_vect) and ISR(INT0_vect) conflicts with libraries in this project
  attachInterrupt(digitalPinToInterrupt(2), PIN2_ISR, LOW);
  attachInterrupt(digitalPinToInterrupt(3), PIN3_ISR, LOW);
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
