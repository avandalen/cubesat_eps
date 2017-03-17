#include <MAX11300.h>
#include <MAX11300registers.h>
#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include <PID_v1.h>
#include "avr/pgmspace.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  //clear
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   //set

uint8_t convertPin = 9;
uint8_t selectPin  = 6;
uint16_t MAX_ID    = 0;

const uint8_t numberOfSingEndSensors = 4;
const uint8_t numberOfDiffSensors    = 8; // 1/2 the number of channels that act as differential

//voltages
float vCell       = 0;
float vBatt       = 0;
float v3v3Bus     = 0;
float v5vBus      = 0;

/*
float vPort1      = 0;
float vPort2      = 0;
float vPort3      = 0;
float vPort4      = 0;
float vPort5      = 0;
float vPort6      = 0;
*/

// currents
float i_cell      = 0;
float iSystem     = 0;
float iPort1      = 0;
float iPort2      = 0;
float iPort3      = 0;
float iPort4      = 0;
float iPort5      = 0;
float iPort6      = 0;

// temperatures
double tExternal2   = 0;
double tExternal1   = 0;
double tInternal    = 0;

MAX11300 MAX11300(&SPI, convertPin, selectPin);

// Define Tasks
void TaskInternalTemp   (void *pvParameters);
void TaskObtainSamples  (void *pvParameters);
void TaskHeaterControl  (void *pvParameters);
void TaskBlink          (void *pvParameters);



void setup() {
  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  // Third argument is the number of words (not bytes!) to allocate for use as the task's stack.
  // ORDER THE TASKS BASED ON RUNTIME
  xTaskCreate(TaskObtainSamples,  (const portCHAR *) "ObtainSamples", 256, NULL, 3, NULL);
  xTaskCreate(TaskInternalTemp,   (const portCHAR *) "InternalTemp" , 128, NULL, 2, NULL);
  xTaskCreate(TaskHeaterControl,  (const portCHAR *) "HeaterControl", 128, NULL, 1, NULL);
  xTaskCreate(TaskBlink,          (const portCHAR *) "Blink"        , 128, NULL, 0, NULL);
  
  Serial.begin(9600);
  
  sei();                    // global enable interrupts
  setupTimerInterrupts();
  setupExternalInterrupts();

  // setup MCU as I2C slave so that the onboard computer can request power over I2C // slave sender, master reader, Sends data as an I2C/TWI slave device
  // also have the ground Arduino request data as a master

  // master writer, slave receiver -> master sends, slave prints out received message       <- for the EPS to 'ground' Arduino

  // master reader, slave sender   -> master requests, slave sends what has been requested  <- for the EPS to onboard computer
  
  MAX11300.begin();
  
  // Identify shield by ID register  
  MAX_ID = MAX11300.checkID();     
  if (MAX_ID == 0x0424 ) {
  Serial.print("Found MAX module ID: 0x");
  Serial.println(MAX_ID, HEX);
  }
  
  // set ADC mode
  // Continuous sweep through the specified pins
  MAX11300.setADCmode(ContinuousSweep);
  // if (MAX11300.getADCmode() == ContinuousSweep){Serial.println("ADC mode set to ContinuousSweep");}
  
  // set conversion rate
  // decrease this and increase the averaging amount to get
  // 400ksps
  MAX11300.setConversionRate(rate200ksps);
  // if (MAX11300.getConversionRate() == rate200ksps){Serial.println("ADC set to rate200ksps");}
  
  // set pin mode and voltage per pin

  // Positive analog input to single-ended ADC -> for system voltages
  MAX11300.setPinModeMAX(0, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(0) == analogIn){Serial.println("pin 0 set to ADC");} if(MAX11300.getPinADCRange(0) == ADCZeroTo10){Serial.println("pin0 set zero to 10");}
  MAX11300.setPinModeMAX(1, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(1) == analogIn){Serial.println("pin 1 set to ADC");} if(MAX11300.getPinADCRange(1) == ADCZeroTo10){Serial.println("pin1 set zero to 10");}
  MAX11300.setPinModeMAX(2, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(2) == analogIn){Serial.println("pin 2 set to ADC");} if(MAX11300.getPinADCRange(2) == ADCZeroTo10){Serial.println("pin2 set zero to 10");}
  MAX11300.setPinModeMAX(3, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(3) == analogIn){Serial.println("pin 3 set to ADC");} if(MAX11300.getPinADCRange(3) == ADCZeroTo10){Serial.println("pin3 set zero to 10");}
  
  // Positive analog input to single-ended ADC -> subtract pairs for system currents -> differential mode producing some spurious results
  MAX11300.setPinModeMAX(4, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(4) == analogIn){Serial.println("pin 4 set to ADC");} if(MAX11300.getPinADCRange(4) == ADCZeroTo10){Serial.println("pin4 set zero to 10");}
  MAX11300.setPinModeMAX(5, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(5) == analogIn){Serial.println("pin 5 set to ADC");} if(MAX11300.getPinADCRange(5) == ADCZeroTo10){Serial.println("pin5 set zero to 10");}
  MAX11300.setPinModeMAX(6, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(6) == analogIn){Serial.println("pin 6 set to ADC");} if(MAX11300.getPinADCRange(6) == ADCZeroTo10){Serial.println("pin6 set zero to 10");}
  MAX11300.setPinModeMAX(7, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(7) == analogIn){Serial.println("pin 7 set to ADC");} if(MAX11300.getPinADCRange(7) == ADCZeroTo10){Serial.println("pin7 set zero to 10");}
  MAX11300.setPinModeMAX(8, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(8) == analogIn){Serial.println("pin 8 set to ADC");} if(MAX11300.getPinADCRange(8) == ADCZeroTo10){Serial.println("pin8 set zero to 10");}
  MAX11300.setPinModeMAX(9, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(9) == analogIn){Serial.println("pin 9 set to ADC");} if(MAX11300.getPinADCRange(9) == ADCZeroTo10){Serial.println("pin9 set zero to 10");}
  MAX11300.setPinModeMAX(10, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(10) == analogIn){Serial.println("pin 10 set to ADC");} if(MAX11300.getPinADCRange(10) == ADCZeroTo10){Serial.println("pin10 set zero to 10");}
  MAX11300.setPinModeMAX(11, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(11) == analogIn){Serial.println("pin 11 set to ADC");} if(MAX11300.getPinADCRange(11) == ADCZeroTo10){Serial.println("pin11 set zero to 10");}
  MAX11300.setPinModeMAX(12, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(12) == analogIn){Serial.println("pin 12 set to ADC");} if(MAX11300.getPinADCRange(12) == ADCZeroTo10){Serial.println("pin12 set zero to 10");}
  MAX11300.setPinModeMAX(13, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(13) == analogIn){Serial.println("pin 13 set to ADC");} if(MAX11300.getPinADCRange(13) == ADCZeroTo10){Serial.println("pin13 set zero to 10");}
  MAX11300.setPinModeMAX(14, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(14) == analogIn){Serial.println("pin 14 set to ADC");} if(MAX11300.getPinADCRange(14) == ADCZeroTo10){Serial.println("pin14 set zero to 10");}
  MAX11300.setPinModeMAX(15, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(15) == analogIn){Serial.println("pin 15 set to ADC");} if(MAX11300.getPinADCRange(15) == ADCZeroTo10){Serial.println("pin15 set zero to 10");}
  MAX11300.setPinModeMAX(16, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(16) == analogIn){Serial.println("pin 16 set to ADC");} if(MAX11300.getPinADCRange(16) == ADCZeroTo10){Serial.println("pin16 set zero to 10");}
  MAX11300.setPinModeMAX(17, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(17) == analogIn){Serial.println("pin 17 set to ADC");} if(MAX11300.getPinADCRange(17) == ADCZeroTo10){Serial.println("pin17 set zero to 10");}
  MAX11300.setPinModeMAX(18, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(18) == analogIn){Serial.println("pin 18 set to ADC");} if(MAX11300.getPinADCRange(18) == ADCZeroTo10){Serial.println("pin18 set zero to 10");}
  MAX11300.setPinModeMAX(19, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(19) == analogIn){Serial.println("pin 19 set to ADC");} if(MAX11300.getPinADCRange(19) == ADCZeroTo10){Serial.println("pin19 set zero to 10");}


  // Positive analog input to differential ADC -> for system currents
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

void TaskObtainSamples(void *pvParameters) {
  (void) pvParameters;
  // Perform an action every 30 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  // NB/ ADC data format is straight binary in single-ended mode, and two’s complement in differential and pseudo- differential modes.
  
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10;
  // Initialise the xLastWakeTime variable with the current time.
  xLastWakeTime = xTaskGetTickCount();

  // initialise arrays to contain ADC data
  uint16_t  rawResultsSing[4]                           = {0};
  uint16_t  rawResultsDiff[16]                          = {0};
  // int16_t  rawResultsDiff[numberOfDiffSensors]       = {0};
  float    scaledResultsSing[4]                         = {0};
  float    scaledResultsDiff[8]                         = {0};

  for (;;) {
    /*
    uint16_t readAnalogPin (uint8_t pin);
    bool burstAnalogRead (uint16_t* samples, uint8_t size);
    bool isAnalogDataReady (uint8_t pin);
    bool isAnalogConversionComplete (void);
    void serviceInterrupt(void);
    */
    // Wait for the next cycle.

    vTaskDelayUntil( &xLastWakeTime, xFrequency);
    
    // Single-ended results  -> 0 to 3
    taskENTER_CRITICAL();
    MAX11300.burstAnalogRead(0, rawResultsSing, numberOfSingEndSensors);
    taskEXIT_CRITICAL();
    for (uint8_t i = 0; i < numberOfSingEndSensors; i++) {
      scaledResultsSing[i] = 10*((float)(rawResultsSing[0]))/4096; 
    }

    // Differential results -> 4 to 19
    taskENTER_CRITICAL();
    MAX11300.burstAnalogRead(4, rawResultsDiff, 16);
    taskEXIT_CRITICAL();
    for (uint8_t i = 0; i < 8; i++) {
      scaledResultsDiff[i] = (10*((float)(rawResultsDiff[(2*i)+1]))/4096) - (10*((float)(rawResultsDiff[2*i]))/4096);
      //Serial.println(scaledResultsDiff[i]);
    }

    /*
    // REMEMBER TO PASS AN int ARRAY NOT uint IF YOU WANT TO USE THIS GUY
    // Differentially configured results
    MAX11300.burstAnalogDiffRead(7, rawResultsDiff, 1);
    scaledResultsDiff[0] = (10*((float)(rawResultsDiff[0]))/2048); // for some reason differential reading is 2.67 ish volts out
    Serial.println(scaledResultsDiff[0]);
    */
  }
}

void TaskInternalTemp(void *pvParameters) {
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
    Serial.print("internal temperature = ");
    Serial.println(tInternal);

    tExternal1 = MAX11300.readExternalTemp1();
    Serial.print("External 1 temperature = ");
    Serial.println(tExternal1);

    tExternal2 = MAX11300.readExternalTemp2();
    Serial.print("External 2 temperature = ");
    Serial.println(tExternal2);
    Serial.println(" ");
    taskEXIT_CRITICAL();
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
  for (;;) {
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    Serial.println("heater control");

    // if temp is below 5 degrees, heat. else don't heat??
    // PID use less power?
    // 
  }
}

void TaskBlink(void *pvParameters) {
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
  for (;;) {
        // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    Serial.println("blink");
  }
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
