#include <MAX11300.h>
#include <MAX11300registers.h>
#include <Arduino_FreeRTOS.h>
#include <SPI.h>
#include "avr/pgmspace.h"

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))  //clear
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))   //set

uint8_t convertPin = 9;
uint8_t selectPin = 6;
uint16_t MAX_ID = 0;

const uint8_t numberOfSensors = 12;

//voltages
float vCell       = 0;
float vBatt       = 0;
float v3v3Bus     = 0;
float v5vBus      = 0;
float vPort1      = 0;
float vPort2      = 0;
float vPort3      = 0;

// currents
float i_cell      = 0;
float iSystem     = 0;
float iPort1      = 0;
float iPort2      = 0;
float iPort3      = 0;

// temperatures
double tBattery     = 0;
double tInternal = 0;

MAX11300 MAX11300(&SPI, convertPin, selectPin);

// Define Tasks
void TaskInternalTemp   (void *pvParameters);
void TaskBlink          (void *pvParameters);
void TaskObtainSamples     (void *pvParameters);


void setup() {
  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  // Third argument is the number of words (not bytes!) to allocate for use as the task's stack.
  xTaskCreate(TaskObtainSamples,  (const portCHAR *) "ObtainSamples", 128, NULL, 1, NULL);
  xTaskCreate(TaskInternalTemp,   (const portCHAR *) "InternalTemp" , 128, NULL, 4, NULL);
  xTaskCreate(TaskBlink,          (const portCHAR *) "Blink"        , 128, NULL, 3, NULL);
  
  Serial.begin(9600);
  
  sei();                    // global enable interrupts
  setupTimerInterrupts();
  setupExternalInterrupts();
  
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
  if (MAX11300.getADCmode() == ContinuousSweep){Serial.println("ADC mode set to ContinuousSweep");}
  
  // set conversion rate
  // 400ksps
  MAX11300.setConversionRate(rate400ksps);
  if (MAX11300.getConversionRate() == rate400ksps){Serial.println("ADC set to rate400ksps");}
  
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
  MAX11300.setPinModeMAX(4, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(4) == analogIn){Serial.println("pin 4 set to ADC");} if(MAX11300.getPinADCRange(4) == ADCZeroTo10){Serial.println("pin4 set zero to 10");}
  MAX11300.setPinModeMAX(5, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(5) == analogIn){Serial.println("pin 5 set to ADC");} if(MAX11300.getPinADCRange(5) == ADCZeroTo10){Serial.println("pin5 set zero to 10");}
  MAX11300.setPinModeMAX(6, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(6) == analogIn){Serial.println("pin 6 set to ADC");} if(MAX11300.getPinADCRange(6) == ADCZeroTo10){Serial.println("pin6 set zero to 10");}

  // Positive analog input to differential ADC -> for system currents
  // setPinModeMAX(uint8_t pin, pinMode_t mode, ADCRange_t ADCrange, uint8_t differentialPin);

  MAX11300.setPinModeMAX(7, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 8);
  /*
  if (MAX11300.getPinModeMAX(7) == analogIn){Serial.println("pin 7 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(7)); Serial.println(" is pin 7's differential partner");  
  if(MAX11300.getPinADCRange(7) == ADCZeroTo10){Serial.println("pin 7 set zero to 10");}
  if(MAX11300.getPinADCRange(8) == ADCZeroTo10){Serial.println("pin 8 set zero to 10");}
  */

  MAX11300.setPinModeMAX(9, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 10);
  /*
  if (MAX11300.getPinModeMAX(9) == analogIn){Serial.println("pin 9 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(9)); Serial.println(" is pin 9's differential partner");  
  if(MAX11300.getPinADCRange(9) == ADCZeroTo10){Serial.println("pin 9 set zero to 10");}
  if(MAX11300.getPinADCRange(10) == ADCZeroTo10){Serial.println("pin 10 set zero to 10");}
  */

  MAX11300.setPinModeMAX(11, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 12);
  /*
  if (MAX11300.getPinModeMAX(11) == analogIn){Serial.println("pin 11 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(11)); Serial.println(" is pin 11's differential partner");  
  if(MAX11300.getPinADCRange(11) == ADCZeroTo10){Serial.println("pin 11 set zero to 10");}
  if(MAX11300.getPinADCRange(12) == ADCZeroTo10){Serial.println("pin 12 set zero to 10");}
  */

  MAX11300.setPinModeMAX(13, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 14);
  /*
  if (MAX11300.getPinModeMAX(13) == analogIn){Serial.println("pin 13 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(13)); Serial.println(" is pin 13's differential partner");  
  if(MAX11300.getPinADCRange(13) == ADCZeroTo10){Serial.println("pin 13 set zero to 10");}
  if(MAX11300.getPinADCRange(14) == ADCZeroTo10){Serial.println("pin 14 set zero to 10");}
  */

  MAX11300.setPinModeMAX(15, MAX_FUNCID_ADC_DIFF_POS, ADCZeroTo10, 16);
  /*
  if (MAX11300.getPinModeMAX(15) == analogIn){Serial.println("pin 15 set to differential ADC");}
  Serial.print("pin "); Serial.print(MAX11300.getDifferentialPin(15)); Serial.println(" is pin 15's differential partner");  
  if(MAX11300.getPinADCRange(15) == ADCZeroTo10){Serial.println("pin 15 set zero to 10");}
  if(MAX11300.getPinADCRange(16) == ADCZeroTo10){Serial.println("pin 16 set zero to 10");}
  */

  // set voltage reference per pin
  // internal
  for (uint8_t i = 0; i < 16; i++){
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
  for (uint8_t i = 0; i < 16; i++){
    MAX11300.setPinAveraging (i, 128);
    Serial.println(MAX11300.getPinAveraging(i));
  }
  
  // enable required temperature sensors
  // 3 sensors can be enables
  // ext2 ext1 internal 
  // count up in 3 bits
  // ie// mode 5 -> 101 -> ext2 enabled and internal enables
  MAX11300.setTempSensorEnableMode(mode1);
  if (MAX11300.getTempSensorEnableMode() == mode0){Serial.println("temp mode0");}
 
}
void loop(){
  // Empty. Things are done in Tasks.
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskObtainSamples(void *pvParameters){
  // Perform an action every 30 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  // NB/ ADC data format is straight binary in single-ended mode, and two’s complement in differential and pseudo- differential modes.
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 20;

  uint16_t rawResults[1]    = {0};
  float    scaledResults[1] = {0};

  /*
  uint16_t rawResults[numberOfSensors]    = {0};
  float    scaledResults[numberOfSensors] = {0};
  */
  for (;;){
    /*
    uint16_t readAnalogPin (uint8_t pin);
    bool burstAnalogRead (uint16_t* samples, uint8_t size);
    bool isAnalogDataReady (uint8_t pin);
    bool isAnalogConversionComplete (void);
    void serviceInterrupt(void);
    */
    // Wait for the next cycle.
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    taskENTER_CRITICAL();
    MAX11300.burstAnalogRead(7, rawResults, 1);
    scaledResults[0] = (((float)(rawResults[0]))*10)/4096;
    Serial.println(scaledResults[0]);
    /*
    for (uint8_t i = 0; i < numberOfSensors; i++) {
    MAX11300.burstAnalogRead(0, rawResults, numberOfSensors);
    scaledResults[i] = (((float)(rawResults[i]))*10)/4096;
    Serial.println(scaledResults[0]);
    }
    */
    taskEXIT_CRITICAL();
  }
}

void TaskInternalTemp(void *pvParameters) {
  // Perform an action every 1000 ticks.
  // 1 tick = 15ms (ATMEGA watchdog timer used)
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1000;
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
    taskEXIT_CRITICAL();
  }
}

void TaskBlink(void *pvParameters) {
  (void) pvParameters;
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    //Serial.println("hello");
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
