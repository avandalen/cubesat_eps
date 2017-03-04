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

MAX11300 MAX11300(&SPI, convertPin, selectPin);

// Define Tasks
void TaskObtainSamples  (void *pvParameters);
void TaskInternalTemp   (void *pvParameters);
void TaskBlink          (void *pvParameters);
void TaskAnalogRead     (void *pvParameters);


void setup() {
  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
  xTaskCreate(TaskObtainSamples,  (const portCHAR *) "ObtainSamples", 128, NULL, 1, NULL);
  xTaskCreate(TaskInternalTemp,   (const portCHAR *) "InternalTemp" , 128, NULL, 4, NULL);
  xTaskCreate(TaskBlink,          (const portCHAR *) "Blink"        , 128, NULL, 3, NULL);
  xTaskCreate(TaskAnalogRead,     (const portCHAR *) "AnalogRead"   , 128, NULL, 2, NULL);
  
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
  MAX11300.setPinModeMAX(0, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(0) == analogIn){Serial.println("pin0 set to ADC");} if(MAX11300.getPinADCRange(0) == ADCZeroTo10){Serial.println("pin0 set zero to 10");}
  MAX11300.setPinModeMAX(1, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(1) == analogIn){Serial.println("pin1 set to ADC");} if(MAX11300.getPinADCRange(1) == ADCZeroTo10){Serial.println("pin1 set zero to 10");}
  MAX11300.setPinModeMAX(2, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(2) == analogIn){Serial.println("pin2 set to ADC");} if(MAX11300.getPinADCRange(2) == ADCZeroTo10){Serial.println("pin2 set zero to 10");}
  MAX11300.setPinModeMAX(3, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(3) == analogIn){Serial.println("pin3 set to ADC");} if(MAX11300.getPinADCRange(3) == ADCZeroTo10){Serial.println("pin3 set zero to 10");}
  MAX11300.setPinModeMAX(4, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(4) == analogIn){Serial.println("pin4 set to ADC");} if(MAX11300.getPinADCRange(4) == ADCZeroTo10){Serial.println("pin4 set zero to 10");}
  MAX11300.setPinModeMAX(5, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(5) == analogIn){Serial.println("pin5 set to ADC");} if(MAX11300.getPinADCRange(5) == ADCZeroTo10){Serial.println("pin5 set zero to 10");}
  MAX11300.setPinModeMAX(6, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(6) == analogIn){Serial.println("pin6 set to ADC");} if(MAX11300.getPinADCRange(6) == ADCZeroTo10){Serial.println("pin6 set zero to 10");} 
  MAX11300.setPinModeMAX(7, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(7) == analogIn){Serial.println("pin7 set to ADC");} if(MAX11300.getPinADCRange(7) == ADCZeroTo10){Serial.println("pin7 set zero to 10");}
  MAX11300.setPinModeMAX(8, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(8) == analogIn){Serial.println("pin8 set to ADC");} if(MAX11300.getPinADCRange(8) == ADCZeroTo10){Serial.println("pin8 set zero to 10");}
  MAX11300.setPinModeMAX(9, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(9) == analogIn){Serial.println("pin9 set to ADC");} if(MAX11300.getPinADCRange(9) == ADCZeroTo10){Serial.println("pin9 set zero to 10");}
  MAX11300.setPinModeMAX(10, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(10) == analogIn){Serial.println("pin10 set to ADC");} if(MAX11300.getPinADCRange(10) == ADCZeroTo10){Serial.println("pin10 set zero to 10");}
  MAX11300.setPinModeMAX(11, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(11) == analogIn){Serial.println("pin11 set to ADC");} if(MAX11300.getPinADCRange(11) == ADCZeroTo10){Serial.println("pin11 set zero to 10");}
  MAX11300.setPinModeMAX(12, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(12) == analogIn){Serial.println("pin12 set to ADC");} if(MAX11300.getPinADCRange(12) == ADCZeroTo10){Serial.println("pin12 set zero to 10");}
  MAX11300.setPinModeMAX(13, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(13) == analogIn){Serial.println("pin13 set to ADC");} if(MAX11300.getPinADCRange(13) == ADCZeroTo10){Serial.println("pin13 set zero to 10");}
  MAX11300.setPinModeMAX(14, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(14) == analogIn){Serial.println("pin14 set to ADC");} if(MAX11300.getPinADCRange(14) == ADCZeroTo10){Serial.println("pin14 set zero to 10");}
  MAX11300.setPinModeMAX(15, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(15) == analogIn){Serial.println("pin15 set to ADC");} if(MAX11300.getPinADCRange(15) == ADCZeroTo10){Serial.println("pin15 set zero to 10");}
  MAX11300.setPinModeMAX(16, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(16) == analogIn){Serial.println("pin16 set to ADC");} if(MAX11300.getPinADCRange(16) == ADCZeroTo10){Serial.println("pin16 set zero to 10");}
  MAX11300.setPinModeMAX(17, MAX_FUNCID_ADC, ADCZeroTo10);  
  //if (MAX11300.getPinModeMAX(17) == analogIn){Serial.println("pin17 set to ADC");} if(MAX11300.getPinADCRange(17) == ADCZeroTo10){Serial.println("pin17 set zero to 10");}
  MAX11300.setPinModeMAX(18, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(18) == analogIn){Serial.println("pin18 set to ADC");} if(MAX11300.getPinADCRange(18) == ADCZeroTo10){Serial.println("pin18 set zero to 10");}
  MAX11300.setPinModeMAX(19, MAX_FUNCID_ADC, ADCZeroTo10);
  //if (MAX11300.getPinModeMAX(19) == analogIn){Serial.println("pin19 set to ADC");} if(MAX11300.getPinADCRange(19) == ADCZeroTo10){Serial.println("pin19 set zero to 10");}

  // set voltage reference per pin
  // internal
  for (uint8_t i=0; i<20; i++){
    MAX11300.setPinADCref(i, ADCInternal);
    if (MAX11300.getPinADCref(i) == ADCInternal){
        Serial.print(i);
        Serial.println(" pin ref ADCInternal");
      }
  }
  
  // set averaging per pin
  // set to average 4 samples before loading into ADC reg for pin
  for (uint8_t i=0; i<20; i++){
    MAX11300.setPinAveraging (i, 4);
    Serial.println(MAX11300.getPinAveraging(i));
  }
  
  // enable required temperature sensors
  // 3 sensors can be enables
  // ext2 ext1 internal 
  // count up in 3 bits
  // ie// mode 5 -> 101 -> ext2 enabled and internal enables
  MAX11300.setTempSensorEnableMode(mode1);
  if (MAX11300.getTempSensorEnableMode() == mode0){Serial.println("temp mode0");}
  else if (MAX11300.getTempSensorEnableMode() == mode1){Serial.println("temp mode1");}
  else if (MAX11300.getTempSensorEnableMode() == mode2){Serial.println("temp mode2");}
  else if (MAX11300.getTempSensorEnableMode() == mode3){Serial.println("temp mode3");}
  else if (MAX11300.getTempSensorEnableMode() == mode4){Serial.println("temp mode4");}
  else if (MAX11300.getTempSensorEnableMode() == mode5){Serial.println("temp mode5");}
  else if (MAX11300.getTempSensorEnableMode() == mode6){Serial.println("temp mode6");}
  else if (MAX11300.getTempSensorEnableMode() == mode7){Serial.println("temp mode7");}
 
}
void loop(){
  // Empty. Things are done in Tasks.
}
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskObtainSamples(void *pvParameters){
  for (;;){
    /*
    uint16_t readAnalogPin (uint8_t pin);
    bool burstAnalogRead (uint16_t* samples, uint8_t size);
    bool isAnalogDataReady (uint8_t pin);
    bool isAnalogConversionComplete (void);
    void serviceInterrupt(void);
    taskENTER_CRITICAL();
    uint16_t testResult = MAX11300.readAnalogPin(0);
    taskEXIT_CRITICAL();
    */
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

void TaskInternalTemp(void *pvParameters){
  double internalTemp = 0;
  for (;;){
    taskENTER_CRITICAL();
    internalTemp = MAX11300.readInternalTemp();
    //Serial.println(internalTemp);
    taskEXIT_CRITICAL();
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
void TaskBlink(void *pvParameters){
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
void TaskAnalogRead(void *pvParameters){
  (void) pvParameters;
  for (;;)
  {
    int sensorValue = analogRead(A0);
    //Serial.println("hello");
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

ISR(TIMER1_COMPA_vect) { // Interrupt Vectors in ATmega328/P
  //period = 4.194 s
  //remember to make variables used in interrupts volatile 
  Serial.println("in timer ISR");
  // check internal and battery temperature
}

void PIN2_ISR() { // ISR for pin 2
  Serial.println("pin2 ISR");
}

void PIN3_ISR() { // ISR for pin 3
  Serial.println("pin3 ISR");
}


/*
// Install the interrupt routine.
ISR(INT0_vect) {
  Serial.println("int 0 ISR");
}

// Install the interrupt routine.
ISR(INT1_vect) {
  Serial.println("int 1 ISR");
}
*/

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
  /*
  // INT1/0: External Interrupt Request 1/0 Enable
  sbi (EIMSK, INT1);  //1
  sbi (EIMSK, INT0);  //1
 
  //00 The low level of INT1/0 generates an interrupt request.
  //01 Any logical change on INT1/0 generates an interrupt request.
  //10 The falling edge of INT1/0 generates an interrupt request.
  //11 The rising edge of INT1/0 generates an interrupt request.
  //INT1
  sbi (EICRA, ISC11); //1
  sbi (EICRA, ISC10); //1
  // INT0
  sbi (EICRA, ISC01); //1
  sbi (EICRA, ISC00); //1
}
*/
