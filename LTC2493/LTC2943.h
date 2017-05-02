#ifndef __LTC2493_REG_H__
#define __LTC2493_REG_H__


// Define register addresses LTC2493...
#define LTC2943_ADDRESS     0x64
#define LTC_STATUS          0x00
#define LTC_CONTROL         0x01
#define LTC_ACCU_CHARGE_MSB 0x02
#define LTC_VOLTAGE_MSB     0x08
#define LTC_CURRENT_MSB     0x0E
#define LTC_TEMP_MSB        0x14

#define SUTDOWN_MASK        0x01
#define ADC_START_CONV_MASK 0XC0

// ACTUAL VALUE IS 0.01000 but this gives a correct current value from the onboard ADC 
// ie the value that gives correct results is 10/3 times smaller
// using actual value q_LSB = 0.34mAh * 50mOhm/10mOhm * 256/4096 = 0.10625 mAh
// using this value q_LSB = 0.34mAh * 50mOhm/3mOhm * 64/4096 = 0.08854 mAh
// #define senseResistor       0.01000
#define senseResistor       0.00300

#endif