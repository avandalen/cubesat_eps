#include <Arduino_FreeRTOS.h>
#include "MAX11300.h"

// default values
#define DEFAULT_THRESHOLD	0x03ff
#define TEMP_LSB			(0.125/16)

MAX11300::MAX11300(SPIClass *spi, uint8_t convertPin, uint8_t selectPin) {
	_convertPin = convertPin;
	_spi = spi;
	_select = selectPin;
	_interrupt = 255;
	_analogStatus = 0;
	pinMode(_convertPin, OUTPUT);
	pinMode(_select, OUTPUT);
	digitalWrite(_convertPin, HIGH);
	digitalWrite(_select, HIGH);
	_spiMode = new SPISettings(10000000, MSBFIRST, SPI_MODE0);
	// originally _spiMode = new SPISettings(20000000L, LSBFIRST, SPI_MODE0);
	// shouldnt it be MSBFIRST?
	// Arduino will automatically use the best speed that is equal to or less than the number you use with SPISettings. 
	_spi->begin();
}

MAX11300::MAX11300(SPIClass *spi, uint8_t convertPin, uint8_t selectPin, uint8_t interruptNumber) {
	_convertPin = convertPin;
	_spi = spi;
	_select = selectPin;
	_interrupt = interruptNumber;
	_analogStatus = 0;
	pinMode(_convertPin, OUTPUT);
	pinMode(_select, OUTPUT);
	digitalWrite(_convertPin, HIGH);
	digitalWrite(_select, HIGH);
	_spiMode = new SPISettings(10000000, MSBFIRST, SPI_MODE0);
	// originally _spiMode = new SPISettings(20000000L, LSBFIRST, SPI_MODE0);
	// shouldnt it be MSBFIRST?
	// Arduino will automatically use the best speed that is equal to or less than the number you use with SPISettings. 
	_spi->begin();
	_spi->usingInterrupt(_interrupt);
}

bool MAX11300::begin(void) {
	//if (_interrupt < 255) {attachInterrupt(_interrupt, MAX11300::serviceInterrupt, FALLING);}
	return true;
}

bool MAX11300::end(void) {
	if (_interrupt < 255) detachInterrupt(_interrupt);
	return true;
}

uint16_t MAX11300::checkID(void) {
	uint16_t val = 0;
	readRegister(MAX_DEVICE_ID, &val, 1);
	return val;
}

bool MAX11300::setPinModeMAX(uint8_t pin, pinMode_t mode) {
	return this->setPinModeMAX(pin, mode, 255, 255);
}

bool MAX11300::setPinModeMAX(uint8_t pin, pinMode_t mode, ADCRange_t ADCrange) {
	return this->setPinModeMAX(pin, mode, ADCrange, 255);
}

bool MAX11300::setPinModeMAX(uint8_t pin, pinMode_t mode, ADCRange_t ADCrange, uint8_t differentialPin) {
	uint16_t configuration;
	uint16_t diffConf;
	uint16_t adjacentConf;

	if ((mode == analogDifferential) && (differentialPin > 19)) return false;	// if we don't have a valid additional pin, bail
	if ((mode == biderectionalTrans) && ((pin+1)		 > 19)) return false;	// if we don't have a valid additional pin, bail
	if (differentialPin == pin) return false;									// this configuration is invalid, so bail

	configuration = readRegister(MAX_FUNC_BASE + pin);							// read in the existing configuration so we don't stomp it
	diffConf 	  = readRegister(MAX_FUNC_BASE + differentialPin);				// read in the existing configuration on diff pin
	adjacentConf  = readRegister(MAX_FUNC_BASE + (pin +1));				        // read in the existing configuration on adjacent pin
	switch (mode) {
		case digitalIn:
			if (getPinThreshold(pin) == 0) {
				setPinThreshold(pin, DEFAULT_THRESHOLD);
				delay(1);
			}
			configuration = MAX_FUNCID_GPI;
			break;
		case digitalOut:
			if (getPinThreshold(pin) == 0) {
				setPinThreshold(pin, DEFAULT_THRESHOLD);
				delay(1);
			}
			configuration = MAX_FUNCID_GPO;
			break;
		case biderectionalTrans:
			configuration = (configuration & ~(MAX_FUNCID_MASK)) | MAX_FUNCID_LVL_TRANS_BI;	// set pin mode to ADC
			adjacentConf  = MAX_FUNCID_HI_Z;												// set adjacent pin to High Z mode
			if (!writeRegister((MAX_FUNC_BASE + (pin + 1)), adjacentConf)) return false;
			break;
		case analogIn:
			configuration = (configuration & ~(MAX_FUNCID_MASK)) | MAX_FUNCID_ADC;	// set pin mode to ADC
			configuration = (configuration & ~(MAX_FUNCPRM_RANGE_MASK)) | ADCrange; // set the ADC range here	
			break;
		case analogOut:
			configuration = (configuration & ~(MAX_FUNCID_MASK)) | MAX_FUNCID_DAC;
			break;
		case analogDifferential:
			configuration = (configuration & ~(MAX_FUNCID_MASK)) | MAX_FUNCID_ADC_DIFF_POS; // set pin mode to positive differential ADC
			configuration = (configuration & ~(MAX_FUNCPRM_RANGE_MASK)) | MAX_DAC_RANGE_5_5; 		// set the ADC range here
			configuration = (configuration & ~(MAX_FUNCPRM_ASSOC_MASK)) | (MAX_FUNCPRM_ASSOC_BASE + differentialPin); // set diff pin to associated pin

			diffConf 	  = (diffConf & ~(MAX_FUNCID_MASK)) | MAX_FUNCID_ADC_DIFF_NEG;   	// set pin mode of diff pin to negative differential ADC
			diffConf	  = (diffConf & ~(MAX_FUNCPRM_RANGE_MASK)) | MAX_DAC_RANGE_5_5; 		        // set the ADC range here	
			//diffConf	  = (diffConf & ~(MAX_FUNCPRM_ASSOC_MASK)) | (MAX_FUNC_BASE + pin); // set associated pin of diff pin as original pin  
			if (!writeRegister((MAX_FUNC_BASE + differentialPin), diffConf)) return false;
			break;
		case highImpedance:
			configuration = MAX_FUNCID_HI_Z;
			break;
		default:
			return false;
	}
	if (writeRegister((MAX_FUNC_BASE + pin), configuration)) return true;
	return false;
}

pinMode_t MAX11300::getPinModeMAX(uint8_t pin) {
	uint16_t conf = readRegister(MAX_FUNC_BASE + pin);
	conf &= MAX_FUNCID_MASK;
	if (conf == MAX_FUNCID_HI_Z) 			return highImpedance;
	if (conf == MAX_FUNCID_GPI) 			return digitalIn;
	if (conf == MAX_FUNCID_GPO) 			return digitalOut;
	if (conf == MAX_FUNCID_LVL_TRANS_BI) 	return biderectionalTrans;
	if (conf == MAX_FUNCID_ADC) 			return analogIn;
	if (conf == MAX_FUNCID_DAC) 			return analogOut;
	if (conf == MAX_FUNCID_ADC_DIFF_POS) 	return analogDifferential;
	if (conf == MAX_FUNCID_ADC_DIFF_NEG) 	return differentialNegative;
	return pinModeNONE;
}

ADCRange_t MAX11300::getPinADCRange(uint8_t pin) {
	uint16_t conf = readRegister(MAX_FUNC_BASE + pin);
	conf &= MAX_FUNCPRM_RANGE_MASK;
	if (conf == MAX_ADC_RANGE_0_10) 	return ADCZeroTo10;
	if (conf == MAX_ADC_RANGE_5_5) 		return ADCNegative5to5;
	if (conf == MAX_ADC_RANGE_10_0) 	return ADCNegative10to0;
	if (conf == MAX_ADC_RANGE_0_2_5) 	return ADCZeroTo2_5;
	return ADCrangeNONE;
}

int8_t MAX11300::getDifferentialPin(uint8_t pin) {
	uint16_t conf = readRegister(MAX_FUNC_BASE + pin);
	return (conf & MAX_FUNCPRM_ASSOC_MASK);
}
	
bool MAX11300::setPinThreshold(uint8_t pin, uint16_t voltage) {
	return writeAnalogPin(pin, voltage);
}

uint16_t MAX11300::getPinThreshold(uint8_t pin) {
	return readRegister(MAX_DACDAT_BASE + pin);
}

bool MAX11300::setDigitalInputMode(uint8_t pin, GPImode_t mode) {
	uint8_t gpiAddress, gpiOffset;
	uint16_t gpimdRegister;
	if (pin > 15) {
		gpiAddress = MAX_GPIMD_16_19;
		gpiOffset = 2*(pin - 16);
	} else if (pin > 7) {
		gpiAddress = MAX_GPIMD_8_15;
		gpiOffset = 2*(pin - 8);
	} else {
		gpiAddress = MAX_GPIMD_0_7;
		gpiOffset = 2*pin;
	}
	return readModifyWriteRegister(gpiAddress, (0x03 << gpiOffset), ((uint16_t)(mode) << gpiOffset));
}

GPImode_t MAX11300::getDigitalInputMode(uint8_t pin) {
	uint8_t gpiAddress, gpiOffset;
	uint16_t gpimdRegister;
	if (pin > 15) {
		gpiAddress = MAX_GPIMD_16_19;
		gpiOffset = 2*(pin - 16);
	} else if (pin > 7) {
		gpiAddress = MAX_GPIMD_8_15;
		gpiOffset = 2*(pin - 8);
	} else {
		gpiAddress = MAX_GPIMD_0_7;
		gpiOffset = 2*pin;
	}
	gpimdRegister = ((readRegister(gpiAddress) >> gpiOffset) && 0x03);
	if (gpimdRegister == 0x0) return GPIneither;
	if (gpimdRegister == 0x1) return GPIpositive;
	if (gpimdRegister == 0x2) return GPInegative;
	if (gpimdRegister == 0x3) return GPIboth;
	return GPINONE;
}

uint16_t MAX11300::readAnalogPin (uint8_t pin) {
	switch(getADCmode()) {
		case Idle:
			return 0;
			break;
		case SingleSweep:
		case SingleSample:
			while (!(isAnalogDataReady(pin))) {
				startConversion();
				while (!(isAnalogConversionComplete()));
			}
			break;
		case ContinuousSweep:
			break;
		default:
			return 0;
	}
	_analogStatus &= ~(1 << pin);					// show that we've read the data from that pin
	return readRegister(MAX_ADCDAT_BASE + pin);
}

bool MAX11300::readDigitalPin (uint8_t pin) {
	uint8_t address, offset;
	uint16_t reg;
	if (pin > 15) {
		address = MAX_GPIDAT_H;
		offset = pin - 15;
	} else {
		address = MAX_GPIDAT_L;
		offset = pin;
	}
	reg = readRegister(address);
	return (bool)((reg >> offset) & 1);
}

bool MAX11300::writeAnalogPin (uint8_t pin, uint16_t value) {
	return writeRegister((MAX_DACDAT_BASE + pin), value);
}

bool MAX11300::writeDigitalPin (uint8_t pin, bool value) {
	uint8_t address, offset;
	uint16_t reg;
	if (pin > 15) {
		address = MAX_GPODAT_H;
		offset = pin - 15;
	} else {
		address = MAX_GPODAT_L;
		offset = pin;
	}
	reg = readRegister(address);
	if (value) {
		reg |= 1 << offset;
	} else {
		reg &= ~(1 << offset);
	}
	return writeRegister(address, reg);
	
}

bool MAX11300::setPinAveraging (uint8_t pin, uint8_t samples) {
	uint16_t val = 0;
	switch (samples) {
		case 1:				// no avg
			val = 0x0000;
			break;
		case 2:             // 2 samples
			val = 0x0020;
			break;
		case 4:				// 4 samples
			val = 0x0040;
			break;
		case 8:				// 8 samples
			val = 0x0060;
			break;
		case 16:			// 16 samples
			val = 0x0080;
			break;
		case 32:			// 32 samples
			val = 0x00a0;;
			break;
		case 64: 			// 64 samples
			val = 0x00c0;
			break;
		case 128:			// 128 samples
			val = 0x00e0;
			break;
		default:
			return false;
	}
	return readModifyWriteRegister((MAX_FUNC_BASE + pin), MAX_FUNCPRM_AVG_MASK, val);
}

uint8_t MAX11300::getPinAveraging (uint8_t pin) {
	uint16_t reg = readRegister(MAX_FUNC_BASE + pin);
	uint16_t base = 2;
	reg = (reg & MAX_FUNCPRM_AVG_MASK) >> 5;
	return (1 << reg);
}

bool MAX11300::setPinADCref (uint8_t pin, ADCref_t reference) {
	return readModifyWriteRegister((MAX_FUNC_BASE + pin), MAX_FUNCPRM_AVR_MASK, (uint16_t)(reference));
}

ADCref_t MAX11300::getPinADCref (uint8_t pin) {
	uint16_t reg = readRegister(MAX_FUNC_BASE + pin);
	if (reg & MAX_FUNCPRM_AVR_MASK) return ADCExternal;
	return ADCInternal;
}

bool MAX11300::setDACref (DACref_t reference) {
	return readModifyWriteRegister(MAX_DEVCTL, (1 << MAX_DACREF), (uint16_t)(reference));
}

DACref_t MAX11300::getDACref (void) {
	uint16_t reg = readRegister(MAX_DEVCTL);
	if (reg & (1 << MAX_DACREF)) return DACExternal;
	return DACInternal;
}

bool MAX11300::setConversionRate(conversionRate_t rate) {
	return readModifyWriteRegister(MAX_DEVCTL, MAX_ADCCONV_MASK, (uint16_t)(rate));
}

conversionRate_t MAX11300::getConversionRate(void) {
	uint16_t reg = readRegister(MAX_DEVCTL);
	reg &= MAX_ADCCONV_MASK;
	if (reg == MAX_RATE_200) return rate200ksps;
	if (reg == MAX_RATE_250) return rate250ksps;
	if (reg == MAX_RATE_333) return rate333ksps;
	if (reg == MAX_RATE_400) return rate400ksps;
	return rateNONE;
}

bool MAX11300::setTempSensorEnableMode(tempSensorEnableMode_t mode){
	return readModifyWriteRegister(MAX_DEVCTL, MAX_TMPCTL_MASK, (uint16_t)(mode));
}

tempSensorEnableMode_t MAX11300::getTempSensorEnableMode(void){
	uint16_t reg = readRegister(MAX_DEVCTL);
	reg &= MAX_TMPCTL_MASK;
	if (reg == MAX_TMPCTL_ENABLE_MODE_0) return mode0;
	if (reg == MAX_TMPCTL_ENABLE_MODE_1) return mode1;
	if (reg == MAX_TMPCTL_ENABLE_MODE_2) return mode2;
	if (reg == MAX_TMPCTL_ENABLE_MODE_3) return mode3;
	if (reg == MAX_TMPCTL_ENABLE_MODE_4) return mode4;
	if (reg == MAX_TMPCTL_ENABLE_MODE_5) return mode5;
	if (reg == MAX_TMPCTL_ENABLE_MODE_6) return mode6;
	if (reg == MAX_TMPCTL_ENABLE_MODE_7) return mode7;
	return modeNONE;
}

bool MAX11300::setTempAveraging (tempSensor_t sensor, uint8_t samples) {
	uint16_t configuration;
	uint16_t val = 0;
	configuration = readRegister(MAX_TMPMONCFG);							// read in the existing configuration so we don't stomp it
	switch (samples) {
		case 4:				// 4 samples -> mode 0
			val = 0x0000;
			break;
		case 8:             // 8 samples -> mode 1
			val = 0x0001;
			break;
		case 16:			// 16 samples -> mode 2
			val = 0x0002;
			break;
		case 32:			// 32 samples -> mode 3
			val = 0x0003;
			break;
		default:
			return false;
	}
	switch (sensor) {
		case internal:
			configuration = (configuration & ~(MAX_TMPINTMONCFG_MASK)) | val;
			break;
		case external1:
			val = val << 2;
			configuration = (configuration & ~(MAX_TMPEXT1MONCFG_MASK)) | val;
			break;
		case external2:
			val = val << 4;
			configuration = (configuration & ~(MAX_TMPEXT2MONCFG_MASK)) | val;
			break;
	}
	if (writeRegister(MAX_TMPMONCFG, configuration)) return true;
	return false;
}

uint16_t MAX11300::getTempAveraging (tempSensor_t sensor) {
	uint16_t reg = readRegister(MAX_TMPMONCFG);
	switch (sensor) {
		case internal:
			reg = reg & MAX_TMPINTMONCFG_MASK;
			break;
		case external1:
			reg = reg & MAX_TMPEXT1MONCFG_MASK;
			reg = reg >> 2;
			break;
		case external2:
			reg = reg & MAX_TMPEXT2MONCFG_MASK;
			reg = reg >> 4;
			break;
	}
	return reg;
}

double MAX11300::readInternalTemp (void) {
	return convertTemp(readRegister(MAX_TMPINTDAT));
}

double MAX11300::readExternalTemp1 (void) {
	return convertTemp(readRegister(MAX_TMPEXT1DAT));
}

double MAX11300::readExternalTemp2 (void) {
	return convertTemp(readRegister(MAX_TMPEXT2DAT));
}

bool MAX11300::setADCmode (ADCmode_t mode) {
	return readModifyWriteRegister(MAX_DEVCTL, MAX_ADCCTL_MASK, (uint16_t)(mode));
}

ADCmode_t MAX11300::getADCmode (void) {
	uint16_t reg = readRegister(MAX_DEVCTL);
	reg &= MAX_ADCCTL_MASK;
	if (reg == MAX_ADC_IDLE) return Idle;
	if (reg == MAX_ADC_SINGLE_SWEEP) return SingleSweep;
	if (reg == MAX_ADC_SINGLE_CONVERSION) return SingleSample;
	if (reg == MAX_ADC_CONTINUOUS) return ContinuousSweep;
	return ADCmodeNONE;
}

bool MAX11300::burstAnalogRead (uint16_t* samples, uint8_t size) {
	return burstAnalogRead(0, samples, size);
}

bool MAX11300::burstAnalogWrite (uint16_t* samples, uint8_t size) {
	return burstAnalogWrite(0, samples, size);
}

bool MAX11300::burstAnalogRead (uint8_t startPin, uint16_t* samples, uint8_t size) {
	if (size > 20) size = 20;
	return readRegister((MAX_ADCDAT_BASE + startPin), samples, size);
}

bool MAX11300::burstAnalogDiffRead (uint8_t startPin, int16_t* samples, uint8_t size) {
	if (size > 10) size = 10;
	return readRegisterSigned((MAX_ADCDAT_BASE + startPin), samples, size);
}


bool MAX11300::burstAnalogWrite (uint8_t startPin, uint16_t* samples, uint8_t size) {
	if (size > 20) size = 20;
	return writeRegister((MAX_DACDAT_BASE + startPin), samples, size);
}

bool MAX11300::isAnalogDataReady (uint8_t pin) {
	if (readRegister(MAX_INT) & MAX_ADCDR_MASK) {
		// read in the analog status registers from the chip and update the internal tracking
		_analogStatus |= (((uint32_t)(readRegister(MAX_ADCST_H))) << 16) |
							(readRegister(MAX_ADCST_L));
	} 
	if (_analogStatus & (1 << pin)) return true;
	return false;

}

bool MAX11300::isAnalogConversionComplete (void) {
	if (_analogFlag) {
		_analogFlag = false;
		return true;
	} 
	else if (readRegister(MAX_INT) & MAX_ADCFLAG_MASK) return true;
	return false;
}

void MAX11300::serviceInterrupt(void) {
	lastEvent.time = micros();							// set the time as soon as possible
	uint16_t lastIntVector = lastEvent.lastIntVector;	// copy the last interrupt vector over for comparison
	lastEvent.clearEvent();								// clears everything except the time
	lastEvent.lastIntVector = readRegister(MAX_INT);	// load the latest interrupt vector
	uint16_t delta = ((lastEvent.lastIntVector ^ lastIntVector) & 
						lastEvent.lastIntVector);
	if (delta & MAX_VMON_MASK) {
		lastEvent.event = VoltageMonitor;
	} else if (delta & MAX_TMPINT_MASK) {
		if (delta & (1 << MAX_TMPINT_AVAIL)) lastEvent.event = InternalTempAvailable;
		if (delta & (1 << MAX_TMPINT_HI)) lastEvent.event = InternalTempMonitorHigh;
		if (delta & (1 << MAX_TMPINT_LO)) lastEvent.event = InternalTempMonitorLow;
	} else if (delta & MAX_TMPEXT1_MASK) {
		if (delta & (1 << MAX_TMPEXT1_AVAIL)) lastEvent.event = ExternalTemp1Available;
		if (delta & (1 << MAX_TMPEXT1_HI)) lastEvent.event = ExternalTemp1MonitorHigh;
		if (delta & (1 << MAX_TMPEXT1_LO)) lastEvent.event = ExternalTemp1MonitorLow;
	} else if (delta & MAX_TMPEXT2_MASK) {
		if (delta & (1 << MAX_TMPEXT2_AVAIL)) lastEvent.event = ExternalTemp2Available;
		if (delta & (1 << MAX_TMPEXT2_HI)) lastEvent.event = ExternalTemp2MonitorHigh;
		if (delta & (1 << MAX_TMPEXT2_LO)) lastEvent.event = ExternalTemp2MonitorLow;
	} else if (delta & MAX_DACOI_MASK) {
		lastEvent.event = DACovercurrent;
		lastEvent.status = (((uint32_t)(readRegister(MAX_DACOI_H))) << 16) |
							(readRegister(MAX_DACOI_L));
	} else if (delta & MAX_GPIDM_MASK) {
		lastEvent.event = DigitalDataMissed;
		lastEvent.status = (((uint32_t)(readRegister(MAX_GPIST_H))) << 16) |
							(readRegister(MAX_GPIST_L));
	} else if (delta & MAX_GPIDR_MASK) {
		lastEvent.event = DigitalDataReady;
		lastEvent.status = (((uint32_t)(readRegister(MAX_GPIST_H))) << 16) |
							(readRegister(MAX_GPIST_L));
	} else if (delta & MAX_ADCDM_MASK) {
		lastEvent.event = AnalogDataMissed;
	} else if (delta & MAX_ADCDR_MASK) {
		lastEvent.event = AnalogDataReady;
		isAnalogDataReady(0);
		lastEvent.status = _analogStatus;
	} else if (delta & MAX_ADCFLAG_MASK) {
		lastEvent.event = AnalogConversionComplete;
		_analogFlag = true;
	} else {
		lastEvent.event = eventNONE;
	}
}

MAX11300Event MAX11300::getLastEvent (void) {
	return lastEvent;
}

bool MAX11300::writeRegister (uint8_t address, uint16_t value) {
	return writeRegister(address, &value, 1);
}

bool MAX11300::writeRegister (uint8_t address, uint16_t * values, uint8_t size) {
	_spi->beginTransaction(*_spiMode);
	digitalWrite(_select, LOW);
	_spi->transfer((address << 0x01) | 0x00);
	for (uint8_t i = 0; i < size; i++) {
		_spi->transfer((uint8_t)((values[i] >> 8) & 0xff));
		_spi->transfer((uint8_t)(values[i] & 0xff));
	}
	digitalWrite(_select, HIGH);
	_spi->endTransaction();
	return true;
}

uint16_t MAX11300::readRegister (uint8_t address) {
	uint16_t val = 0;
	readRegister(address, &val, 1);
	return val;
}

uint16_t MAX11300::readRegister (uint8_t address, uint16_t * values, uint8_t size) {
	_spi->beginTransaction(*_spiMode);
	digitalWrite(_select, LOW);
	_spi->transfer((address << 0x01) | 0x01);
	for (uint8_t i = 0; i < size; i++) {
		values[i] = ((uint16_t)(_spi->transfer(0)) << 8) + _spi->transfer(0);
	}
	digitalWrite(_select, HIGH);
	_spi->endTransaction();
	return size;
}

uint16_t MAX11300::readRegisterSigned (uint8_t address, int16_t * values, uint8_t size) {
	int8_t MSB, LSB;
	_spi->beginTransaction(*_spiMode);
	digitalWrite(_select, LOW);
	_spi->transfer((address << 0x01) | 0x01);
	for (uint8_t i = 0; i < size; i++) {
		MSB = _spi->transfer(0);
		LSB = _spi->transfer(0);
		values[i] = (((MSB << 12) | (LSB << 4))) >> 4;     	// assemble values at top of int
															// shift back to where they should be	
	}
	digitalWrite(_select, HIGH);
	_spi->endTransaction();
	return size;
}

void MAX11300::startConversion(void) {
	digitalWrite(_convertPin, LOW);
	delayMicroseconds(1);
	digitalWrite(_convertPin, HIGH);
}

bool MAX11300::readModifyWriteRegister(uint8_t address, uint16_t mask, uint16_t value) {
	uint16_t reg = readRegister(address);
	reg = (reg & ~mask) | (uint16_t)(value);
	return writeRegister(address, reg);
}

double MAX11300::convertTemp (uint16_t temp) {
	return ((int16_t)(temp << 4) * TEMP_LSB);
}

uint16_t MAX11300::convertTemp (double temp) {
	return (uint16_t)(((int16_t)(temp/TEMP_LSB)) >> 4);
}

MAX11300Event::MAX11300Event(void) {
	this->clearEvent();
}

void MAX11300Event::clearEvent(void) {
	lastIntVector = 0;
	event = eventNONE;
	status = 0;
}