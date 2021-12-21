
/*!
 *  @file Adafruit_VCNL4040.cpp
 *
 *  @mainpage Adafruit VCNL4040 proximity and ambient light sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the VCNL4040 proximity and ambient light sensor library
 *
 * 	This is a library for the Adafruit VCNL4040 breakout:
 * 	https://www.adafruit.com/product/4161
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"
#include <Wire.h>

#include "Adafruit_VCNL4040.h"

/*!
 *    @brief  Instantiates a new VCNL4040 class
 */
Adafruit_VCNL4040::Adafruit_VCNL4040(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
boolean Adafruit_VCNL4040::begin(uint8_t i2c_address, I2C_HandleTypeDef *i2c_handle) {
  i2c_han = i2c_handle;
  i2c_addr = i2c_address << 1;

//  if (!i2c_dev->begin()) {
//    return false;
//  }

  return _init();
}

boolean Adafruit_VCNL4040::_init(void) {
  uint16_t chip_id;
  readRegister((uint16_t) VCNL4040_DEVICE_ID,(uint8_t*) &chip_id, 2);
  // make sure we're talking to the right chip
  if (chip_id != 0x0186) {
    return false;
  }

//  ALS_CONFIG = new Adafruit_BusIO_Register(i2c_dev, VCNL4040_ALS_CONFIG, 2);
//  PS_CONFIG_12 = new Adafruit_BusIO_Register(i2c_dev, VCNL4040_PS_CONF1_L, 2);
//  PS_MS = new Adafruit_BusIO_Register(i2c_dev, VCNL4040_PS_MS_H, 2);

  enableProximity(true);
  enableWhiteLight(true);
  enableAmbientLight(true);
  setProximityHighResolution(true);

  return true;
}
/**************** Sensor Data Getters *************************************/
/**************************************************************************/
/*!
    @brief Gets the current proximity sensor value.
    @return The current proximity measurement in units
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getProximity(void) {
  delay(10);
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_PS_DATA,(uint8_t*) &temp, 2);
  return temp;
}

/**************************************************************************/
/*!
    @brief Gets the current ambient light sensor value.
    @return The current ambient light measurement in units
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getAmbientLight(void) {
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_ALS_DATA,(uint8_t*) &temp, 2);
  return temp;
}
/**************************************************************************/
/*!
    @brief Gets the current white light value.
    @return The current white light measurement in units
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getWhiteLight(void) {

  // scale the light depending on the value of the integration time
  // see page 8 of the VCNL4040 application note:
  // https://www.vishay.com/docs/84307/designingvcnl4040.pdf
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_WHITE_DATA,(uint8_t*) &temp, 2);
  return (temp * (0.1 / (1 << getAmbientIntegrationTime())));
}

/**************************************************************************/
/*!
    @brief Gets the current ambient light sensor in Lux.
    @return The current ambient light measurement in Lux
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getLux(void) {
  // scale the lux depending on the value of the integration time
  // see page 8 of the VCNL4040 application note:
  // https://www.vishay.com/docs/84307/designingvcnl4040.pdf
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_ALS_DATA,(uint8_t*) &temp, 2);
  return (temp * (0.1 / (1 << getAmbientIntegrationTime())));
}
/**************** Sensor Enable Functions   *******************************/

/**************************************************************************/
/*!
    @brief Enables or disables proximity measurements.
    @param  enable
            Set to true to enable proximity measurements,
            set to false to disable.
*/
void Adafruit_VCNL4040::enableProximity(bool enable) {

  modifyRegisterMultipleBit16(VCNL4040_PS_CONF1_L, !enable, 0, 1);

}
/**************************************************************************/
/*!
    @brief Enables ambient light measurements
    @param  enable
            Set to true to enable ambient light measurements,
            set to false to disable.
*/
void Adafruit_VCNL4040::enableAmbientLight(bool enable) {

  modifyRegisterMultipleBit16(VCNL4040_ALS_CONFIG, !enable, 0, 1);

}
/**************************************************************************/
/*!
    @brief Enables white light measurements
    @param  enable
            Set to true to enable white light measurements,
            set to false to disable.
*/
void Adafruit_VCNL4040::enableWhiteLight(bool enable) {

  modifyRegisterMultipleBit16(VCNL4040_ALS_CONFIG, !enable, 15, 1);
}

/*************************** Interrupt Functions  *********************** */

/**************************************************************************/
/*!
    @brief Gets and clears the interrupt status register.
    @return The current value of the interrupt status register.
            Indivitual interrupt types can be checked by anding the returned
   byte with the members of `VCNL4040_InterruptType`:`VCNL4040_PROXIMITY_AWAY`,
            `VCNL4040_PROXIMITY_CLOSE`, `PROXIMITY_LOW`, or `PROXIMITY_HIGH`
*/
/**************************************************************************/
uint8_t Adafruit_VCNL4040::getInterruptStatus(void) {

  return (uint16_t) readRegisterMultipleBit16(VCNL4040_INT_FLAG, 8, 8);
}

/********************* Ambient Light Interrupt Functions **************** */

/**************************************************************************/
/*!
    @brief Enables or disables ambient light based interrupts.
    @param  enable
            Set to true to enable or false to disable ambient light interrupts
*/
/**************************************************************************/
void Adafruit_VCNL4040::enableAmbientLightInterrupts(bool enable) {

  modifyRegisterMultipleBit16(VCNL4040_ALS_CONFIG, enable, 1, 1);

}

/**************************************************************************/
/*!
    @brief Gets the current ambient light high threshold.
    @return The current ambient light high threshold
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getAmbientLightHighThreshold(void) {
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_ALS_THDH,(uint8_t*) &temp, 2);
  return temp;
}
/**************************************************************************/
/*!
    @brief Sets the ambient light high threshold.
    @param  high_threshold
            The high threshold to set
*/
/**************************************************************************/
void Adafruit_VCNL4040::setAmbientLightHighThreshold(uint16_t high_threshold) {

  writeRegister(VCNL4040_ALS_THDH, (uint8_t*) &high_threshold, 2);
}
/**************************************************************************/
/*!
    @brief Gets the ambient light low threshold.
    @return the current ambient light low threshold
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getAmbientLightLowThreshold(void) {
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_ALS_THDL,(uint8_t*) &temp, 2);
  return temp;
}

/**************************************************************************/
/*!
    @brief Sets the ambient light low threshold.
    @param  low_threshold
            The low threshold to set
*/
/**************************************************************************/
void Adafruit_VCNL4040::setAmbientLightLowThreshold(uint16_t low_threshold) {
  writeRegister(VCNL4040_ALS_THDL, (uint8_t*) &low_threshold, 2);

}

/********************* Proximity Interrupt Functions **************** */

/**************************************************************************/
/*!
    @brief Disables or  enables proximity interrupts under a given condition.
    @param  interrupt_condition
            The condition under which to raise an interrupt. Must be a
   `VCNL4040_ProximityType`. Use `VCNL4040_PROXIMITY_INT_DISABLE` to disable
   proximity interrupts.
*/
/**************************************************************************/
void Adafruit_VCNL4040::enableProximityInterrupts(
    VCNL4040_ProximityType interrupt_condition) {

  modifyRegisterMultipleBit16(VCNL4040_PS_CONF1_L, interrupt_condition, 8, 2);

}

/**************************************************************************/
/*!
    @brief Gets the proximity low threshold.
    @returns  The current low threshold
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getProximityLowThreshold(void) {
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_PS_THDL,(uint8_t*) &temp, 2);
  return temp;
}
/**************************************************************************/
/*!
    @brief Sets the proximity low threshold.
    @param  low_threshold
            The low threshold to set
*/
/**************************************************************************/
void Adafruit_VCNL4040::setProximityLowThreshold(uint16_t low_threshold) {
  writeRegister(VCNL4040_PS_THDL, (uint8_t*) &low_threshold, 2);
}
/**************************************************************************/
/*!
    @brief Gets the proximity high threshold.
    @returns  The current high threshold
*/
/**************************************************************************/
uint16_t Adafruit_VCNL4040::getProximityHighThreshold(void) {
  uint16_t temp;
  readRegister((uint16_t) VCNL4040_PS_THDH,(uint8_t*) &temp, 2);
  return temp;
}
/**************************************************************************/
/*!
    @brief Sets the proximity high threshold.
    @param  high_threshold
            The high threshold to set
*/
/**************************************************************************/
void Adafruit_VCNL4040::setProximityHighThreshold(uint16_t high_threshold) {
  writeRegister(VCNL4040_PS_THDH, (uint8_t*) &high_threshold, 2);
}

/******************** Tuning Functions ********************************** */

/**************************************************************************/
/*!
    @brief Gets the integration time for proximity sensing measurements.
    @returns The integration time being used for proximity measurements.
*/
VCNL4040_ProximityIntegration
Adafruit_VCNL4040::getProximityIntegrationTime(void) {
  delay(50);
  return (VCNL4040_ProximityIntegration) readRegisterMultipleBit16(VCNL4040_PS_CONF1_L, 1, 3);
}
/**************************************************************************/
/*!
    @brief Sets the integration time for proximity sensing measurements.
    @param  integration_time
            The integration time to use for proximity measurements. Must be a
            `VCNL4040_ProximityIntegration`.
*/
void Adafruit_VCNL4040::setProximityIntegrationTime(
    VCNL4040_ProximityIntegration integration_time) {
  delay(50);
  modifyRegisterMultipleBit16(VCNL4040_PS_CONF1_L, integration_time, 1, 3);
}

/**************************************************************************/
/*!
    @brief Gets the integration time for ambient light sensing measurements.
    @returns The integration time being used for ambient light measurements.
*/
VCNL4040_AmbientIntegration Adafruit_VCNL4040::getAmbientIntegrationTime(void) {
  delay(50);
  return (VCNL4040_AmbientIntegration) readRegisterMultipleBit16(VCNL4040_ALS_CONFIG, 6, 2);
}

/**************************************************************************/
/*!
    @brief Sets the integration time for ambient light sensing measurements.
    @param  integration_time
            The integration time to use for ambient light measurements. Must be
   a `VCNL4040_AmbientIntegration`.
*/
void Adafruit_VCNL4040::setAmbientIntegrationTime(
    VCNL4040_AmbientIntegration integration_time) {

  // delay according to the integration time to let the reading at the old IT
  // clear out
  uint8_t old_it_raw = readRegisterMultipleBit16(VCNL4040_ALS_CONFIG, 6, 2);
  uint16_t old_it_ms = ((8 << old_it_raw) * 10);
  uint16_t new_it_ms = ((8 << integration_time) * 10);

  modifyRegisterMultipleBit16(VCNL4040_ALS_CONFIG, integration_time, 6, 2);
  delay((old_it_ms + new_it_ms + 1));
}

/**************************************************************************/
/*!
    @brief Gets the current for the LED used for proximity measurements.
    @returns The LED current value being used for proximity measurements.
*/
VCNL4040_LEDCurrent Adafruit_VCNL4040::getProximityLEDCurrent(void) {
  return (VCNL4040_LEDCurrent) readRegisterMultipleBit16(VCNL4040_PS_MS_H, 8, 2);
} /**************************************************************************/
/*!
    @brief Sets the current for the LED used for proximity measurements.
    @param  led_current
            The current value to be used for proximity measurements. Must be a
            `VCNL4040_LEDCurrent`.
*/
void Adafruit_VCNL4040::setProximityLEDCurrent(
    VCNL4040_LEDCurrent led_current) {

  modifyRegisterMultipleBit16(VCNL4040_PS_MS_H, led_current, 8, 2);

}

/**************************************************************************/
/*!
    @brief Sets the duty cycle for the LED used for proximity measurements.
    @returns The duty cycle value being used for proximity measurements.
*/
VCNL4040_LEDDutyCycle Adafruit_VCNL4040::getProximityLEDDutyCycle(void) {
  return (VCNL4040_LEDDutyCycle)readRegisterMultipleBit16(VCNL4040_PS_CONF1_L, 6, 2);
}
/**************************************************************************/
/*!
    @brief Sets the duty cycle for the LED used for proximity measurements.
    @param  duty_cycle
            The duty cycle value to be used for proximity measurements. Must be
   a `VCNL4040_LEDDutyCycle`.
*/
void Adafruit_VCNL4040::setProximityLEDDutyCycle(
    VCNL4040_LEDDutyCycle duty_cycle) {

  modifyRegisterMultipleBit16(VCNL4040_PS_CONF1_L, duty_cycle, 6, 2);

}

/**************************************************************************/
/*!
    @brief Gets the resolution of proximity measurements
    @return The current proximity measurement resolution
            If true, proximity measurements are 16-bit,
            If false, proximity measurements are 12-bit,
*/
bool Adafruit_VCNL4040::getProximityHighResolution(void) {
  Adafruit_BusIO_RegisterBits ps_hd =
      Adafruit_BusIO_RegisterBits(PS_CONFIG_12, 1, 11);
  return (bool)readRegisterMultipleBit16(VCNL4040_PS_CONF1_L, 11, 1);
}
/**************************************************************************/
/*!
    @brief Sets the resolution of proximity measurements
    @param  high_resolution
            Set to true to take 16-bit measurements for proximity,
            set to faluse to use 12-bit measurements.
*/
void Adafruit_VCNL4040::setProximityHighResolution(bool high_resolution) {

  modifyRegisterMultipleBit16(VCNL4040_PS_CONF1_L, high_resolution, 11, 1);
}


bool Adafruit_VCNL4040::readRegister(uint16_t mem_addr, uint8_t *dest,
		uint16_t size) {
	if (HAL_OK
			== HAL_I2C_Mem_Read(i2c_han, i2c_addr, mem_addr, 1, dest, size, 10)) {
		return true;
	} else {
		return false;
	}
}

bool Adafruit_VCNL4040::writeRegister(uint8_t mem_addr, uint8_t *val,
		uint16_t size) {
	if (HAL_OK
			== HAL_I2C_Mem_Write(i2c_han, i2c_addr, mem_addr, 1, val, size, 10)) {
		return true;
	} else {
		return false;
	}
}

//bool Adafruit_AS7341::modifyRegisterBit16(uint16_t reg, bool value, uint16_t pos) {
//	uint8_t register_value = readRegisterByte(reg);
//	register_value = modifyBitInByte(register_value, (uint8_t) value, pos);
//
//	return writeRegisterByte(reg, register_value);
//}

bool Adafruit_AS7341::modifyRegisterMultipleBit16(uint16_t reg, uint16_t value,
						  uint16_t pos, uint8_t bits) {
	uint16_t register_value;
	readRegister(reg, (uint8_t *)&register_value,2);

	uint16_t mask = (1 << (bits)) - 1;
	value &= mask;

	mask <<= pos;
	register_value &= ~mask;          // remove the current data at that spot
	register_value |= value << pos; // and add in the new data

	return writeRegister(reg, (uint8_t *) &register_value, 2);
}

uint16_t Adafruit_AS7341::readRegisterMultipleBit16(uint16_t reg,
						  uint16_t pos, uint8_t bits) {
	uint16_t register_value;
	readRegister(reg, (uint8_t *)&register_value,2);

	uint16_t mask = (1 << (bits)) - 1;
	value &= mask;

	mask <<= pos;
	register_value &= ~mask;          // remove the current data at that spot

	return register_value
}


//uint8_t Adafruit_AS7341::checkRegisterBit(uint16_t reg, uint8_t pos) {
//	return (uint8_t) ((readRegisterByte(reg) >> pos) & 0x01);
//}
