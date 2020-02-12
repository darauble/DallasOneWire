/*
 * The header of the library to read Dallas temperature sensors. It uses
 * OneWire library, which provides core underlying protocol functionality.
 * Intended for use with SMT32 Cortex-M3 microcontrolers, but solely depends
 * only on OneWire drivers.
 *
 * dallas.h
 *
 *  Created on: Dec 2, 2016
 *      Author: Darau, blė
 *
 *  Credits:
 *  - Hagai Shatz
 *  - https://github.com/milesburton/Arduino-Temperature-Control-Library
 *
 *  This file is a part of personal use libraries developed specifically for STM32 Cortex-M3
 *  microcontrollers, most notably STM32F103 series, later extended to
 *  be used on any device with provided drivers
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#ifndef INCLUDE_DALLAS_H_
#define INCLUDE_DALLAS_H_

// Save some space by undefining older devices support
//#define DS_SUPPORT_18S20


// Sensor types
#ifdef DS_SUPPORT_18S20
#define TYPE_DS18S20  0x10 // 9-bit non-configurable sensor
#endif
#define TYPE_DS18B20  0x28 // Most popular OneWire thermometer with configurable resolution, ±0.5°C accuracy
#define TYPE_DS1822   0x22 // Almost the same as above, but only ±2.0°C accuracy
#define TYPE_DS1825   0x3B // The same family as MAX31850 thermocouple reader, with hardware addressing
#define TYPE_DS28EA00 0x42 // Model with sequence detection pins, allowing to order devices on a wire


// Converstion resolution
#define RES_9  0x1F //  9 bit
#define RES_10 0x3F // 10 bit
#define RES_11 0x5F // 11 bit
#define RES_12 0x7F // 12 bit

typedef enum {
	SCR_L = 0,
	SCR_H,
	SCR_HI_ALARM,
	SCR_LO_ALARM,
	SCR_CFG,
	SCR_FFH,
	SCR_RESERVED,
	SCR_10H,
	SCR_CRC,
	__SCR_LENGTH,
} scratch_pad_index;

#define CMD_READ_SCR 0xBE
#define CMD_START_CONV 0x44
#define CMD_WRITE_SCR 0x4E
#define CMD_COPY_SCR 0x48

#include <stdint.h>
#include "onewire.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * Issue a global command to start temperature conversion for
 * all connected sensors.
 *
 * @param: OneWire describing and initialized strucutre
 * @return: OneWire device presence on the bus
 */
int ds_convert_all(owu_struct_t *wire);

/**
 * Issue a command to start temperature conversion for
 * particular device, selected by its serial
 *
 * @param: OneWire describing and initialized strucutre
 * @param: 8 byte serial of the sensor
 * @return: OneWire device presence on the bus
 */
int ds_convert_device(owu_struct_t *wire, uint8_t *addr);

/**
 * Read a scratchpad of a particular sensor identified by its serial number
 *
 * @param: OneWire describing and initialized strucutre
 * @param: 8 byte serial of the sensor
 * @param: a pointer to put read scratchpad to
 * @return: OW_OK if read successful, error code otherwise
 */
int ds_read_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad);

/**
 * Read only two bytes of the scratchpad were temperature bytes are located.
 * Useful, when some CPU cycles are desired to be saved. With big amount of sensors
 * on one bus saves about 40% of time (even though it is negligible in general)
 *
 * @param: OneWire describing and initialized strucutre
 * @param: 8 byte serial of the sensor
 * @param: a pointer to put read scratchpad to
 * @return: OW_OK if read successful, error code otherwise
 */
 int ds_read_temp_only(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad);

/**
 * Gets raw 12 bit representation of the temperature bytes. Used
 * to calculate Celsius and Farenheit. It is public for those interested
 * in tinkering with integer arithmetics.
 *
 * @param: a pointer to read scratchpad
 * @return: 12 bit representation of raw temperature
 */
int16_t ds_get_raw(uint8_t *scratchpad);

/**
 * Convert scratchpad's value to Celsius
 *
 * @param: a pointer to read scratchpad
 * @return: floating pointer of temperature in Celsius
 */
float ds_get_temp_c(uint8_t *scratchpad);

/**
 * Convert scratchpad's value to that strange Farenheit...
 *
 * @param: a pointer to read scratchpad
 * @return: floating pointer of temperature in Farenheit
 */
float ds_get_temp_f(uint8_t *scratchpad);

/**
 * Convert scratchpad's value to integer Celsius.
 * If 1 ℃ accuracy is enough for you, save MCU cycles!
 *
 * @param: a pointer to read scratchpad
 * @return: quick'n'dirty integer temperature in Celsius
 */
int16_t ds_get_temp_c_int(uint8_t *scratchpad);

/**
 * Check if the device is compatible with temperature conversion. Useful
 * when OneWire bus contains mixed types of devices.
 *
 * @param: 8 byte serial of the sensor
 * @return: 1 if device converts temperature, 0 otherwise
 */
int ds_is_temp_device(uint8_t *addr);

/**
 * Retrieve device's resolution from the scratchpad.
 *
 */
int ds_get_resolution(uint8_t *scratchpad);

/**
 * Write the scratchpad. It is used to configure the device:
 * - set low alarm temperature (in Celsius, sorry Farenheit lovers)
 * - set high alarm temperature (in Celsius)
 * - set resolution (if supported)
 *
 * To configure device first read its scratchpad, then alter desired registers, e.g.:
 * >> scratchpad[SCR_HI_ALARM] = 75
 * >> scratchpad[SCR_LO_ALARM] = -15
 * >> scratchpad[SCR_CFG] = RES_12
 *
 * Then call this function.
 *
 * NOTES:
 * - you can only alter those registers, whichever you like. But all three
 *   must be written. That's why it is recommended to read scratchpad first.
 * - Wait at least 20 ms after this function call to start using sensors again.
 *   Delay is not included to avoid issues with multitasking RTOSes.
 *
 * @param: OneWire describing and initialized strucutre
 * @param: 8 byte serial of the sensor
 * @param: scratchpad witch configuration values
 */
int ds_write_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad);

#if defined(__cplusplus)
}
#endif

#endif /* INCLUDE_DALLAS_H_ */
