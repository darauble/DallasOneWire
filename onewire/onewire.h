/*
 * The source of the OneWire protocol library agnostic of underlying "driver"
 * implementation.
 *
 * onewire.h
 *
 *  Created on: Nov 22, 2016
 *      Author: Darau, blė
 *
 *  Credits: Jim Studt, Paul Stoffregen et al
 *  https://github.com/PaulStoffregen/OneWire
 *
 *  This file is a part of personal use libraries developed to be used
 *  on various microcontrollers and Linux devices.
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

#ifndef INCLUDE_ONEWIRE_H_
#define INCLUDE_ONEWIRE_H_

#include "ow_driver.h"

// Undefine if OneWire address search is not required, save several bytes
#define OW_LIB_SEARCH

// The structure describing OneWire instance. Don't use it directly, only
// provide required variables and pointer to this structure to appropriate
// owu_init(...) function (constructor).
typedef struct owu_struct {
	ow_driver_ptr driver;
#ifdef OW_LIB_SEARCH
	uint8_t last_discrepancy;
	uint8_t last_device_flag;
	uint8_t last_family_discrepancy;
	uint8_t rom_no[8];
#endif
} owu_struct_t;

#if defined(__cplusplus)
extern "C" {
#endif

/**
 * OneWire structure initializer
 * @param already initialized One Wire driver to be used
 * @param pointer to owu_struct_t to be initialized
 */
void owu_init(owu_struct_t *, ow_driver_ptr);

/**
 * Resets the OneWire bus with reset pulse.
 * @param owu_struct_t type initialized instance
 * @return OW_OK if there are connected OneWire devices, OW_NOTHING if bus is empty
 */
int owu_reset(owu_struct_t *);

/**
 * Writes a byte to OneWire bus using defined mode.
 * @param owu_struct_t type initialized instance
 * @param a byte to be written to the bus
 * @return OW_OK if operation successful, error code otherwise
 */
int owu_write_byte(owu_struct_t *, uint8_t);

/**
 * Reads a byte from OneWire bus using defined mode.
 * @param owu_struct_t type initialized instance
 * @param a pointer to place read byte
 * @return an error/success code
 */
int owu_read_byte(owu_struct_t *, uint8_t*);

/**
 * Send skip ROM address check command
 * @param owu_struct_t type initialized instance
 * @return OW_OK if operation successful, error code otherwise
 */
int owu_skip(owu_struct_t *);

/**
 * Selects particular device by address for further
 * commands.
 * @param owu_struct_t type initialized instance
 * @param address of the device
 * @return OW_OK if operation successful, error code otherwise
 */
int owu_select_device(owu_struct_t *, const uint8_t *);

/**
 * Calculate CRC8 for a given bytes of OneWire data
 * @param a pointer to byte array to calculate CRC8 on
 * @param length of data referenced by the pointer
 */
uint32_t owu_crc8(uint8_t*, uint32_t);

#ifdef OW_LIB_SEARCH
/**
 * Resets the search, when one is required to be restarted
 * from the beginning (i.e. full search again).
 * @param owu_struct_t type initialized instance
 */
void owu_reset_search(owu_struct_t *wire);

/**
 * OneWire device search iterator, copied from Arduino OneWire library
 * by Jim Studt, Paul Stoffregen and others, downloaded from
 * http://www.pjrc.com/teensy/td_libs_OneWire.html
 *
 * @param owu_struct_t type initialized instance
 * @param pointer to 8 byte array to place found address
 */
int owu_search(owu_struct_t *wire, uint8_t *addr);
#endif

#if defined(__cplusplus)
}
#endif


#endif /* INCLUDE_ONEWIRE_H_ */
