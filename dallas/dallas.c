/*
 * The implementation of the library to read Dallas temperature sensors.
 *
 * dallas.c
 *
 *  Created on: Dec 2, 2016
 *      Author: Darau, blė
 *
 *  Credits:
 *  - Hagai Shatz
 *  - https://github.com/milesburton/Arduino-Temperature-Control-Library
 *
 *  This file is a part of personal use libraries developed specifically for STM32 Cortex-M3
 *  microcontrollers, most notably STM32F103 series.
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

#include "dallas.h"

int ds_convert_all(owu_struct_t *wire)
{
	int presence;
	presence = owu_reset(wire);
	owu_skip(wire);
	owu_write_byte(wire, CMD_START_CONV);

	return presence;
}

int ds_convert_device(owu_struct_t *wire, uint8_t *addr)
{
	int wire_status;

	wire_status = owu_reset(wire);
	if (wire_status != OW_OK) return wire_status;

	wire_status = owu_select_device(wire, addr);
	if (wire_status != OW_OK) return wire_status;

	wire_status = owu_write_byte(wire, CMD_START_CONV);
	if (wire_status != OW_OK) return wire_status;

	return wire_status;
}

static int ds_read(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad, int len)
{
	int wire_status;

	wire_status = owu_reset(wire);
	if (wire_status != OW_OK) return wire_status;
	
	wire_status = owu_select_device(wire, addr);
	if (wire_status != OW_OK) return wire_status;

	wire_status = owu_write_byte(wire, CMD_READ_SCR);
	if (wire_status != OW_OK) return wire_status;

	uint8_t i;

	for (i=0; i<len; i++) {
		wire_status = owu_read_byte(wire, &scratchpad[i]);
		if (wire_status != OW_OK) {
			return wire_status;
		}
	}

	return OW_OK;
}

int ds_read_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad)
{
	return ds_read(wire, addr, scratchpad, __SCR_LENGTH);
}

int ds_read_temp_only(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad)
{
	return ds_read(wire, addr, scratchpad, 2);
}

int16_t ds_get_raw(uint8_t *scratchpad)
{
	int16_t raw;
	raw = ((((int16_t) scratchpad[SCR_H]) << 11)
		| (((int16_t) scratchpad[SCR_L]) << 3));

#ifdef DS_SUPPORT_18S20
	if (scratchpad[SCR_CFG] == 0xFF) {
		// Device is either DS1820 or DS18S20,
		// recalculate: http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
		raw = ((raw & 0xfff0) << 3) - 16
				+ (((scratchpad[SCR_10H] - scratchpad[SCR_RESERVED]) << 7)
						/ scratchpad[SCR_10H]);
	}
#endif

	return raw;
}

float ds_get_temp_c(uint8_t *scratchpad)
{
	return 0.0078125*(float) ds_get_raw(scratchpad);
}

float ds_get_temp_f(uint8_t *scratchpad)
{
	return (0.0140625*(float) ds_get_raw(scratchpad)) + 32;
}

int16_t ds_get_temp_c_int(uint8_t *scratchpad)
{
	return (scratchpad[SCR_H] << 4) | (scratchpad[SCR_L] >> 4);
}

int ds_is_temp_device(uint8_t *addr)
{
	switch (addr[0]) {
#ifdef DS_SUPPORT_18S20
		case TYPE_DS18S20:
#endif
		case TYPE_DS18B20:
		case TYPE_DS1822:
		case TYPE_DS1825:
		case TYPE_DS28EA00:
			return 1;
		default:
			return 0;
	}
}

int ds_get_resolution(uint8_t *scratchpad)
{
	switch (scratchpad[SCR_CFG]) {
#ifdef DS_SUPPORT_18S20
		// Reserved byte in DS1820 and DS18S20, which are 9-bit devices, but provide data to calculate 12 bit values
		case 0xFF:
#endif
		case RES_12:
			return 12;
		case RES_11:
			return 11;
		case RES_10:
			return 10;
		case RES_9:
			return 9;
	}

	return 0;
}

int ds_write_scratchpad(owu_struct_t *wire, uint8_t *addr, uint8_t *scratchpad)
{
	int presence;
	presence = owu_reset(wire);
	if (presence != OW_OK) {
		return presence;
	}

	owu_select_device(wire, addr);
	owu_write_byte(wire, CMD_WRITE_SCR);
	owu_write_byte(wire, scratchpad[SCR_HI_ALARM]);
	owu_write_byte(wire, scratchpad[SCR_LO_ALARM]);

#ifdef DS_SUPPORT_18S20
	if (addr[0] != TYPE_DS18S20) {
		owu_write_byte(wire, scratchpad[SCR_CFG]);
	}
#endif

	presence = owu_reset(wire);
	if (presence != OW_OK) {
		return presence;
	}
	owu_select_device(wire, addr);
	owu_write_byte(wire, CMD_COPY_SCR);

	return OW_OK;
}
