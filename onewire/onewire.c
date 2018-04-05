/*
 * The source of the OneWire protocol library agnostic of underlying "driver"
 * implementation.
 *
 * onewire.c
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

#include <onewire.h>
#include <string.h>

#define CMD_SKIP   (uint8_t) 0xCC
#define CMD_SEL    (uint8_t) 0x55
#define CMD_SEARCH (uint8_t) 0xF0

void owu_init(owu_struct_t *wire, ow_driver_ptr driver)
{
	wire->driver = driver;
#ifdef OW_LIB_SEARCH
	// Initialize search variables
	owu_reset_search(wire);
#endif
}

int owu_reset(owu_struct_t *wire)
{
	return reset_wire(wire->driver);
}

int owu_write_byte(owu_struct_t *wire, uint8_t byte)
{
	return write_byte(wire->driver, byte);
}

int owu_read_byte(owu_struct_t *wire, uint8_t *byte)
{
	return read_byte(wire->driver, byte);
}

void owu_skip(owu_struct_t *wire)
{
	write_byte(wire->driver, CMD_SKIP);
}

void owu_select_device(owu_struct_t *wire, const uint8_t *addr)
{
	write_byte(wire->driver, CMD_SEL);
	uint8_t i;
	for (i=0; i<8; i++) {
		write_byte(wire->driver, *addr++);
	}
}

uint32_t owu_crc8(uint8_t* data, uint32_t len)
{
	uint32_t crc = 0;

	while (len--) {
		uint8_t inbyte = *data++;
		uint8_t i;
		for (i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

#ifdef OW_LIB_SEARCH
void owu_reset_search(owu_struct_t *wire)
{
	wire->last_discrepancy = 0;
	wire->last_device_flag = 0;
	wire->last_family_discrepancy = 0;
	memset(&wire->rom_no[0], 0, 8);
}

int owu_search(owu_struct_t *wire, uint8_t *addr)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number, search_result;
	uint8_t id_bit, cmp_id_bit;
	uint8_t rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = 0;

	// if the last call was not the last one
		if (!wire->last_device_flag)
		{
			// 1-Wire reset
			if (reset_wire(wire->driver) == OW_NOTHING)
			{
				// reset the search
				wire->last_discrepancy = 0;
				wire->last_device_flag = 0;
				wire->last_family_discrepancy = 0;
				//print_usart(USART1, "Search::no devices\r\n");
				return 0;
			}

			// issue the search command
			write_byte(wire->driver, CMD_SEARCH);

			// loop to do the search
			do
			{
				// read a bit and its complement
				read_bit(wire->driver, &id_bit);
				read_bit(wire->driver, &cmp_id_bit);

				// check for no devices on 1-wire
				if ((id_bit == 1) && (cmp_id_bit == 1)) {
					break;
				}
				else
				{
					// all devices coupled have 0 or 1
					if (id_bit != cmp_id_bit) {
						search_direction = id_bit;  // bit write value for search
					}
					else
					{
						// if this discrepancy if before the Last Discrepancy
						// on a previous next then pick the same as last time
						if (id_bit_number < wire->last_discrepancy) {
							search_direction = ((wire->rom_no[rom_byte_number] & rom_byte_mask) > 0);
						}
						else {
							// if equal to last pick 1, if not then pick 0
							search_direction = (id_bit_number == wire->last_discrepancy);
						}
						// if 0 was picked then record its position in LastZero
						if (search_direction == 0)
						{
							last_zero = id_bit_number;

							// check for Last discrepancy in family
							if (last_zero < 9) {
								wire->last_family_discrepancy = last_zero;
							}
						}
					}

					// set or clear the bit in the ROM byte rom_byte_number
					// with mask rom_byte_mask
					if (search_direction == 1) {
					  wire->rom_no[rom_byte_number] |= rom_byte_mask;
					}
					else {
					  wire->rom_no[rom_byte_number] &= ~rom_byte_mask;
					}

					// serial number search direction write bit
					write_bit(wire->driver, search_direction);

					// increment the byte counter id_bit_number
					// and shift the mask rom_byte_mask
					id_bit_number++;
					rom_byte_mask <<= 1;

					// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
					if (rom_byte_mask == 0)
					{
						 rom_byte_number++;
						 rom_byte_mask = 1;
					}
				}
			}
			while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

			// if the search was successful then
			if (!(id_bit_number < 65))
			{
				// search successful so set wire->last_discrepancy,wire->last_device_flag,search_result
				wire->last_discrepancy = last_zero;

				// check for last device
				if (wire->last_discrepancy == 0)
					wire->last_device_flag = 1;

				search_result = 1;
			}
		}

		// if no device found then reset counters so next 'search' will be like a first
		if (!search_result || !wire->rom_no[0])
		{
			wire->last_discrepancy = 0;
			wire->last_device_flag = 0;
			wire->last_family_discrepancy = 0;
			search_result = 0;
			//print_usart(USART1, "Search::result reset\r\n");
		} else {
			/*for (int i = 0; i < 8; i++) {
				addr[i] = wire->rom_no[i];
			}*/
			memcpy(addr, wire->rom_no, 8);
		}

		return search_result;
}
#endif /* OW_LIB_SEARCH */
