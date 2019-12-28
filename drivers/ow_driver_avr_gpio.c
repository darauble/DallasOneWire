/*
 * The source of the OneWire protocol driver for bitbanging PORTs on AVR family microcontrollers.
 * Currently supports PORTB, PORTC and PORTD on ATMega168/328 family.
 *
 * ow_driver_avr_gpio.c
 *
 *  Created on: Mar 31, 2018
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "ow_driver.h"
#include "ow_driver_avr_gpio.h"


struct one_wire_driver {
	volatile uint8_t* port;
	uint8_t pin;
};

struct one_wire_driver one_wire_heap[OW_HEAP_SIZE];
static int heap_ptr = 0;

int init_driver(ow_driver_ptr *driver, int pin)
{
	if (heap_ptr > OW_HEAP_SIZE) {
		return OW_NOMEMORY;
	}
	*driver = &one_wire_heap[heap_ptr++];
	ow_driver_ptr d = *driver;
	//uint8_t pin;


	if (pin >= E_PORTB && pin < E_PORTC) {
		d->port = &PORTB;
		d->pin = _BV(pin - E_PORTB);
	} else if (pin >= E_PORTC && pin < E_PORTD) {
		d->port = &PORTC;
		d->pin = _BV(pin - E_PORTC);
	} else if (pin >= E_PORTD && (pin - E_PORTD) < 8) {
		d->port = &PORTD;
		d->pin = _BV(pin - E_PORTD);
	} else {
		heap_ptr--; // Reclaim heap in case of error
		return OW_ERR;
	}

	return OW_OK;
}
int reset_wire(ow_driver_ptr d)
{
	uint8_t ow_presence;
	uint8_t retries = 125;

	DDR(*d->port) &= ~d->pin; // Input mode

	// Backup interrupt register value and disable interrupts
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // Wait while pin is high, just in case
        do {
            if (--retries == 0) {
                return OW_ERR;
            }
            delay_us(2);
        } while ((PIN(*d->port) & d->pin) == 0);

        DDR(*d->port) |= d->pin;; // Output mode
        *d->port &= ~d->pin; // Output low

        delay_us(480);

        DDR(*d->port) &= ~d->pin; // Input mode

        delay_us(70);

        if ((PIN(*d->port) & d->pin) == 0) {
            ow_presence = OW_OK;
        } else {
            ow_presence = OW_NOTHING;
        }

        // Restore interrupt state to previous one
    }

	delay_us(410);

	OW_YIELD;

	return ow_presence;
}

int write_bit(ow_driver_ptr d, uint8_t bit)
{
	uint32_t d1, d2;

	if (bit & 1) {
		d1 = 10; d2 = 55;
	} else {
		d1 = 65; d2 = 5;
	}

	DDR(*d->port) |= d->pin;; // Output mode

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        *d->port &= ~d->pin; // Output low
        delay_us(d1);
        *d->port |= d->pin; // Output high
        delay_us(d2);

        DDR(*d->port) &= ~d->pin; // Input mode, leave floating on exit
    }

	OW_YIELD;

	return OW_OK;
}

int read_bit(ow_driver_ptr d, uint8_t *rbit)
{
	DDR(*d->port) |= d->pin;; // Output mode

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        *d->port &= ~d->pin; // Output low

        delay_us(3);

        *d->port |= d->pin; // Output high
        DDR(*d->port) &= ~d->pin; // Input mode

        delay_us(10);

        if ((PIN(*d->port) & d->pin) > 0) {
            *rbit = 1;
        } else {
            *rbit = 0;
        }
    }

	delay_us(53);

	return OW_OK;
}

int write_byte(ow_driver_ptr d, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		write_bit(d, byte & 0x01);
		byte >>= 1;
		OW_YIELD;
	}
	return OW_OK;
}

int read_byte(ow_driver_ptr d, uint8_t *rbyte)
{
	int c;
	uint8_t byte = 0, bit, i;
	for (i=0; i<8; i++){
		byte >>= 1;
		c = read_bit(d, &bit);
		if (c != OW_OK) {
			return c;
		}
		if (bit) {
			byte |= 0x80;
		}
	}
	*rbyte = byte;

	OW_YIELD;

	return OW_OK;
}

