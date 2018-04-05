/*
 * The header of the OneWire protocol driver for bitbanging PORTs on AVR family microcontrollers.
 * Currently supports PORTB, PORTC and PORTD on ATMega168/328 family.
 *
 * ow_driver_avr_gpio.h
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

#ifndef OW_DRIVER_AVR_GPIO_H_
#define OW_DRIVER_AVR_GPIO_H_

/* It is generally not a good idea to use malloc in embedded systems. So the library allocates
 * predefined size of static memory to put driver structures in. Decide how many GPIO pins
 * you'll be using as One Wire masters. Usually 1 is sufficient for small amount of sensors.
 */
#define OW_HEAP_SIZE 2


/** Arguments to pass to init_driver and provide PORT base and a pin. Example of use:
 *  init_driver(d, E_PORTB+5);
 */
typedef enum {
	E_PORTA = 0x00,
	E_PORTB = 0x08,
	E_PORTC = 0x10,
	E_PORTD = 0x18
} avr_port_t;

typedef enum {
	E_ARDUINO_PIN_0  = 0x18,
	E_ARDUINO_PIN_1  = 0x19,
	E_ARDUINO_PIN_2  = 0x1A,
	E_ARDUINO_PIN_3  = 0x1B,
	E_ARDUINO_PIN_4  = 0x1C,
	E_ARDUINO_PIN_5  = 0x1D,
	E_ARDUINO_PIN_6  = 0x1E,
	E_ARDUINO_PIN_7  = 0x1F,
	E_ARDUINO_PIN_8  = 0x08,
	E_ARDUINO_PIN_9  = 0x09,
	E_ARDUINO_PIN_10 = 0x0A,
	E_ARDUINO_PIN_11 = 0x0B,
	E_ARDUINO_PIN_12 = 0x0C,
	E_ARDUINO_PIN_13 = 0x0D,
	E_ARDUINO_PIN_A0 = 0x10,
	E_ARDUINO_PIN_A1 = 0x11,
	E_ARDUINO_PIN_A2 = 0x12,
	E_ARDUINO_PIN_A3 = 0x13,
	E_ARDUINO_PIN_A4 = 0x14,
	E_ARDUINO_PIN_A5 = 0x15,
} arduino_pin_map_t;

/**
 * External function providing microseconds delay
 */
extern void delay_us(uint32_t);

/**
 * Macroses to get port configuration and pin by actual port
 */
#define DDR(x) (*(&x - 1))      /* address of data direction register of port x */
#if defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__)
    /* on ATmega64/128 PINF is on port 0x00 and not 0x60 */
    #define PIN(x) ( &PORTF==&(x) ? _SFR_IO8(0x00) : (*(&x - 2)) )
#else
	#define PIN(x) (*(&x - 2))    /* address of input register of port x          */
#endif

#endif /* OW_DRIVER_AVR_GPIO_H_ */
