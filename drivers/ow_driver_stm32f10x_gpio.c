/*
 * The source of the OneWire protocol driver for bitbanging GPIOS on STM32F10x family microcontrollers.
 * Currently supports GPIOA and GPIOB on STM32F103
 *
 * ow_driver_stm32f10x_gpio.c
 *
 *  Created on: Mar 23, 2018
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

#include <ow_driver.h>
#include <ow_driver_stm32f10x_gpio.h>
#include "stm32f10x.h"

struct one_wire_driver {
	GPIO_TypeDef *gpio;
	volatile uint32_t *cr;
	uint32_t pin;
	uint32_t clear_mask;
	uint32_t in_mask;
	uint32_t out_mask;
};

struct one_wire_driver one_wire_heap[OW_HEAP_SIZE];
static int heap_ptr = 0;

int init_driver(ow_driver_ptr *d, int gpio)
{
	if (heap_ptr > OW_HEAP_SIZE) {
		return OW_NOMEMORY;
	}
	*d = &one_wire_heap[heap_ptr++];
	uint32_t pin;

	if (gpio >= E_GPIOA && gpio < E_GPIOB) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN;
		(*d)->gpio = GPIOA;
		pin = gpio;
		(*d)->pin = 0x01 << pin;
	} else if (gpio >= E_GPIOB && gpio < E_GPIOC) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPBEN;
		(*d)->gpio = GPIOB;
		pin = (gpio - E_GPIOB);
		(*d)->pin = 0x01 << pin;
	} else if (gpio >= E_GPIOC && gpio < E_GPIOD) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPCEN;
		(*d)->gpio = GPIOC;
		pin = (gpio - E_GPIOC);
		(*d)->pin = 0x01 << pin;
	} else if (gpio >= E_GPIOD && (gpio - E_GPIOD) < 16) {
		RCC->APB2ENR |=  RCC_APB2ENR_IOPDEN;
		(*d)->gpio = GPIOD;
		pin = (gpio - E_GPIOD);
		(*d)->pin = 0x01 << pin;
	} else {
		heap_ptr--; // Reclaim heap in case of error
		return OW_ERR;
	}

	if (pin < 8) {
		(*d)->cr = &(*d)->gpio->CRL;
		(*d)->clear_mask = (GPIO_CRL_CNF0 | GPIO_CRL_MODE0) << (4 * pin);
		(*d)->in_mask = (GPIO_CRL_CNF0_0) << (4 * pin);
		(*d)->out_mask = (GPIO_CRL_CNF0_0 | GPIO_CRL_MODE0) << (4 * pin);
	} else {
		(*d)->cr = &(*d)->gpio->CRH;
		(*d)->clear_mask = (GPIO_CRH_CNF8 | GPIO_CRH_MODE8) << (4 * (pin-8));
		(*d)->in_mask = (GPIO_CRH_CNF8_0) << (4 * (pin-8));
		(*d)->out_mask = (GPIO_CRH_CNF8_0 | GPIO_CRH_MODE8) << (4 * (pin-8));
	}

	return OW_OK;
}
int reset_wire(ow_driver_ptr d)
{
	uint8_t ow_presence;
	uint8_t retries = 125;

	*d->cr &= ~d->clear_mask;
	*d->cr |= d->in_mask; // Input mode

	// Wait while pin is high, just in case
	do {
		if (--retries == 0) {
			return OW_ERR;
		}
		delay_us(2);
	} while ((d->gpio->IDR & d->pin) == 0);

	*d->cr &= ~d->clear_mask;
	*d->cr |= d->out_mask; // Output mode
	d->gpio->BRR |= d->pin; // Output low

	delay_us(480);

	///d->gpio->BSRR |= d->pin;

	*d->cr &= ~d->clear_mask;
	*d->cr |= d->in_mask; // Input mode

	delay_us(70);

	if ((d->gpio->IDR & d->pin) == 0) {
		ow_presence = OW_OK;
	} else {
		ow_presence = OW_NOTHING;
	}
	delay_us(410);

	*d->cr &= ~d->clear_mask; // Clear on leave

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

	*d->cr &= ~d->clear_mask;
	*d->cr |= d->out_mask; // Output mode

	d->gpio->BRR |= d->pin; // Output low
	delay_us(d1);
	d->gpio->BSRR |= d->pin; // Output high
	delay_us(d2);

	*d->cr &= ~d->clear_mask;

	OW_YIELD;

	return OW_OK;
}

int read_bit(ow_driver_ptr d, uint8_t *rbit)
{
	*d->cr &= ~d->clear_mask;
	*d->cr |= d->out_mask; // Output mode
	d->gpio->BRR |= d->pin; // Output low

	delay_us(3);

	d->gpio->BSRR |= d->pin; // Output high
	*d->cr &= ~d->clear_mask;
	*d->cr |= d->in_mask; // Input mode

	delay_us(10);

	if ((d->gpio->IDR & d->pin) > 0) {
		*rbit = 1;
	} else {
		*rbit = 0;
	}

	*d->cr &= ~d->clear_mask;
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

