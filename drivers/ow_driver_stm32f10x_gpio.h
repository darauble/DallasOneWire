/*
 * The header of the OneWire protocol driver for bitbanging GPIOS on STM32F10x family microcontrollers.
 * Currently supports GPIOA, GPIOB, GPIOC and GPIOD on STM32F10x
 *
 * ow_driver_stm32f10x_gpio.h
 *
 *  Created on: Mar 24, 2018
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

#ifndef OW_DRIVER_STM32_GPIO_H_
#define OW_DRIVER_STM32_GPIO_H_

/* It is generally not a good idea to use malloc in embedded systems. So the library allocates
 * predefined size of static memory to put driver structures in. Decide how many GPIO pins
 * you'll be using as One Wire masters. Usually 1 is sufficient for small amount of sensors.
 */
#define OW_HEAP_SIZE 2


/** Arguments to pass to init_driver and provide GPIOx base and a pin. Example of use:
 *  init_driver(d, E_GPIOA+5);
 */
enum {
	E_GPIOA = 0,
	E_GPIOB = 0x10,
	E_GPIOC = 0x20,
	E_GPIOD = 0x30
};

/**
 * External function providing microseconds delay
 */
extern void delay_us(uint32_t);

#endif /* OW_DRIVER_STM32_GPIO_H_ */
