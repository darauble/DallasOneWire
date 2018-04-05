/*
 * The header of the OneWire protocol driver for USARTs on STM32F10x family microcontrollers.
 * Currently supports USART1, USART2 and USART3 on STM32F103, UART4 and UART5 on STM32F107.
 *
 * ow_driver_stm32f10x_usart.h
 *
 *  Created on: Mar 24, 2018
 *      Author: Darau, blė
 *
 *  Credits:
 *	- Игорь (aka dadigor), steel_ne, Jonas Trečiokas (aka Jonis):
 *	  http://we.easyelectronics.ru/STM32/stm32-1-wire-dma-prodolzhenie.html
 *	  http://we.easyelectronics.ru/STM32/esche-raz-o-stm32-i-ds18b20-podpravleno.html
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

#ifndef OW_DRIVER_STM32_USART_H_
#define OW_DRIVER_STM32_USART_H_

/* It is generally not a good idea to use malloc in embedded systems. So the library allocates
 * predefined size of static memory to put driver structures in. On STM32F103 type MCUs
 * there are three USARTs available, hence the maximum count of OW_HEAP_SIZE. Adjust
 * to your own needs.
 */
#define OW_HEAP_SIZE 3

/* Includes partial DMA support, i.e. writing one byte (8 USART bytes) from buffer using
 * hardware flow. However, DMA is supported only for writing bytes to the bus. Useful, when
 * selecting particular device by its serial for reading the data, CPU can be freed at that
 * time.
 *
 * Note: better provide this option from your makefile using -DOW_MODE_USART_DMA
 */
//#define OW_MODE_USART_DMA

// Arguments to pass to init_driver to initialize appropriate USART as a master
enum {
	E_USART1 = 0
	,E_USART2
	,E_USART3
#if defined (STM32F10X_HD) || defined  (STM32F10X_CL) || defined  (STM32F10X_XL)
	,E_UART4
	,E_UART5
#endif
};


#endif /* OW_DRIVER_STM32_USART_H_ */
