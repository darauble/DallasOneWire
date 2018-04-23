/*
 * The header of the OneWire protocol driver for UARTs on ESP32 microcontrollers.
 *
 * ow_driver_esp32_usart.h
 *
 *  Created on: Apr 10, 2018
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

#ifndef OW_DRIVER_ESP32_USART_H_
#define OW_DRIVER_ESP32_USART_H_

// On provided ESP32 FreeRTOS UART driver there's flaw that prevents to use TX/RX
// completed flags. So after transmitting any OneWire signal (except for reset pulse)
// we need to add short delay, otherwise signalling gets corrupted by underlying
// software ring buffer
#define OW_DELAY_US 70


// Arguments to pass to init_driver to initialize appropriate USART as a master.
// E.g. init_driver(d, E_UART1 | E_GPIO_RX_5 | E_GPIO_TX_4)
enum {
	E_UART0 = 0,
	E_UART1 = 0x1000,
	E_UART2 = 0x2000,
};

enum {
	E_GPIO_RX_0 = 0,
	E_GPIO_RX_1 = 1,
	E_GPIO_RX_2 = 2,
	E_GPIO_RX_3 = 3,
	E_GPIO_RX_4 = 4,
	E_GPIO_RX_5 = 5,
	E_GPIO_RX_6 = 6,
	E_GPIO_RX_7 = 7,
	E_GPIO_RX_8 = 8,
	E_GPIO_RX_9 = 9,
	E_GPIO_RX_10 = 10,
	E_GPIO_RX_11 = 11,
	E_GPIO_RX_12 = 12,
	E_GPIO_RX_13 = 13,
	E_GPIO_RX_14 = 14,
	E_GPIO_RX_15 = 15,
	E_GPIO_RX_16 = 16,
	E_GPIO_RX_17 = 17,
	E_GPIO_RX_18 = 18,
	E_GPIO_RX_19 = 19,
	E_GPIO_RX_20 = 20,
	E_GPIO_RX_21 = 21,
	E_GPIO_RX_22 = 22,
	E_GPIO_RX_23 = 23,
	E_GPIO_RX_24 = 24,
	E_GPIO_RX_25 = 25,
	E_GPIO_RX_26 = 26,
	E_GPIO_RX_27 = 27, // Gap as per ESP32's gpio.h
	E_GPIO_RX_32 = 32,
	E_GPIO_RX_33 = 33,
	E_GPIO_RX_34 = 34, // This and remaining for input only, ok for RX
	E_GPIO_RX_35 = 35,
	E_GPIO_RX_36 = 36,
	E_GPIO_RX_37 = 37,
	E_GPIO_RX_38 = 38,
	E_GPIO_RX_39 = 39,
};

enum {
	E_GPIO_TX_0 = 0,
	E_GPIO_TX_1 = 0x0040,
	E_GPIO_TX_2 = 0x0080,
	E_GPIO_TX_3 = 0x00C0,
	E_GPIO_TX_4 = 0x0100,
	E_GPIO_TX_5 = 0x0140,
	E_GPIO_TX_6 = 0x0180,
	E_GPIO_TX_7 = 0x01C0,
	E_GPIO_TX_8 = 0x0200,
	E_GPIO_TX_9 = 0x0240,
	E_GPIO_TX_10 = 0x0280,
	E_GPIO_TX_11 = 0x02C0,
	E_GPIO_TX_12 = 0x0300,
	E_GPIO_TX_13 = 0x0340,
	E_GPIO_TX_14 = 0x0380,
	E_GPIO_TX_15 = 0x03C0,
	E_GPIO_TX_16 = 0x0400,
	E_GPIO_TX_17 = 0x0440,
	E_GPIO_TX_18 = 0x0480,
	E_GPIO_TX_19 = 0x04C0,
	E_GPIO_TX_20 = 0x0500,
	E_GPIO_TX_21 = 0x0540,
	E_GPIO_TX_22 = 0x0580,
	E_GPIO_TX_23 = 0x05C0,
	E_GPIO_TX_24 = 0x0600,
	E_GPIO_TX_25 = 0x0640,
	E_GPIO_TX_26 = 0x0680,
	E_GPIO_TX_27 = 0x06C0,
	E_GPIO_TX_32 = 0x0800,
	E_GPIO_TX_33 = 0x0840,
};


#endif /* OW_DRIVER_ESP32_USART_H_ */
