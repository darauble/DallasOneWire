/*
 * Demo application using Dallas temperature and OneWire protocol libraries
 * on top of STM32F10x MCU GPIO driver.
 *
 * main.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Darau, blė
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

#define MAX_OW_DEV_COUNT 10

#include "stm32f10x.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <ow_driver.h>
#include <onewire.h>
#include <ow_driver_stm32f10x_gpio.h>
#include <dallas.h>

#include "usart_utils.h"
#include "makrosai.h"

char buf[100];

uint8_t scratch_pad[__SCR_LENGTH];
uint8_t dev_count;
uint8_t dev_addr[MAX_OW_DEV_COUNT][8];

void beautifyFloatStr(char *);

int main(void)
{
	REMAP_JTAG_FULL_DISABLE;
	uint32_t i;

	ow_driver_ptr driver;
	owu_struct_t o2;

	init_delay_us();

	enable_usart(USART1, 115200);
	print_usart(USART1, "OneWire GPIO Driver!\r\n");

	// Init OneWire GPIO driver on GPIOA and pin 5 (PA5)
	init_driver(&driver, E_GPIOA+5);
	print_usart(USART1, "Driver on\r\n");
	owu_init(&o2, driver);
	print_usart(USART1, "One wire initialized\r\n");

	dev_count = 0;
	while(owu_search(&o2, dev_addr[dev_count])) {
		snprintf(buf, 99, "Device found %d\r\n", ++dev_count);
		print_usart(USART1, buf);
		if (dev_count >= MAX_OW_DEV_COUNT) {
			break;
		}
	}

	while (1) {
		print_usart(USART1, "Convert all\r\n");

		ds_convert_all(&o2);

		i = 50000000;
		while (--i);

		uint8_t j;
		for (i=0; i<dev_count; i++) {
			ds_read_scratchpad(&o2, dev_addr[i], scratch_pad);

			snprintf(buf, 99, "0x%02X%02X%02X%02X%02X%02X%02X%02X ",
				dev_addr[i][0], dev_addr[i][1], dev_addr[i][2], dev_addr[i][3],
				dev_addr[i][4], dev_addr[i][5], dev_addr[i][6], dev_addr[i][7]);
			print_usart(USART1, buf);

			for (j=0; j<__SCR_LENGTH; j++) {
				snprintf(buf, 99, " %02X", scratch_pad[j]);
				print_usart(USART1, buf);
			}
			snprintf(buf, 99, "  %.5f", ds_get_temp_c(scratch_pad));
			beautifyFloatStr(buf);
			print_usart(USART1, buf);
			print_usart(USART1, "\r\n");
		}

		i = 10000000;
		while (--i);
    }
}

void beautifyFloatStr(char *str)
{
	uint16_t l = strlen(str);
	int i;
	for (i = l-1; i >= 0; i--) {
		if (str[i] == '0') {
			str[i] = 0;
		} else if (str[i] == '.') {
			str[i] = 0;
			break;
		} else {
			break;
		}
	}
}
