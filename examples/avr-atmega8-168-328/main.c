/*
 * Demo application using Dallas temperature and OneWire protocol libraries
 * on top of AVR GPIO driver.
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
#define UART_BAUD_RATE 115200
#define BUF_LEN 100

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "ow_driver.h"
#include "ow_driver_avr_gpio.h"
#include "onewire.h"
#include "dallas.h"
#include "uart.h"

char buf[BUF_LEN];

void delay_us(uint32_t);
void beautifyFloatStr(char *);

uint8_t scratch_pad[__SCR_LENGTH];
uint8_t dev_count;
uint8_t dev_addr[MAX_OW_DEV_COUNT][8];

int main()
{
	ow_driver_ptr driver;
	owu_struct_t o2;

	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU));
	sei();

	uart_puts_P("OneWire AVR driver!\r\n");

	init_driver(&driver, E_PORTD+3); // AVR PD3, Arduino Digital Pin 3
	uart_puts_P("Driver on\r\n");
	owu_init(&o2, driver);
	uart_puts_P("One wire initialized\r\n");

	dev_count = 0;
	while(owu_search(&o2, dev_addr[dev_count])) {
		snprintf(buf, BUF_LEN, "Device found %d\r\n", ++dev_count);
		uart_puts(buf);
		if (dev_count >= MAX_OW_DEV_COUNT) {
			break;
		}
	}

	uint32_t i;

	while(1) {
		uart_puts_P("Convert all\r\n");
		ds_convert_all(&o2);
		i = 5000000;
		while (--i) __asm__ __volatile__ ("nop");// In AVR simple "while (--i);" won't work, need "nop" to be executed

		uint8_t j;
		for (i=0; i<dev_count; i++) {
			ds_read_scratchpad(&o2, dev_addr[i], scratch_pad);

			snprintf(buf, BUF_LEN, "0x%02X%02X%02X%02X%02X%02X%02X%02X ",
				dev_addr[i][0], dev_addr[i][1], dev_addr[i][2], dev_addr[i][3],
				dev_addr[i][4], dev_addr[i][5], dev_addr[i][6], dev_addr[i][7]);
			uart_puts(buf);

			for (j=0; j<__SCR_LENGTH; j++) {
				snprintf(buf, BUF_LEN, " %02X", scratch_pad[j]);
				uart_puts(buf);
			}
			snprintf(buf, BUF_LEN, "  %.4f", (double) ds_get_temp_c(scratch_pad));
			beautifyFloatStr(buf);
			uart_puts(buf);
			uart_puts_P("\r\n");
		}

		i = 10000000;
		while (--i) __asm__ __volatile__ ("nop");
	}
	return 1;
}

// Scavenged somewhere from AVR or Arduino libraries
void delay_us(uint32_t us)
{
	if (--us == 0) return;
	if (--us == 0) return;
	us <<= 1; us--;

	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t"
		"brne 1b" : "=w" (us) : "0" (us)
	);
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
