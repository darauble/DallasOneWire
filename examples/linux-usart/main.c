/*
 * Demo application using Dallas temperature and OneWire protocol libraries
 * on top of Linux USART "driver".
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>

#include <ow_driver.h>
#include <onewire.h>
#include <dallas.h>

uint8_t scratch_pad[__SCR_LENGTH];
uint8_t dev_count;
uint8_t dev_addr[MAX_OW_DEV_COUNT][8];

ow_driver_ptr driver;
owu_struct_t o2;
int i, s, loop;

static void handle_signal(int);
static void setup_signals(void);

int main(void) {
	loop = 0;

	setup_signals();

	// Init on /dev/ttyUSB0 (zero, right?)
	s = init_driver(&driver, 0);
	if (s == OW_OK) {
		printf("Init completed\n");
	} else {
		printf("Failed to init driver: %d", s);
		return EXIT_FAILURE;
	}
	owu_init(&o2, driver);

	dev_count = 0;
	while(owu_search(&o2, dev_addr[dev_count])) {
		printf("Device found %d\n", ++dev_count);
		if (dev_count >= MAX_OW_DEV_COUNT) {
			break;
		}
	}

	loop = 1;

	while (loop)
	{
		printf("Convert all\n");
		ds_convert_all(&o2);
		sleep(1);

		uint8_t j;
		for (i=0; i<dev_count; i++) {
			ds_read_scratchpad(&o2, dev_addr[i], scratch_pad);

			printf("0x%02X%02X%02X%02X%02X%02X%02X%02X ",
				dev_addr[i][0], dev_addr[i][1], dev_addr[i][2], dev_addr[i][3],
				dev_addr[i][4], dev_addr[i][5], dev_addr[i][6], dev_addr[i][7]);

			for (j=0; j<9; j++) {
				printf(" %02X", scratch_pad[j]);
			}
			printf("  %.4f\n", ds_get_temp_c(scratch_pad));
		}

		if (loop) { sleep(3);}
	}

	release_driver(&driver);
	return EXIT_SUCCESS;
}

static void handle_signal(int n)
{
	printf("\nSignal %d\n", n);
	if (loop) {
		loop = 0;
	} else {
		exit(EXIT_FAILURE);
	}
}

static void setup_signals(void)
{
	struct sigaction s;

	memset(&s, 0, sizeof(s));
	s.sa_handler = handle_signal;
	s.sa_flags = 0;
	sigaction(SIGINT, &s, NULL);
	sigaction(SIGTERM, &s, NULL);
	sigaction(SIGUSR1, &s, NULL);
	sigaction(SIGUSR2, &s, NULL);

	s.sa_handler = SIG_IGN;
	sigaction(SIGPIPE, &s, NULL);
}
