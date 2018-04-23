/*
 * Demo application using Dallas temperature and OneWire protocol libraries
 * on top of ESP-32 driver + IDF UART FreeRTOS driver
 *
 * main.c
 *
 *  Created on: Apr 10, 2018
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

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"

#include "ow_driver.h"
#include "ow_driver_esp32_usart.h"
#include "onewire.h"
#include "dallas.h"

uint8_t scratch_pad[__SCR_LENGTH];

uint8_t dev_count;
uint8_t dev_addr[10][8];

static void read_temperature()
{
	uint32_t i;
	ow_driver_ptr driver;
	owu_struct_t o2;

	printf("OneWire UART driver on ESP32\r\n");

	init_driver(&driver, E_UART1 | E_GPIO_RX_5 | E_GPIO_TX_4);
	printf("Driver on\r\n");
	owu_init(&o2, driver);
	printf("One wire initialized\r\n");

	dev_count = 0;
	while(owu_search(&o2, dev_addr[dev_count])) {
		printf("Device found %d\r\n", ++dev_count);
	}

	while (1) {
		printf("Convert all\r\n");

		ds_convert_all(&o2);

		vTaskDelay(1000 / portTICK_RATE_MS);

		uint8_t j;
		for (i=0; i<dev_count; i++) {
			ds_read_scratchpad(&o2, dev_addr[i], scratch_pad);

			printf("0x%02X%02X%02X%02X%02X%02X%02X%02X ",
					dev_addr[i][0], dev_addr[i][1], dev_addr[i][2], dev_addr[i][3],
					dev_addr[i][4], dev_addr[i][5], dev_addr[i][6], dev_addr[i][7]);

			for (j=0; j<__SCR_LENGTH; j++) {
				printf(" %02X", scratch_pad[j]);
			}
			printf("  %.5f\r\n", ds_get_temp_c(scratch_pad));
		}

		vTaskDelay(5000 / portTICK_RATE_MS);

	}
}


void app_main()
{
	xTaskCreate(read_temperature, "temp_task", 4096, NULL, 10, NULL);
}

