/*
 * The source of the OneWire protocol driver for UARTs on ESP32 microcontrollers.
 *
 * ow_driver_esp32_usart.c
 *
 *  Created on: Apr 10, 2018
 *      Author: Darau, blė
 *
 *	Credits:
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

#include "ow_driver.h"
#include "ow_driver_esp32_usart.h"

//#include "freertos/task.h"
#include "driver/uart.h"

#define UB_9600	9600
#define UB_115200 115200

#define UCLKDIV_9600 (uint32_t) 0x0050208D
#define UCLKDIV_115200 (uint32_t) 0x007002B6

#define UART_MASK (uint32_t)0x3F000
#define TX_MASK (uint32_t)0xFC0
#define RX_MASK (uint32_t)0x3F
#define ECHO_TEST_RTS  (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS  (UART_PIN_NO_CHANGE)

#define RESET_BYTE (uint8_t)0xF0

#define SEND_0	0x00
#define SEND_1	0xff
#define OW_R_1	0xff

const uint8_t ow_bits[] = { SEND_0, SEND_1 };

struct one_wire_driver {
	uart_port_t uart;
};

//struct one_wire_driver one_wire_heap[OW_HEAP_SIZE];
//static int heap_ptr = 0;

int init_driver(ow_driver_ptr *d, int u)
{
	uart_port_t uart = (u & UART_MASK) >> 12;
	if (uart >= UART_NUM_MAX) {
		return OW_ERR;
	}

	*d = (ow_driver_ptr)malloc(sizeof(struct one_wire_driver));
	if (*d == NULL) {
		return OW_NOMEMORY;
	}

	(*d)->uart = uart;
	int tx, rx;
	tx = (u & TX_MASK) >> 6;
	rx = (u & RX_MASK);

	 uart_config_t uart_config = {
		.baud_rate = UB_115200,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};

	if (uart_param_config((*d)->uart, &uart_config) != ESP_OK) { return OW_ERR; }
	if (uart_set_pin((*d)->uart, tx, rx, ECHO_TEST_RTS, ECHO_TEST_CTS) != ESP_OK) { return OW_ERR; }
	if (uart_driver_install((*d)->uart, UART_FIFO_LEN*2, 0, 0, NULL, 0) != ESP_OK) { return OW_ERR; }

	return OW_OK;
}

int release_driver(ow_driver_ptr *d)
{
	if (uart_driver_delete((*d)->uart) != ESP_OK) {
		return OW_ERR;
	}
	free(*d);

	return OW_OK;
}

int reset_wire(ow_driver_ptr d)
{
	uint8_t ow_presence;

	uart_flush(d->uart);
	// Set 9600 baud for 480 µs reset pulse
	WRITE_PERI_REG(UART_CLKDIV_REG(d->uart), UCLKDIV_9600);
	char rst = RESET_BYTE;
	uart_write_bytes(d->uart, (const char *) &rst, 1);

	uart_read_bytes(d->uart, &ow_presence, 1, 20 / portTICK_RATE_MS);

	// Restore "normal" operating speed
	WRITE_PERI_REG(UART_CLKDIV_REG(d->uart), UCLKDIV_115200);

	if (ow_presence != RESET_BYTE) {
		return OW_OK;
	}

	return OW_NOTHING;
}

int write_bit(ow_driver_ptr d, uint8_t bit)
{
	uart_flush(d->uart);
	uart_write_bytes(d->uart, (const char *) &ow_bits[bit], 1);

	// Wait until all 8 bits sent, i.e. TX_DAT7 reached (at least) and RX_STP1 (received bits filled in)
	uint32_t s = READ_PERI_REG(UART_STATUS_REG(d->uart));
	while ( ((s & 0x0F000000) < 0x09000000) && ((s & 0x0F00) < 0x0B00) )
	{
		s = READ_PERI_REG(UART_STATUS_REG(d->uart));
		ets_delay_us(5);
	}

	return OW_OK;
}

int read_bit(ow_driver_ptr d, uint8_t *rbit)
{
	uint8_t bit;
	char owone = OW_R_1;

	uart_flush(d->uart);
	uart_write_bytes(d->uart, (const char *) &owone, 1);

	while ((READ_PERI_REG(UART_STATUS_REG(d->uart)) & 0x0F000000) < 0x09000000);

	uart_read_bytes(d->uart, &bit, 1, 20 / portTICK_RATE_MS);

	if (bit == OW_R_1) {
		*rbit = 1;
	} else {
		*rbit = 0;
	}

	return OW_OK;
}

int write_byte(ow_driver_ptr d, uint8_t byte)
{
	uint8_t i;
	for (i=0; i<8; i++) {
		write_bit(d, byte & 0x01);
		byte >>= 1;
	}
	return OW_OK;
}

int read_byte(ow_driver_ptr d, uint8_t *rbyte)
{
	int c;
	uint8_t byte = 0, bit, i;
	for (i=0; i<8; i++) {
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

	return OW_OK;
}
