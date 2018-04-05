/*
 * The source of the OneWire protocol "driver" for Linux supported serial
 * TTY devices (like FT232, CH304 etc.)
 *
 * ow_driver_linux_usart.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Darau, blė
 *
 *  Credits: Chi Zhang aka dword1511
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
#include <ow_driver_linux_usart.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#define RESET_BYTE (uint8_t)0xF0

#define SEND_0	0x00
#define SEND_1	0xff
#define OW_R_1	0xff

const uint8_t ow_bits[] = { SEND_0, SEND_1 };

const char *TTY = "/dev/tty%s%d";

struct one_wire_driver {
	int fd;
};

static void set_baud(ow_driver_ptr, int);

int init_driver(ow_driver_ptr *driver, int dev_id)
{
	*driver = (ow_driver_ptr)malloc(sizeof(struct one_wire_driver));

	char dev_path[PATH_LEN];

	if (dev_id < OW_TTY_ACM) {
		snprintf(dev_path, PATH_LEN, TTY, "USB", dev_id);
	} else if (dev_id >= OW_TTY_ACM && dev_id < OW_TTY_S) {
		snprintf(dev_path, PATH_LEN, TTY, "ACM", dev_id-OW_TTY_ACM);
	} else if (dev_id >= OW_TTY_S && dev_id < OW_TTY_AMA) {
		snprintf(dev_path, PATH_LEN, TTY, "S", dev_id-OW_TTY_S);
	} else if (dev_id >= OW_TTY_AMA) {
		snprintf(dev_path, PATH_LEN, TTY, "AMA", dev_id-OW_TTY_AMA);
	}

	(*driver)->fd = open(dev_path, O_RDWR | O_NOCTTY | O_NDELAY);

	if ((*driver)->fd < 0) {
		// Error...
		printf("Could not open device handler: %d\n", (*driver)->fd);
		free(*driver);
		return OW_ERR;
	}

	set_baud(*driver, B115200);

	return OW_OK;
}

int release_driver(ow_driver_ptr *driver)
{
	close((*driver)->fd);
	free(*driver);

	return OW_OK;
}

int reset_wire(ow_driver_ptr d)
{
	int ret;
	unsigned char c = RESET_BYTE;

	set_baud(d, B9600);

	ret = write(d->fd, &c, 1);
	if (ret != 1) {
		return OW_ERR;
	}
	while(!read(d->fd, &c, 1));

	set_baud(d, B115200);

	if (c != RESET_BYTE) {
		return OW_OK;
	}

	return OW_NOTHING;
}

int write_bit(ow_driver_ptr d, uint8_t bit)
{
	int ret;
	unsigned char c;
	c = ow_bits[bit];

	ret = write(d->fd, &c, 1);
	if (ret != 1) {
		return OW_ERR;
	}
	while(!read(d->fd, &c, 1)); // Need to read and flush on write, as RX/TX are interconnected

	return OW_OK;
}

int read_bit(ow_driver_ptr d, uint8_t *rbit)
{
	int ret;
	unsigned char c = OW_R_1;

	ret = write(d->fd, &c, 1);
	if (ret != 1) {
		return OW_ERR;
	}
	while(!read(d->fd, &c, 1));

	if (c == OW_R_1) {
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

static void set_baud(ow_driver_ptr d, int baud)
{
	struct termios tio = {
		.c_cflag = baud | CS8 | CLOCAL | CREAD,
		.c_iflag = 0,
		.c_oflag = 0,
		.c_lflag = NOFLSH,
		.c_cc = {0},
	};

	tcflush(d->fd, TCIFLUSH);
	tcsetattr(d->fd, TCSANOW, &tio);
}
