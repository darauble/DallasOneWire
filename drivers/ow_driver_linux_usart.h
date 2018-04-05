/*
 * The header of the OneWire protocol "driver" for Linux supported serial
 * TTY devices (like FT232, CH304 etc.)
 *
 * ow_driver_linux_usart.h
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

#ifndef OW_DRIVER_LINUX_USART_H_
#define OW_DRIVER_LINUX_USART_H_

/* Serial devices on linux are usually enumerated as /dev/ttyUSB0, /dev/ttyUSB1.
 * /dev/ttyACM0 are usually used for modems, but happens with USART chips too.
 * To init driver, provide a number of TTY device like that:
 * init_driver(driver, OW_TTY_USB+0) or init_driver(driver, OW_TTY_ACM+0)
 */
enum {
	OW_TTY_USB = 0,
	OW_TTY_ACM = 0x80,
	OW_TTY_S = 0x100,
	OW_TTY_AMA = 0x180,
};

#define PATH_LEN 16 // Should suffice for most of the paths


#endif /* OW_DRIVER_LINUX_USART_H_ */
