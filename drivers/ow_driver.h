/*
 * The header of the OneWire protocol driver with abstract details.
 *
 * This file provides basic API to talk with OneWire slaves. Actual
 * commands to be executed by appropriate and more abstract software.
 *
 * ow_driver.h
 *
 *  Created on: Mar 22, 2018
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

#ifndef OW_DRIVER_H_
#define OW_DRIVER_H_

#include <stdint.h>

// An abstract pointer to particular driver's implementation
typedef struct one_wire_driver * ow_driver_ptr;

/* Define a method to yield for a particular system. It can be "yield"
 * for Arduino based ESP8266 or "taskYIELD" for FreeRTOS system.
 *
 * Yield is called in case of waiting operations, e.g. byte written to the bus
 * or waiting for byte to be available for reading.
 */
#ifndef OW_YIELD
#define OW_YIELD
#endif

typedef enum {
	OW_OK = 0,
	OW_NONE,
	OW_ERR,
	OW_NOTHING,
	OW_NOMEMORY
} ow_ret_code_t;

#if defined(__cplusplus)
extern "C" {
#endif
/**
 * Initializes the One Wire driver.
 *
 * @param a pointer to the driver's structure
 * @param initialization parameter, different for each particular driver.
 *        It might be a pin, GPIO or USART number (starting from 0).
 *
 * @return: 0 if ok, other number in case of error.
 */
int init_driver(ow_driver_ptr*, int);

/**
 * Release resources acquired by the driver. E.g. shutdown hardware
 * clocks, release file descriptor etc.
 *
 * @param a pointer to the driver's structure
 *
 * @return: 0 if ok, other number in case of error
 */
int release_driver(ow_driver_ptr*);

/**
 * Issue a reset pulse to One Wire. Implementation is up to the driver,
 * as pulse length might be controlled by particular hardware.
 *
 * @param a pointer to the initialized driver's structure
 * @return 0 if ok, other number in case of error.
 */
int reset_wire(ow_driver_ptr);

/**
 * Write a bit to One Wire bus.
 *
 * @param a pointer to the initialized driver's structure
 * @param bit to write (1 or 0)
 * @return 0 if ok, other number in case of error.
 */
int write_bit(ow_driver_ptr, uint8_t);


/**
 * Read a bit from One Wire bus
 *
 * @param a pointer to the initialized driver's structure
 * @param a pointer to place read bit (1 or 0)
 * @return 0 if ok, other number in case of error.
 */
int read_bit(ow_driver_ptr, uint8_t*);

/**
 * Write a byte to One Wire bus. Usually it is 8 "write_bit",
 * but in other cases it might be implemented more efficiently.
 *
 * @param a pointer to the initialized driver's structure
 * @param byte to write (1 or 0)
 * @return 0 if ok, other number in case of error.
 */
int write_byte(ow_driver_ptr, uint8_t);

/**
 * Read a byte from One Wire bus. Usually it is 8 "read_bit",
 * but might be implemented more efficiently.
 *
 * @param a pointer to the initialized driver's structure
 * @param a pointer to place read byte (1 or 0)
 * @return 0 if ok, other number in case of error.
 */
int read_byte(ow_driver_ptr, uint8_t*);

#if defined(__cplusplus)
}
#endif

#endif /* OW_DRIVER_H_ */
