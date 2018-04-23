# Dallas Temperature and Maxim OneWire Library

Layered agnostic implementation of OneWire library for use on any possible device with appropriate driver.

Targeted devices:
- STM32F10x: GPIO and U(S)ART
- STM32F7xx (planned)
- AVR: GPIO
- Linux UART
- ESP32: UART

## Why?

There's that nice library by Jim Studt and Paul Stoffregen. Arduino framework nowadays could be used with
virtually any device.

But. The libraries written by others usually differ from our own point of view and solving the problems.
Above mentioned library is really good and I've used it on multiple occassions, when I wanted simple,
quick'n'dirty solution. However, the main caveat is solely single implementation independent of any
underlying hardware: bitbanging.

That's why I've started to implement my own libraries shamlessly copying open source written by others and
providing more flexible solution, where actual physical layer can be virtually anything, including USART
devices, SPI masters etc. Just add appropriate driver and it will go.

Also I did not like Dallas Temperature library because of too much hidden code lines and implementation. It
is, I admit, fool proof in most cases, but adds up piles of code.

And also I am believer that C++ is not the best language for embedded devices. But let's not be religious.

## Whom My Library Is Meant For?

I dedicate my library implementation for real programmers who love C, not just amateur DIYers. Please,
don't take this notation personally: I am DIYer myself, but also I am professional embedded engineer,
so my solutions usually are different, more structured and layered.

Ranting done. Can I do better? Maybe. Can I do it different? Sure! Can I do it in C? You bet.

Here's my work for your judgement.

# Idea

The idea behind implementation is to split code into layers:
- Application layer: any "business logic" that final application provides
- Functional layer: in this case Dallas Temperature library
- Peripheral layer: in this case OneWire bus library, agnostic of actual hardware
- Abstract driver API: as much generic as possible. Or fully generic
- Actual driver implementation: bitbang GPIO, USART, SPI master, you name it

## Dallas Temperature

Library provides:
- Issue command for all devices on the bus to convert temperature
- Issue same command for particular device
- Read particular device's scratchpad
- Read only two first bytes (i.e. temperature) of a particular device (save MCU cycles)
- Get raw 12 bit representation of the temperature
- Calculate Celsius from scratchpad
- Calculate Farenheit from scratchapad (why in heavens I did that...)
- Calculate only integer part of Celsius, when that's acceptable (very fast operation)
- Check if device is temperature sensor by its serial number
- Convert sensor's resolution to human readable format (9-12)
- Write configuration: low and high alarm temperature, resolution

### Notes

Library does not provide check for sensor's existence on bus, malfunction etc. It becomes obvious from
scratchpad's values (`0xFF`), so programmers can handle that by themselves.

There is no internal/hidden check if devices is temperature sensor. That is not needed in most cases,
as we, DIYers and engineers, plan what devices we'll connect. However, on busy buses with different
devices that check can be required, so function is provided. It is purely application's layer, IMHO.

Temperature sensors accept three bytes for configuration (DS18S20 only two): low temperature alarm,
high temperature alarm and resolution. Even though library could hide it by providing `setResolution`,
it would internally send three bytes anyway. So, again, I leave it to programmer. Read scratchpad of
particular device, change configuration bytes, issue command to write that configuration down.

Delay after sending configuration is not included. *Don't forget* to give sensor at least 20 ms to save
configuration internally!

**Parasite mode** is not supported and will not be by me. However, contributions are welcome if desired.
I am simply not interested in it, I always wire temperature sensors with UTP cable anyway, there
are more than enough strands.

## OneWire

OneWire library provides peripheral functions of the bus itself:
- Send reset pulse (using driver)
- Write and read byte from bus
- Look up for devices
- Generic command _select_
- Generic command _skip_
- CRC8 calculation (no lookup tables)

**Note:** look up of alarmed devices not provided. That can be done in application layer by checking
scratchpad. Also, there's not much difference between coding _if's_ for alarm or for internal
temperature comparison. I admit, configuring sensors off-site and connecting them preconfigured on-site
can be useful, but again: check the scratchpad.

## Drivers

Driver's API is described in *ow_driver.h* file. The driver's structure is hidden from OneWire library
by providing only pointer to it:

```typedef struct one_wire_driver * ow_driver_ptr;```

The driver provides the following API:
- Create and initialize driver (memory's pre-allocated for embedded devices)
- Release driver (not used in embedded devices)
- Provide hardware reset pulse
- Provide reading and writing a bit to the bus
- Provide reading and writing a byte (on USART, for example, not the same as 8 sequential bit operations)

Included example drivers:
- AVR bitbanging GPIO driver
- STM32F10x bitbanging GPIO driver
- STM32F10x USART1 to UART5 driver. Can write bytes using DMA
- Linux serial device USB-UART driver (FT232, CH304, PL011 etc.)

**Note:** STM32F10x drivers are implemented using CMSIS. No HAL, no LL, no Standar Peripherals.

**Note 2:** Bitbang drivers need external `delay_us` function. For STM32F10x using debug registers please
check [my other pile of STM32 libraries](https://github.com/darauble/STM32-ARM-Libs).

**Note 3:** This particular design has a caveat. Only one type of driver can be used at one time. I.e. if
one is using USART driver in their application, it is impossible to simultaneously use GPIO driver. I have
thought about it and decided this limitation is negligible.

Also, this obstacle _can be worked around_ by using C++ as core language and dividing source into
several namespaces. Programming rocks!

## Example Code

There is example code provided for each driver. Please check appropriate readme's.

### AVR Example

Fully compilable on Linux PC, when AVR GCC is installed. `Makefile` and `configure` provided.

### STM32F10x Examples

`main.c` is provided, however no `Makefile`. Please create your own using favourite toolchain and/or IDE.

For dependencies check above mentioned pile of libraries.

### ESP32 Example

Run `./configure`, add Xtensa compile to your PATH, set IDF_PATH and type `make`.

### Linux Example

Connect USB-UART converter to your PC or Raspberry Pi. `Makefile` is provided. Please check included
pictures for connection.

# Credits

I've added credits to appropriate header and source files, but here's full list in one place.

People:
- Hagai Shatz
- Jim Studt, Paul Stoffregen et al
- Chi Zhang aka dword1511
- Игорь (aka dadigor)
- steel_ne
- Jonas Trečiokas aka Jonis

URLs:
- http://www.jonis.eu/
- https://term.lt/
- https://github.com/PaulStoffregen/OneWire
- https://github.com/milesburton/Arduino-Temperature-Control-Library
- http://we.easyelectronics.ru/STM32/stm32-1-wire-dma-prodolzhenie.html
- http://we.easyelectronics.ru/STM32/esche-raz-o-stm32-i-ds18b20-podpravleno.html

