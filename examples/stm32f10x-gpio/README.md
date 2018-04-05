# STM32F10x GPIO Bitbang Driver Example

No `Makefile` provided, please create your own build environment
with favourite toolchain and/or IDE. Depends on CMSIS and might be
Standard Peripherals leftovers present.

Example code initializes OneWire driver on GPIOA pin 5 (PA5).
Then it tries to lookup OneWire devices and save their serial numbers in memory (up
to 10, can be adjusted). Then main loop issues a command to convert temperature
and later reads it.

Example code depends on other libraries of mine, [check them out here](https://github.com/darauble/STM32-ARM-Libs).

**Note:** Fat and heavy float numbers printing is used in this example. It is advisable
not to do that when possible. I.e. no `-mfloat-abi=soft` flag for compiler and no
`-u _printf_float` flag for linker.

Example output to USART1:

```OneWire GPIO Driver!
Driver on
One wire initialized
Device found 1
Device found 2
Device found 3
Convert all
0x28FFB24C74160420  BA 01 4B 46 7F FF 0C 10 37  27.625
0x28FF16E67316059A  94 01 4B 46 7F FF 0C 10 26  25.25
0x28FF43168016049B  97 01 4B 46 7F FF 0C 10 E3  25.4375```
