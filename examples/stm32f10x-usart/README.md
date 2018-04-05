# STM32F10x USART+DMA Driver Example

No `Makefile` provided, please create your own build environment
with favourite toolchain and/or IDE. Depends on CMSIS and might be
Standard Peripherals leftovers present.

Example code initializes OneWire driver on USART2.
Then it tries to lookup OneWire devices and save their serial numbers in memory (up
to 10, can be adjusted). Then main loop issues a command to convert temperature
and later reads it.

Example code depends on other libraries of mine, [check them out here](https://github.com/darauble/STM32-ARM-Libs).

**Notes**
- Please check included schematic. It is _safe_ to connect together OneWire USARTs TX/RX pins, as TX is configured as _open drain_.
- To enable DMA mode provide a define `OW_MODE_USART_DMA`
- Fat and heavy float numbers printing is used in this example. It is advisable not to do that when possible. I.e. no `-mfloat-abi=soft` flag for compiler and no `-u _printf_float` flag for linker.

Example output to USART1:

```OneWire DMA Driver!
OneWire DMA flag on
Driver on
One wire initialized
Device found 1
Device found 2
Device found 3
Convert all
0x28FFB24C74160420  F3 01 4B 46 7F FF 0C 10 17  31.1875
0x28FF16E67316059A  D8 01 4B 46 7F FF 0C 10 50  29.5
0x28FF43168016049B  E5 01 4B 46 7F FF 0C 10 D0  30.3125```
