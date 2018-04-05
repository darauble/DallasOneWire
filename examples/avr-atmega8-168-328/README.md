# AVR GPIO Driver Usage Example

First, install AVR toolchain. It is fairly easy on Linux. However,
if it's first time, please read [this comprehensive guide (Linux, macOS, Windows)](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).

Next, run `./configure` to download UART library by Peter Fleury. It is
used to print output to AVR's UART.

Adjust frequency and MCU model in `Config.in`.

And then you can run `make`.

Example code initializes driver on provided PIN (pin 3 on PORTD or Pin 3 on Arduino).
Then it tries to lookup OneWire devices and save their serial numbers in memory (up
to 10, can be adjusted). Then main loop issues a command to convert temperature
and later reads it.

**Note:** Fat and heavy float numbers printing is used in this example. It is advisable
not to do that when possible. I.e. no `-u,vfprintf -lprintf_flt` flags for compiler.

Example output to UART:

```OneWire AVR driver!
Driver on
One wire initialized
Device found 1
Device found 2
Convert all
0x28FF16E67316059A  93 01 4B 46 7F FF 0C 10 F6  25.1875
0x28FF43168016049B  8D 01 4B 46 7F FF 0C 10 1B  24.8125```
