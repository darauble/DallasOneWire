# ESP32 UART Example

First, install ESP32 toolchain: Xtensa toolkit and Espressif's IDF. If you are not familiar
with developing ESP32 applications, please check [how to get started.](https://dl.espressif.com/doc/esp-idf/latest/get-started/index.html)

Next, run `./configure` to create `components` directory with appropriate code inside.

Ensure PATH has Xtensa compiler and IDF_PATH is set to correct value.

Run `make` or `make flash` right away and check console.

Its is expected that UART0 is configured as `stdout`.

Example output to ESP32's `sdout`:

```OneWire UART driver on ESP32
Driver on
One wire initialized
Device found 1
Device found 2
Device found 3
Convert all
0x28FFB24C74160420  8F 01 4B 46 7F FF 0C 10 9D  24.93750
0x28FF16E67316059A  90 01 4B 46 7F FF 0C 10 33  25.00000
0x28FF43168016049B  8B 01 4B 46 7F FF 0C 10 88  24.68750
```
