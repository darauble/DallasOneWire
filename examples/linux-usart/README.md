# Linux USB-UART Driver Example

Just run `make` on your Linux PC, virtual machine or Raspberry Pi. Example
is hardcoded to use `/dev/ttyUSB0`, as a first enumerated device, so if required,
adjust accordingly.

Please take a good look on included schematic and a picture how to connect
converter to the sensor. It is highly important to connect TX pin via Schottky
diode, as otherwise sensor will short TX pin to ground and burn it.

Schematic shows how Dallas DS18B20 sensor's DQ pin is connected to USART's TX
pin via diode. And the photo displays actual testing setup.

Example output:

```$ ./linux-dallas-uart
Init completed
Device found 1
Device found 2
Device found 3
Convert all
0x28FFB24C74160420  93 01 4B 46 7F FF 0C 10 F6  25.1875
0x28FF16E67316059A  96 01 4B 46 7F FF 0C 10 A0  25.3750
0x28FF43168016049B  8E 01 4B 46 7F FF 0C 10 DE  24.8750
Convert all
0x28FFB24C74160420  93 01 4B 46 7F FF 0C 10 F6  25.1875
0x28FF16E67316059A  96 01 4B 46 7F FF 0C 10 A0  25.3750
0x28FF43168016049B  8E 01 4B 46 7F FF 0C 10 DE  24.8750```
