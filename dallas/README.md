# Dallas DS18xxx Library

Programmer-oriented software library for interfacing Dallas OneWire temperature sensors.

Provided functions:
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

## Notes

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

Check the comments in `dallas.h`, also code examples.

