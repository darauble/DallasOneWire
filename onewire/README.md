# OneWire Library

This is hardware agnostic implementation of the OneWire protocol by relying
on particular drivers. Provides basic OneWire protocol functions:
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

