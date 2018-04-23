# OneWire Hardware Drivers

The driver API is described in the `ow_driver.h` file. There'is no actual
driver structure provided, only a pointer to it. In this way higher level
software can be fully agnostic of underlying hardware.

## Provided Functions

Driver's API provides the following functions:
- Create and initialize driver (memory's pre-allocated for embedded devices)
- Release driver (not used in embedded devices)
- Provide hardware reset pulse
- Provide reading and writing a bit to the bus
- Provide reading and writing a byte (on USART, for example, not the same as 8 sequential bit operations)

## Driver Imlementations

Each particular driver also includes a header file. There are enums for easier use
and possible adjustments. Read the comments inside.

Note, that each driver is initialized using the same function:
```int init_driver(ow_driver_ptr*, int);```

That means, driver's configuration (GPIO pin, USART etc) must be configured using single
integer constant. However, fear not, explanations and user-friendly constants are provided
for this purpose.

So, for example, let's say, we are initializing a driver on USART3 of STM32F10x MCU. It is as
easy as that:

```status = init_driver(drv, E_USART3);```

Another example. Let's initialize a GPIO driver on STM32F10x MCU. Chosen port is GPIOB, and
chosen pin is PB5. Piece of cake:

```status = init_driver(drv, E_GPIOB+5);```

Take a note on `OW_YIELD` define in comments.

### AVR Bitbang GPIO

This is based on Jim Studt and Paul Stoffregen's most popular OneWire library. However, as it
is close-to-hardware implementation, it uses ports and pins directly.

For driver's initialization on particular port's pin friendly enum is provided. So PORTC and pin 3
becomes `E_PORTC+3`. There's also an enum for compatibility with Arduino Uno and Nano. So driver
can be initialized by configuring it with `E_ARDUINO_PIN_5`.

**Notes**
- This driver pre-allocates a blob of memory to initialize driver's instance. Check appropriate header to configure desired amount of "heap".
- Driver depends on external `delay_us` function for microseconds delay. Provide one yourself or take from example code.

### STM32F10x Bitbang GPIO

Almost the same as AVR, only using different MCU. Notes are the same. But for `delay_us` you
can use [my own implementation using DWT registers](https://github.com/darauble/STM32-ARM-Libs/tree/master/Utils).

**Notes**
- This driver pre-allocates a blob of memory to initialize driver's instance. Check appropriate header to configure desired amount of "heap".

### STM32F10x U(S)ART+Optional DMA

This is where it all started. As per [Maxim's tutorial](https://www.maximintegrated.com/en/app-notes/index.mvp/id/214), USART is a perfect hardware master
for OneWire devices and is readily available on almost any MCU. ARM MCUs tend to have more
than one USART, so that's where it becomes convenient, cost effective and error proof. GPIO
bitbanging cannot be that accurate and fails when is used in multitasking RTOSes (incorrectly
and/or lazily).

USART, on the other hand, is controlled by separate hardware. A byte sent over USART cannot be
"jittered" into delayed bits by loading main CPU. And for OneWire, the byte over USART is
a controlled and guaranteed bit.

Even more fun: using DMA controller it is possible to make full OneWire byte (8 USART bytes) to
be sent by separate hardware thus releasing CPU resources and yielding RTOS task (as an example).
This provides even more flexibility on supported hardware.

**Notes**
- This driver pre-allocates a blob of memory to initialize driver's instance. Check appropriate header to configure desired amount of "heap".
- It is safe to connect together RX/TX pins of the same USART, as TX is configured as _open drain_.

**__BIG NOTE__**

I was not able to get DMA working also for reading, using the same (or separate) buffer on the
same USART register. I am sure I'm simply missing something, but couldn't nail it down and
decided to leave for some time. 

_So code review and contributions are more than welcome_

### ESP32 UART Driver

This driver is based on top of Espressif's IDF provided UART driver, designed especially for FreeRTOS with
resource spinlocks and indirect memory-to-peripheral mapping. It is quite heavyweight, but ESP32 can
handle it. As it is not my main priority, I will not put too much effort into implementing it as lightweight
as possible. And there's no need to.

ESP32 SDK provides `malloc` and `free`, so driver release routine is also implemented.

### Linux USB-UART

Or, actually, any serial device, controlled by Linux kernel. It is a fun driver, as one can
easily connect USB converter, attach Dallas sensors and read temperature using the same USART
principles as with the MCU. No DMA there, but with OS and lots of memory it is possible to use
big memory buffers to write to the bus. However, I've implemeted one byte (OneWire bit) approach.

It is much more stable and safe than using Raspberry Pi's GPIO, for example. If there's static
on the wire, only USB-UART would burn instead of precious GPIO (or all Pi, as a matter of fact).
It is more easy to divide star topology wires into shorter segments with more adapters.

## Limitations

Because of source architecture only one driver can be used at a time. USART and GPIO
cannot be used simulatenously. However, it is possible to use all five U(S)ARTs on single
STM32F107 MCU, for example. It is extremely useful, when star topology is used to
connect sensors, as it is desirable to keep full sum of wire length as short as possible.
So dividing a star into several segments improves reliability.

However, IMHO, the limitation of using only one type of driver at a time is negligible.
Also, this obstacle _can be worked around_ by using C++ as core language and dividing source into
several namespaces.

