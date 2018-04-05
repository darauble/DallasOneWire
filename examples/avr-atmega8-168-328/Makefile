TGT=avr-dallas-gpio
CC=avr-gcc
OBJCPY=avr-objcopy
OBJSIZE=avr-size
CFLAGS=-Wall
BUILD_FLAGS=-Os -Wl,-u,vfprintf -lprintf_flt -lm -g
SRC=main.c uart.c ../../dallas/dallas.c ../../onewire/onewire.c ../../drivers/ow_driver_avr_gpio.c
INCLUDES=-I. -I../../dallas -I../../onewire -I../../drivers

-include Config.in

all: $(SRC:.c=.o) $(TGT) hex

$(SRC:.c=.o):
	$(CC) $(INCLUDES) $(CFLAGS) $(BUILD_FLAGS) -mmcu=$(MCU) -DF_CPU=$(F_CPU) -c $(@:.o=.c) -o $@

$(TGT): $(SRC:.c=.o)
	$(CC) $(BUILD_FLAGS) -mmcu=$(MCU) -o $(TGT).elf $(SRC:.c=.o)

hex:
	$(OBJCPY) -j .text -j .data -O ihex $(TGT).elf $(TGT).hex
	$(OBJSIZE) --format=avr --mcu=$(MCU) $(TGT).elf

clean:
	rm $(SRC:.c=.o)
	rm $(TGT).*
