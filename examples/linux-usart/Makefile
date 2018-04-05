TGT=linux-dallas-uart
CC=gcc
CFLAGS=-Wall
BUILD_FLAGS=-O0 -g3 -c -fmessage-length=0
SRC=main.c ../../dallas/dallas.c ../../onewire/onewire.c ../../drivers/ow_driver_linux_usart.c
INCLUDES=-I../../dallas -I../../onewire -I../../drivers

all: $(SRC:.c=.o) $(TGT)

$(SRC:.c=.o):
	$(CC) $(INCLUDES) $(CFLAGS) $(BUILD_FLAGS) -c $(@:.o=.c) -o $@

$(TGT): $(SRC:.c=.o)
	$(CC) -o $(TGT) $(SRC:.c=.o)

clean:
	rm $(SRC:.c=.o)
	rm $(TGT)
