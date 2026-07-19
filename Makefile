CC ?= gcc
CFLAGS ?= -O2 -march=armv8-a -mtune=cortex-a53 -std=c11 -Wall -Wextra -Wpedantic
LDFLAGS ?=
LDLIBS ?=

SRC = src/main.c src/gpio_rpi.c src/lens_bus.c src/lens_proto.c
OBJ = $(SRC:.c=.o)
BIN = lensctl

all: $(BIN)

$(BIN): $(OBJ)
	$(CC) $(LDFLAGS) -o $@ $(OBJ) $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

clean:
	rm -f $(OBJ) $(BIN)

.PHONY: all clean
