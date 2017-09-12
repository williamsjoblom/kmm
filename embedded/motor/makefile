OBJ=$(patsubst %.c, %.o, $(wildcard *.c))

all: build prog

build: stepper.hex

stepper.hex: stepper
	avr-objcopy -O ihex -R .eeprom stepper stepper.hex

stepper: $(OBJ)
	avr-gcc -mmcu=atmega328p $(OBJ) -o $@

%.o: %.c
	avr-gcc -Os -DF_CPU=16000000 -std=gnu99 -mmcu=atmega328p -c $< -o $@

prog:
	avrdude -v -p atmega328p -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:stepper.hex:i

clean:
	rm -f *.o
	rm stepper.hex stepper

.PHONY: all build prog clean
