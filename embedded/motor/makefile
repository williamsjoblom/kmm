OBJ=$(patsubst %.c, %.o, $(wildcard *.c))

all: build prog

build: stepper.hex

stepper.hex: stepper
	avr-objcopy -O ihex -R .eeprom stepper stepper.hex

stepper: $(OBJ)
	avr-gcc -mmcu=atmega328p $(OBJ) -o $@

%.o: %.c
	avr-gcc -Os -DF_CPU=16000000 -fgnu89-inline -std=gnu99 -mmcu=atmega328p -c $< -o $@

prog:
	avrdude -c atmelice_isp -p atmega328p -U flash:w:stepper.hex -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m 

clean:
	rm -f *.o
	rm stepper.hex stepper


.PHONY: all build prog clean test
