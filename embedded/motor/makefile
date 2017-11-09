OBJ=$(patsubst %.c, %.o, $(wildcard *.c))

all: build prog

build: stepper.hex

stepper.hex: stepper
	avr-objcopy -O ihex -R .eeprom stepper stepper.hex

stepper: $(OBJ)
	avr-gcc -mmcu=atmega168 $(OBJ) -o $@

%.o: %.c
	avr-gcc -Os -DF_CPU=8000000 -std=gnu99 -mmcu=atmega168 -c $< -o $@

prog:
	avrdude -c atmelice_isp -p atmega168 -U flash:w:stepper.hex

clean:
	rm -f *.o
	rm stepper.hex stepper


.PHONY: all build prog clean test
