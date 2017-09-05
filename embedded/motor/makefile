all:
	avr-gcc -Os -DF_CPU=16000000 -std=gnu99 -mmcu=atmega328p -c main.c timer.c
	avr-gcc -mmcu=atmega328p main.o timer.o -o stepper
	avr-objcopy -O ihex -R .eeprom stepper stepper.hex
	avrdude -v -p atmega328p -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:stepper.hex:i
