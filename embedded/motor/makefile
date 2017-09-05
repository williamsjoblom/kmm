all:
	avr-gcc -Os -mmcu=atmega328p -c stepper.c
	avr-gcc -mmcu=atmega328p stepper.o -o stepper
	avr-objcopy -O ihex -R .eeprom stepper stepper.hex
	avrdude -v -p atmega328p -c arduino -P /dev/ttyUSB0 -b 57600 -D -U flash:w:stepper.hex:i
