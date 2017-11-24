OBJ=$(patsubst %.c, %.o, $(wildcard *.c))

all: build prog

build: sensor_tst.hex

sensor_tst.hex: sensor_tst
	avr-objcopy -O ihex -R .eeprom sensor_tst sensor_tst.hex

sensor_tst: $(OBJ)
	avr-gcc -mmcu=atmega328p $(OBJ) -o $@

%.o: %.c
	avr-gcc -Os -DF_CPU=16000000 -fgnu89-inline -std=gnu99 -mmcu=atmega328p -c $< -o $@

prog:
	avrdude -c atmelice_isp -p atmega328p -U flash:w:sensor_tst.hex -U lfuse:w:0xe0:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

deploy: sensor_tst.hex
	avrdeploy 0 sensor_tst.hex

clean:
	rm -f *.o
	rm sensor_tst.hex sensor_tst


.PHONY: all build prog clean test
