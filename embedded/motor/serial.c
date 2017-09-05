
#include <avr/io.h>
#include "serial.h"

void delay_long() {
	unsigned int delay = 65500U;
	while (delay > 0) {
		asm("nop");
		delay--;
	}
}

unsigned char serial_rx_available (void) {
	return (UCSR0A & _BV(RXC0)) != 0;
}

unsigned char serial_tx_ready (void) {
	return (UCSR0A & _BV(UDRE0)) != 0;
}

unsigned char serial_read (void) {
	while (!serial_rx_available());
	return UDR0;
}

void serial_write(unsigned char data) {
	while (!serial_tx_ready());
	UDR0 = data;
}

void serial_establish_contact (void) {
	while (!serial_rx_available()) {
		serial_write('A'); // 65U
		delay_long();
		delay_long();
		delay_long();
		delay_long();
		delay_long();
		delay_long();
		delay_long();
	}
}

void serial_init (void) {
  DDRD = _BV(1); // TODO: Check what this does
  //DDRB = _BV(0) | _BV(1) | _BV(3) | _BV(5); // TODO: Check what this does

  // Baud rate
  UBRR0H = (unsigned char)(MYUBRR >> 8);
  UBRR0L = (unsigned char) MYUBRR;

  // Enable receiver and transmitter
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  // Frame format: 8data, No parity, 1stop bit
  UCSR0C = (3 << UCSZ00);

  // Turn on LED @ PB1
  PORTB |= _BV(1);

  serial_establish_contact();

  // Turn off LED
  PORTB &= 253U;
}
