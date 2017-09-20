
#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include "serial.h"

void delay_long() {
    unsigned int delay = 65500U;
    while (delay > 0) {
	asm("nop");
	delay--;
    }
}


inline unsigned char serial_rx_available (void) {
    return (UCSR0A & _BV(RXC0)) != 0;
}


inline unsigned char serial_tx_ready (void) {
    return (UCSR0A & _BV(UDRE0)) != 0;
}


unsigned char serial_read (void) {
    while (!serial_rx_available());
    return UDR0;
}


void serial_write(char data) {
    while (!serial_tx_ready());
    UDR0 = data;
}


void serial_write_uint(unsigned int i) {
    char buffer[32];
    itoa(i, buffer, 10);
    serial_write_str(buffer);
}


void serial_write_str(char* str) {
    while (*str != '\0') {
	serial_write(*str);
	str++;
    }
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


void serial_send_byte(char byte, FILE* stream) {
    while (!serial_tx_ready());
    UDR0 = byte;
}


FILE serial_out = FDEV_SETUP_STREAM(serial_send_byte, NULL, _FDEV_SETUP_WRITE);

void serial_init (void) {
    DDRD = _BV(1); // Set TX pin as output.
    //DDRB = _BV(0) | _BV(1) | _BV(3) | _BV(5); // TODO: Check what this does

    // Baud rate
    UBRR0H = (unsigned char) (MYUBRR >> 8);
    UBRR0L = (unsigned char) MYUBRR;

    // Enable receiver and transmitter
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Frame format: 8data, No parity, 1stop bit
    UCSR0C = (3 << UCSZ00);

    stdout = &serial_out;
}


