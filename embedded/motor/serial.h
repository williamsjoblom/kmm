
#ifndef SERIAL_H
#define SERIAL_H

#include <stdio.h>

#define BAUD_RATE 9600
#define MYUBRR (F_CPU / 16 / BAUD_RATE ) - 1

void delay_long();

unsigned char serial_rx_available (void);

unsigned char serial_tx_ready (void);

unsigned char serial_read (void);

void serial_write(char data);
void serial_write_str(char* str);
void serial_write_uint(unsigned int i);

void serial_establish_contact (void);

void serial_init (void);

/**
 * Printf
 */
void serial_send_byte(char byte, FILE* stream);

#endif
