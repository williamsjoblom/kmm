
#ifndef SERIAL_H
#define SERIAL_H

#define BAUD_RATE 9600
#define MYUBRR (F_CPU / 16 / BAUD_RATE ) - 1

void delay_long();

unsigned char serial_rx_available (void);

unsigned char serial_tx_ready (void);

unsigned char serial_read (void);

void serial_write(unsigned char data);

void serial_establish_contact (void);

void serial_init (void);

#endif
