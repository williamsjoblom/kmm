#include <avr/io.h>

void spi_slave_init() {
    SPCR = _BV(SPE);
}

unsigned char spi_slave_read() {
    // Wait for receive.
    while (!(SPSR & _BV(SPIF)));
    return SPDR;
}
