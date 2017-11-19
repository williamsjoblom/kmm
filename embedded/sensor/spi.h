#ifndef _SPIAVR_H_
#define _SPIAVR_H_


void spi_slave_init();
void spi_slave_write(uint8_t *data, uint8_t len);

#endif
