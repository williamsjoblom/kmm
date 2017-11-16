#include<sensor/spi.h>
#include <avr/io.h>

#define DDR_SPI DDRB
#define DD_MISO DDB4

void spi_slave_init() {
  DDR_SPI = _BV(DD_MISO); // Set MISO as output pin
  SPCR = _BV(SPE); // Enable SPI
}

void spi_slave_write(uint8_t *data, uint8_t len) {
  // Shift the full array of either accelero/gyroscope data
  // without recieving any byte from master.
  uint8_t i;

  for(i = 0; i < len; i++){
    SPDR = data[i];
    // Wait transmission complete
    while (!(SPSR & _BV(SPIF)));
  }
}
