
#include <stdlib.h>
#include <avr/io.h>

#include "sensor/main.h"
#include "sensor/spi.h"
#include "sensor/I2C.h"

/**
 * Main.
 */
 int main() {
  DDRB = 0b11001000;
  DDRD = 0b01111111;
  DDRC = 0b11111100;
  
  // Write cool logic to fetch data with I2C and then send that
  // over SPI to the pi yo.
  
  spi_slave_init();

  while (1) {
    
  }

  return 1;
 }


 