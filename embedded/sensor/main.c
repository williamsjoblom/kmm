#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

#include "main.h"
#include "spi.h"
#include "I2C.h"

/**
 * Main.
 */
 int main() {
  DDRB = 0b11001000;
  DDRD = 0b01111111;
  DDRC = 0b11111100;

  TWBR = (F_CPU / 100000UL - 16)/2;
  
  // Write cool logic to fetch data with I2C and then send that
  // over SPI to the pi yo.
  
  spi_slave_init();
  uint8_t buf[4];
  uint8_t data_out[7];
  uint8_t tst[4];
  int acc_return_value;
  int gyro_return_value;
  int n = 0;
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  while (1){
    n++;
    acc_return_value = twi_read_bytes(ACC_ADDR, ACC_START, buf);
    if (acc_return_value == 0){
      data_out[0] = buf[0];
      data_out[1] = buf[1];
      data_out[2] = buf[2];
      data_out[3] = buf[3];
      gyro_return_value = twi_read_bytes(GYRO_ADDR, GYRO_START, buf);
      if (gyro_return_value == 0) {
        data_out[4] = buf[0];
        data_out[5] = buf[1];
        data_out[6] = (uint8_t)n;
        spi_slave_write(data_out, 7);

      } 
    }
  }
  return -1;
 }


 /* while (1) {
  
  }
  return -1;
 }
 */
 //while (1) {
  /* acc_return_value = twi_read_bytes(ACC_ADDR, buf);
   if (acc_return_value == 0){
     //data_out[0] = buf[0];
     //data_out[1] = buf[1];
     //data_out[2] = buf[2];
     //data_out[3] = buf[3];
     tst[0] = 0xFF;
     tst[1] = 0x1;
   } else {

   }
   gyro_return_value = twi_read_bytes(GYRO_ADDR, buf);
   if (gyro_return_value == 0) {
       //data_out[4] = buf[1];
       //data_out[5] = buf[2];
       //spi_slave_write(data_out, 6);
     } 
   spi_slave_write(tst, 6); */