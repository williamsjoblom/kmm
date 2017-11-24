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
  DDRB = 0x93;
  DDRD = 0xFF;
  DDRC = 0xBF;

  i2c_init();
  spi_slave_init();
  uint8_t buf[4];
  uint8_t data_out[6];
  int acc_return_value;
  int gyro_return_value;
  int n = 0;
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  while (1){
    n++;
    acc_return_value = twi_read_bytes(ACC_ADDR, ACC_START, buf, 4);
    if (acc_return_value == 0){
      data_out[0] = buf[0];
      data_out[1] = buf[1];
      data_out[2] = buf[2];
      data_out[3] = buf[3];
      gyro_return_value = twi_read_bytes(GYRO_ADDR, GYRO_START, buf, 2);
      if (gyro_return_value == 0) {
        data_out[4] = buf[0];
        data_out[5] = buf[1];
        //data_out[6] = (uint8_t)n;
        //spi_slave_write(data_out, 6);

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
