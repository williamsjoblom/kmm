#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include "main.h"
#include "spi.h"
#include "i2c_master.h"


void get_acc_data(uint8_t buf[]){
  int n = 0;
  for(int i = 3; i > 0; i--){
    buf[n] = i2c_read_ack(); 
    n++;
  }
  buf[n] = i2c_read_nack();
}

void get_gyro_data(uint8_t buf[]){
  int n = 4;
  for(int i = 1; i > 0; i--){
    buf[n] = i2c_read_ack(); 
    n++;
  }
  buf[n] = i2c_read_nack();
}

int main(){
  DDRB = 0x93;
  DDRD = 0xFF;
  DDRC = 0xBF;

  i2c_init();
  spi_slave_init();
  uint8_t buf[4];
  uint8_t data_out[6];
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  
  while(1){
    
    // Accelerometer data fetch
    i2c_start();
    i2c_select_slave(ACC_WRITE);
    i2c_select_register(ACC_START);

    i2c_stop();
    _delay_us(1); // Delay after stop condition
    i2c_start();
    i2c_select_slave(ACC_READ);
    get_acc_data(data_out);
    i2c_stop();

    // Gyroscope data fetch
    i2c_start();
    i2c_select_slave(GYRO_WRITE);
    i2c_select_register(GYRO_START);

    i2c_stop();
    _delay_us(1); // Delay after stop condition
    i2c_start();
    i2c_select_slave(GYRO_READ);
    get_gyro_data(data_out);
    i2c_stop();

    // Publish to Raspberry Pi 3
    spi_slave_write(data_out, 6);
  }
}