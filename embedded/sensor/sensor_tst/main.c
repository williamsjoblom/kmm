#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>
#include <util/delay.h>

#include "main.h"
#include "spi.h"
#include "i2c_master.h"

#define BUFFER_SIZE 6

uint8_t validate_data(uint8_t buf[]) {
  
  for(int i = 0; i < BUFFER_SIZE; i++) {
    if((buf + i) == NULL) {
      return 1;
    }
  }
  return 0;
}

void get_acc_data(uint8_t buf[], Packet *acc_packet) {

  i2c_start_or_repstart(acc_packet);
  i2c_select_slave(ACC_WRITE, acc_packet);
  i2c_select_register(ACC_START, acc_packet);

  //i2c_stop();
  i2c_start_or_repstart(acc_packet);
  i2c_select_slave(ACC_READ, acc_packet);

  int n = 0;
  Packet *read_packet;
  for(int i = 3; i > 0; i--) {
    read_packet = i2c_read_ack(acc_packet);  
    buf[n] = *(read_packet->data);
    n++;
  }
  read_packet = i2c_read_nack(acc_packet);
  buf[n] = *(read_packet->data);
}

void get_gyro_data(uint8_t buf[], Packet *gyro_packet) {

  i2c_start_or_repstart(gyro_packet);
  i2c_select_slave(GYRO_WRITE, gyro_packet);
  i2c_select_register(GYRO_START, gyro_packet);

  //i2c_stop();
  i2c_start_or_repstart(gyro_packet);
  i2c_select_slave(GYRO_READ, gyro_packet);

  int n = 4;
  Packet *read_packet;
  for(int i = 1; i > 0; i--) {
    read_packet = i2c_read_ack(gyro_packet);  
    buf[n] = *(read_packet->data);
    n++;
  }
  read_packet = i2c_read_nack(gyro_packet);
  buf[n] = *(read_packet->data);
}

int main() {
  DDRB = 0x93;
  DDRD = 0xFF;
  DDRC = 0xBF;

  
  i2c_init();
  spi_slave_init();
  uint8_t buf[BUFFER_SIZE];
  Packet acc_packet, gyro_packet;

  acc_packet.transmission_error = 0;
  acc_packet.data = NULL;
  gyro_packet.transmission_error = 0;
  gyro_packet.data = NULL;
  buf[0] = 0x00;
  buf[1] = 0x00;
  buf[2] = 0x00;
  buf[3] = 0x00;
  buf[4] = 0x00;
  buf[5] = 0x00;

  while(1) {
    
    // Accelerometer data fetch
    get_acc_data(buf, &acc_packet);

    // Gyroscope data fetch
    get_gyro_data(buf, &gyro_packet);

    // Publish to Raspberry Pi 3
    spi_slave_write(buf, 6);
  }
}