#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#define ACC_WRITE  0b00110010
#define ACC_READ  0b00110011 
#define GYRO_WRITE 0b11010110
#define GYRO_READ 0b11010111 

#define ACC_START 0b10101000 // MSB READ + ADDR
#define GYRO_START 0b10101100 // MSB READ + ADDRs

#define MAX_ITER 200


void i2c_init();
void i2c_stop();
void i2c_error();
uint8_t i2c_start();
uint8_t i2c_select_slave(uint8_t address);
uint8_t i2c_select_register(uint8_t reg);
uint8_t i2c_read_ack();
uint8_t i2c_read_nack();


#endif