#ifndef _I2C_MASTER_H
#define _I2C_MASTER_H

#define ACC_WRITE  0b00110010
#define ACC_READ  0b00110011 
#define GYRO_WRITE 0b11010110
#define GYRO_READ 0b11010111 

#define ACC_START 0b10101000 // MSB READ + ADDR
#define GYRO_START 0b10101100 // MSB READ + ADDRs

#define GYRO_RANGE_250DPS 0x00
#define GYRO_RANGE_500DPS 0x10
#define GYRO_RANGE_20000DPS 0x20

#define CTRL_REG1_A 0x20 
#define CTRL_REG4_A 0x23

#define I2C_START 0
#define I2C_STOP 1
#define I2C_DATA_ACK 2
#define I2C_DATA_NACK 3


typedef struct _Packet {
  uint8_t transmission_error;
  uint8_t* data;
} Packet ; 

/*
 * Initialize registers for the master AVR on I2C interface.
 */
void i2c_init();

/*
 * Write a byte to the selected slave's register specified in function call.
 */
uint8_t i2c_write8(uint8_t slave_address, uint8_t reg_address, uint8_t byte);

/*
 * Handles a variety of transmit codes and takes the proper actions for each of
 * them that are used to communicate between slave and master.
 */
uint8_t i2c_transmit(uint8_t code);

/*
 * Starts a transmission between master and slave or in read cases sends a
 * repeated start condition to the slave.
 */
void i2c_start_or_repstart(Packet *transmisson_packet);

/*
 * Select which slave on the sensor to communicate with.
 */
void i2c_select_slave(uint8_t slave_address, Packet *transmisson_packet);

/*
 * Specify which register that the master whishes to read or write to.
 */
void i2c_select_register(uint8_t reg, Packet *transmisson_packet);

/*
 * Read a byte from slave's register with the intent to read once again.
 */
Packet* i2c_read_ack(Packet *transmisson_packet);

/*
 * Read a byte from the slave's register with the intent to stop reading
 * transmission after this call.
 */
Packet* i2c_read_nack(Packet *transmisson_packet);

#endif