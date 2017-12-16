/*
 * i2c_master.c
 *
 * Created: 24/11/2017
 * Author: hamer848, sabse455 
 */

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

#include "i2c_master.h"

uint8_t twst;

void i2c_init() {

  TWSR = 0x00;             // Initial value for TWSR
  TWCR &= ~(_BV(TWIE));    // Reset control register
  TWBR = 0x48;             // TWBR @ 100kHz is 0x48 (72)

  i2c_write8(ACC_WRITE, CTRL_REG1_A, 0x57);   // Power on accelerometer
  i2c_write8(GYRO_WRITE, CTRL_REG1_A, 0x0F);  // Power on gyroscope

  // Increase Gyro range to avoid hitting the range roof.
  //i2c_write8(GYRO_WRITE, CTRL_REG4_A, GYRO_RANGE_500DPS); 

  // Enable the High Resolution mode for accelerometer.
  i2c_write8(ACC_WRITE, CTRL_REG4_A, 0x08); 
}

uint8_t i2c_write8(uint8_t slave_address, uint8_t reg_address, uint8_t byte) {

  twst = i2c_transmit(I2C_START);
  if((twst == TW_REP_START) || (twst == TW_START)) {
    TWDR = slave_address;
    twst = i2c_transmit(I2C_DATA_NACK);
    if(twst == TW_MT_SLA_ACK) {
      TWDR = reg_address;
      twst = i2c_transmit(I2C_DATA_NACK);
      if(twst == TW_MT_DATA_ACK) {
        TWDR = byte;
        twst = i2c_transmit(I2C_DATA_NACK);
        if(twst == TW_MT_DATA_ACK) {
          return i2c_transmit(I2C_STOP);
        } else {
          return 1;
        } 
      } else {
        return 1;
      } 
    } else {
      return 1;
    }
  } else {
    return 1;
  }
}

uint8_t i2c_transmit(uint8_t code) {

  switch(code) {
    case I2C_START:
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
    break;

    case I2C_STOP:
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
    return 0;

    case I2C_DATA_ACK:
    TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    break;

    case I2C_DATA_NACK:
    TWCR = _BV(TWINT) | _BV(TWEN);
    break;

    default:
    return 1;
  }

  while((TWCR & _BV(TWINT)) == 0);  // Wait for transmission to be done
  return TW_STATUS;               // return valid status
}

void i2c_start_or_repstart(Packet *transmission_packet) {

  if(!(transmission_packet->transmission_error)) {
    twst = i2c_transmit(I2C_START);
    // Return 1 if start not correctly transmitted
    if((twst != TW_START) && (twst != TW_REP_START)) {
      transmission_packet->transmission_error = 1;
    }
  }
}

void i2c_select_slave(uint8_t slave_address, Packet *transmission_packet) {

  if(!(transmission_packet->transmission_error)) {
    TWDR = slave_address; // Load either slave address as WRITE or READ
    twst = i2c_transmit(I2C_DATA_NACK);

    // Return 1 if slave did not acknowledge WRITE or READ
    if((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) {
      transmission_packet->transmission_error = 1;
    }
  }
}

void i2c_select_register(uint8_t reg_address, Packet *transmission_packet) {

  if(!(transmission_packet->transmission_error)) {
    TWDR = reg_address; // Load register address
    twst = i2c_transmit(I2C_DATA_NACK);

    // Return 1 if slave did not acknowledge register address
    if(twst != TW_MT_DATA_ACK) {
      transmission_packet->transmission_error = 1;
    }
  }
}

Packet* i2c_read_ack(Packet *transmission_packet) {

  if(!(transmission_packet->transmission_error)) {
    twst = i2c_transmit(I2C_DATA_ACK);

    // Return 1 if slave did not acknowledge the read
    if(twst != TW_MR_DATA_ACK) {
      transmission_packet->transmission_error = 1;
    } else {
      *(transmission_packet->data) = TWDR;
    }
  }

  return transmission_packet;
}

Packet* i2c_read_nack(Packet *transmission_packet) {

  if(!(transmission_packet->transmission_error)) {
    twst = i2c_transmit(I2C_DATA_NACK);

    // Return 1 if slave did not return NO_ACK on read request
    if(twst != TW_MR_DATA_NACK) {
      transmission_packet->transmission_error = 1;
    } else {
      *(transmission_packet->data) = TWDR;
      i2c_transmit(I2C_STOP);
    }
  }

  return transmission_packet;
}