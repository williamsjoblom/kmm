#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

#include "i2c_master.h"

uint8_t twst;

void i2c_init(){
  
  TWSR = 0xF8;    // Initial value for TWSR
  TWCR = 0x00;    // Reset control register
  TWBR = 0x48;    // TWBR @ 100kHz is 0x48 (72)
                  // half the speed yields 0x98 (152).

}

void i2c_stop(){
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
}

uint8_t i2c_start(){
  
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA); // Start condition

  while((TWCR & _BV(TWINT)) == 0); // Wait for transmission
  
  twst = TW_STATUS;
  if(twst != TW_START) // Return 1 if start not correctly transmitted
    return 1;

  return 0;
}

uint8_t i2c_select_slave(uint8_t address){
  
  TWDR = address; // Load either slave WRITE or READ
  
  TWCR = _BV(TWINT) | _BV(TWEN);

  while((TWCR & _BV(TWINT)) == 0); // Wait for transmission
  
  twst = TW_STATUS;
  // Return 1 if slave did not acknowledge WRITE or READ
  if((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK)) 
    return 1;
  
  return 0;
}

uint8_t i2c_select_register(uint8_t reg){
  
  TWDR = reg; // Load register address 
  
  TWCR = _BV(TWINT) | _BV(TWEN);

  while((TWCR & _BV(TWINT)) == 0); // Wait for transmission
  
  twst = TW_STATUS;
  // Return 1 if slave did not acknowledge WRITE or READ
  if(twst != TW_MT_DATA_ACK) 
    return 1;
  
  return 0;
}

uint8_t i2c_read_ack(){
  
  TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);

  while((TWCR & _BV(TWINT)) == 0); // Wait for transmission

  return TWDR;
}

uint8_t i2c_read_nack(){
  
  TWCR = _BV(TWINT) | _BV(TWEN);

  while((TWCR & _BV(TWINT)) == 0); // Wait for transmission

  return TWDR;
}