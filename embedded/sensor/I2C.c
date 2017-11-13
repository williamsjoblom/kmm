#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/twi.h>

#include "I2C.h"


/* 
 * Variable to save possible error messages. This is done since
 * We can only guarantee valid content in TWSR while TWINT is set
 * in TWCR
 */
uint8_t twst;


/* Acceleration data is composed of 16-bit values per coordinate.
 * Gyroscope data is composed of 16-bit values per coordinate.
 * IIC can read one byte (8 bits) at a time, meaning every fetch
 * will take 4 reads, assuming data in the z-axis is redundant.
 */ 

/*
 * Reads are done in the following manner:
 * ST --> SAD+W --> SAK --> SUB --> SAK --> SR --> SAD+R --> SAK
 * --> DATA --> MAK --> DATA --> MAK --> DATA --> MAK --> DATA 
 * --> NMAK --> SP
 * 
 * ST := START CONDITION
 * SAD+R/W := SLAVE ADRESS + READ/WRITE
 * SAK := SLAVE ACKNOWLEDGE
 * SUB := REGISTER ADRESS + MSB AS 1
 * SR := REPEATED START
 * MAK := MASTER ACKNOWLEDGE
 * NMAK := NO MASTER ACKNOWLEDGE
 * SP := STOP CONDITION
 */

unsigned char twi_transmit(unsigned char type){
  switch(type){
    case TWI_START:
      TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
      break;
    case TWI_DATA:
      TWCR = _BV(TWINT) | _BV(TWEN);
      break;
    case TWI_STOP:
      TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
      return 0;
  }

  while((TWCR & _BV(TWINT)) == 0);
  return TW_STATUS;

}

int twi_read(int iter, uint8_t addr, uint8_t *buf){
  for(int i = iter; i >= 0; i--){
    TWDR = addr;
    twst = twi_transmit(TWI_DATA);
    switch(twst){
      case TW_MT_SLA_ACK:
        *buf++ = TWDR;
        // ADD CONDITION TO CHECK BUF LENGHT
      case TW_MT_SLA_NACK:
        i = 0;
        break;
      case TW_MT_ARB_LOST:

    }
  }
}


int twi_repeat_start(int iter, uint8_t addr, uint8_t *buf){ //EDIT 
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:
    case TW_START:
    {
      twi_read(iter, addr, buf);
      break;
    }
    case TW_MT_ARB_LOST:
      break;

    default:
      return -1;
  }
}


int twi_select_register(uint8_t addr){
  TWDR = addr | READ_BIT; // ADD ACTUAL BIT 
  twst = twi_transmit(TWI_DATA);
  switch(twst){
    case TW_MT_SLA_ACK:
    {
      twi_repeat_start();
    }
    case TW_MT_SLA_NACK:
      return 0;

    case TW_MT_ARB_LOST:
      return 0;
  }
}

int twi_select_slave(const uint8_t &sad, uint8_t addr){
  TWDR = sad | TD_WRITE;
  twst = twi_transmit(TWI_DATA);
  switch(twst) {
    case TW_MT_SLA_ACK:
    {
      twi_select_register(addr)
    }
    case TW_MT_SLA_NACK:
      return 0;

    case TW_MT_ARB_LOST:
      return 0;
  }
}


int twi_send_start(){
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:
    // Connect this one to TW_START??
    case TW_START:
    {
      twi_select_slave(sad);
      return_value = 0;
      break;
    }
    case TW_MT_ARB_LOST:
      break;

    default:
      return -1;
  }
}


int twi_read_bytes(const uint8_t &sad, int iter, uint8_t *buf) {
  uint8_t twcr, n, addr = 0;
  int return_value = -1;

  if(sad == ACC_ADDR) {
    addr = ACC_START;
  } else{
    addr = GYRO_START;
  }
  // OLD LOOP WAS HERE

while(n++ <= MAX_ITER){
  twi_send_start();
  if (return_value == 0):
    break; 
}
  return return_value;
}


  /*begin:
  if(n++ >= MAX_ITER)
    return -1;
    
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:

    case TW_START:
      break;
    
    case TW_MT_ARB_LOST:
      goto begin;
    
    default:
      return -1;
  }

  TWDR = sad | TD_WRITE;
  twst = twi_transmit(TWI_DATA);
  switch(twst) {
    case TW_MT_SLA_ACK:
      break;
    
    case TW_MT_SLA_NACK:
      goto begin;
    
    case TW_MT_ARB_LOST:
      goto begin;
    
    default:
      twi_transmit(TWI_STOP);
      return_value = -1;
      return return_value;
  }*/