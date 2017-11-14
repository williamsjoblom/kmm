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

/*
 * Set TWBR to a value higher than 10 to produce the correct SDA and
 * SCL output as master on the TWI. Can be set dynamically with the
 * following equation:
 *
 * TWBR = (CPU_FREQUENCY/SCL_FREQUENCY-16)/2
 * 
 * Note: if the AVR has a clock frequency lower than 3.6MHz, set the
 * TWBR to 10.
 */
void ioinit(void){
  #if defined(TWSP0)
    TWSR = 0;
  #endif

  #if CPU_FREQ < 3600000UL
    TWBR = 10;
  #else
    TWBR = (CPU_FREQ / SCL_FREQ - 16) / 2;
  #endif
}


/* Acceleration data is composed of 16-bit values per coordinate.
 * Gyroscope data is composed of 16-bit values per coordinate.
 * IIC can read one byte (8 bits) at a time, meaning every fetch
 * will take 4 reads, assuming data in the z-axis is redundant.
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

int twi_assess_error(unsigned char type){
  switch(type){
    case TWI_ERROR:
      TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTO);
      return -1;
    case TWI_RESTART:
      return 1;
  }
}

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

int twi_read(uint8_t addr, uint8_t *buf){
  
}


int twi_repeat_start(uint8_t addr, uint8_t *buf){ //EDIT 
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:
      /*FALLTHROUGH*/
    case TW_START:
      return twi_read(iter, addr, buf);

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);

    default:
      return twi_assess_error(TWI_ERROR);
  }
}


int twi_select_register(uint8_t addr){
  TWDR = addr | TW_READ; 
  twst = twi_transmit(TWI_DATA);
  switch(twst){
    case TW_MT_SLA_ACK:
      return twi_repeat_start(addr);

    case TW_MT_SLA_NACK:
      return twi_assess_error(TWI_ERROR);

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);
    
    default:
      return twi_assess_error(TWI_ERROR)
  }
}

int twi_select_slave(const uint8_t &sad, uint8_t addr){
  TWDR = sad | TD_WRITE;
  twst = twi_transmit(TWI_DATA);
  switch(twst) {
    case TW_MT_SLA_ACK:
      return twi_select_register(addr);

    case TW_MT_SLA_NACK:
      /*FALLTHROUGH*/
    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);
    
    default:
      return twi_assess_error(TWI_ERROR);  
  }
}


int twi_send_start(const uint8_t &sad, uint8_t addr){
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:
    /*FALLTHROUGH*/
    case TW_START:
      return twi_select_slave(sad, addr);

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);

    default:
      return -1;
  }
}


int twi_read_bytes(const uint8_t &sad, uint8_t *buf) {
  uint8_t twcr, n, addr = 0;
  int return_value = -1;

  if(sad == ACC_ADDR) {
    addr = ACC_START;
  } else{
    addr = GYRO_START;
  }
  // OLD LOOP WAS HERE

  while(n++ <= MAX_ITER){
    return_value = twi_send_start(sad, addr);
    if (return_value == 0)
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