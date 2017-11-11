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

int twi_read_bytes(uint8_t sad, int iter, uint8_t *buf) {
  uint8_t twcr, n, addr = 0;
  int return_value = 0;
  
  if(sad == ACC_ADDR) {
    addr = ACC_START;
  } else{
    addr = GYRO_START;
  }

  restart:
  if(n++ >= MAX_ITER)
    return -1;

  begin:
  TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
  while((TWCR & _BV(TWINT)) == 0);
  switch((twst = TW_STATUS)) {
    case TW_REP_START:
    case TW_START:
      break;
    
    case TW_MT_ARB_LOST:
      goto begin;
    
    default:
      return -1;
  }

  TWDR = sad | TD_WRITE;
  TWCR = _BV(TWINT) | _BV(TWEN);
  while((TWCR & _BV(TWINT)) == 0);
  switch((twst = TW_STATUS)) {
    case TW_MT_SLA_ACK:
      break;
    
    case TW_MT_SLA_NACK:
      goto restart;
    
    case TW_MT_ARB_LOST:
      goto begin;
    
    default:
      goto error;
  }
   
  

}
