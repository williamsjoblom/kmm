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
      TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWSTA);
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

int twi_read(uint8_t addr, uint8_t *buf, const uint8_t sad){
  TWDR = sad | TW_READ;
  twst = twi_transmit(TWI_DATA);
  switch (twst) {
    case TW_MR_SLA_ACK:
        break;

    case TW_MR_SLA_NACK:
      PORTD = _BV(PD5);
      //return twi_assess_error(TWI_ERROR);
      twi_assess_error(TWI_ERROR);  
      return -6;

    case TW_MR_ARB_LOST:
      return twi_assess_error(TWI_RESTART);

    default:
      //return twi_assess_error(TWI_ERROR);
      twi_assess_error(TWI_ERROR);  
      return -7;
  }
  int iter;
  if (sad == ACC_ADDR) {
    iter = 4;     //Read X- and Y -data from accelerometer
  } else {
    iter = 2;     //Read Z-data from gyro
  }
  for (int i = 0; i <= iter; i++) {
    if (i == 1) {
      TWCR = _BV(TWINT) | _BV(TWEN);
    } else {
      TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
    }
    while((TWCR & _BV(TWINT)) == 0);
    switch((twst = TW_STATUS)){
      case TW_MR_DATA_NACK:
        buf[i] = TWDR;
        //return twi_assess_error(TWI_ERROR);
        twi_transmit(TWI_STOP);  
        return 0;
      case TW_MR_DATA_ACK:
        PORTD = _BV(PD6);
        buf[i] = TWDR;
        break;

      default:
        //return twi_assess_error(TWI_ERROR);
        twi_assess_error(TWI_ERROR);  
        return -9;
    }
  }
  return twi_transmit(TWI_STOP);
}

/* Sends repeated start condition, #4 in event loop*/
int twi_repeat_start(uint8_t addr, uint8_t *buf, const uint8_t sad){
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_START:
      /*FALLTHROUGH (Is okay, but should not happen)*/
    case TW_REP_START:
      return twi_read(addr, buf, sad);

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);

    default:
      //return twi_assess_error(TWI_ERROR);
      twi_assess_error(TWI_ERROR); 
      return -5; 
  }
}

/* Selects which data register to read from,
  #3 in event loop*/
int twi_select_register(uint8_t addr, const uint8_t sad, uint8_t *buf){
  TWDR = addr; 
  twst = twi_transmit(TWI_DATA);
  switch(twst){
    case TW_MT_DATA_ACK:
      return twi_repeat_start(addr, buf, sad);

    case TW_MT_DATA_NACK:
      //PORTD = _BV(PD7);
      //return twi_assess_error(TWI_ERROR);
      twi_assess_error(TWI_ERROR);  
      return -3;

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);
    
    default:
      //return twi_assess_error(TWI_ERROR);
      twi_assess_error(TWI_ERROR); 
      return -4;
  }
}

/* Selects what sensor/slave to read from, either gyro or accelerometer
  #2 in event loop*/
int twi_select_slave(const uint8_t sad, uint8_t addr, uint8_t *buf){
  TWDR = sad | TW_WRITE;
  twst = twi_transmit(TWI_DATA);
  switch(twst) {
    case TW_MT_SLA_ACK:
      return twi_select_register(addr, sad, buf);

    case TW_MT_SLA_NACK:
      /*FALLTHROUGH*/
      PORTB = _BV(PB0);
    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);
    
    default:
      //return twi_assess_error(TWI_ERROR);  
      twi_assess_error(TWI_ERROR); 
      return -2; 
  }
}

/* Sends start condition, #1 in event loop*/
int twi_send_start(const uint8_t sad, uint8_t addr, uint8_t *buf){
  twst = twi_transmit(TWI_START);
  switch(twst) {
    case TW_REP_START:
    /*FALLTHROUGH*/
    case TW_START:
      return twi_select_slave(sad, addr, buf);

    case TW_MT_ARB_LOST:
      return twi_assess_error(TWI_RESTART);

    default:
      return -1;
  }
}

/* Reads the data from the adafruit sensor, starts the event loop*/
int twi_read_bytes(const uint8_t sad, uint8_t addr, uint8_t *buf) {
  uint8_t n = 0;
  int return_value = 0;
  PORTD = _BV(PB0);
  while(n++ <= MAX_ITER){
    return_value = twi_send_start(sad, addr, buf);
    if (return_value <= 0)
      PORTD = 0x00;
      return return_value;
  }
  return -10;
}