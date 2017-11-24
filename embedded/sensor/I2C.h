#ifndef _TWI_H
#define _TWI_H

    #define ACC_ADDR  0b00110010 // + R/W bit
    #define GYRO_ADDR 0b11010110 // + R/W bit

    /* Register adresses for each coordinate. Used in SUB.
     * ACC_X_L   0b0101000
     * ACC_X_H   0b0101001
     * ACC_Y_L   0b0101010
     * ACC_Y_H   0b0101011
     * GYRO_Z_L  0b0101100
     * GYRO_Z_H  0b0101101
     */
    #define ACC_START 0b10101000 // MSB READ + ADDR
    #define GYRO_START 0b10101100 // MSB READ + ADDRs
    /*
    * Maximum no. iterations that we will wait for a selection to be
    * responded to by the slave. Used to be able to abort an infinte
    * loop, although still being enough for a pending write to finish.
    * WILL HAVE TO BE CHANGED IF CLOCK OF TWI IS CHANGED.
    * RN BASED ON A TWI CLOCK RATE OF 100 kHz.
    */
    #define MAX_ITER 200


    #define TWI_START 0
    #define TWI_DATA 1
    #define TWI_STOP 2
    #define TWI_ERROR 3
    #define TWI_RESTART 4
    #define TWI_REP_STOP 5

uint8_t i2c_enable_reading(uint8_t address, uint16_t enable);
void i2c_init(void);
int twi_assess_error(unsigned char type);
unsigned char twi_transmit(unsigned char type);
int twi_read(uint8_t addr, uint8_t *buf, const uint8_t sad, int iter);
int twi_repeat_start(uint8_t addr, uint8_t *buf, const uint8_t sad, int iter);
int twi_select_register(uint8_t addr, const uint8_t sad, uint8_t *buf, int iter);
int twi_select_slave(const uint8_t sad, uint8_t addr, uint8_t *buf, int iter);
int twi_send_start(const uint8_t sad, uint8_t addr, uint8_t *buf, int iter);
int twi_read_bytes(const uint8_t sad, uint8_t addr, uint8_t *buf, int iter);

#endif
