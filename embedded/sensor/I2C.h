#ifndef _TWI_H
#define _TWI_H

    #define ACC_ADDR  0b0011001 // + R/W bit
    #define GYRO_ADDR 0b1101010 // + R/W bit
    
    /* Register adresses for each coordinate. Used in SUB.
     * ACC_X_L   0b0101000
     * ACC_X_H   0b0101001
     * ACC_Y_L   0b0101010
     * ACC_Y_H   0b0101011
     * GYRO_Z_L  0b0101100
     * GYRO_Z_H  0b0101101
     */
    #define ACC_START 0b0101000
    #define GYRO_START 0b0101100

    /*
    * Maximum no. iterations that we will wait for a selection to be
    * responded to by the slave. Used to be able to abort an infinte 
    * loop, although still being enough for a pending write to finish.
    * WILL HAVE TO BE CHANGED IF CLOCK OF TWI IS CHANGED. 
    * RN BASED ON A TWI CLOCK RATE OF 100 kHz.
    */
    #define MAX_ITER 200

int twi_read_bytes(uint8_t sad, int iter, uint8_t *buf)

#endif