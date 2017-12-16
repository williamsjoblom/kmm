/*
 * spi.h
 *
 * SPI heeader contains two functions; one for initialization of the SPI
 * and one function to write len bytes of data to the main module.
 * 
 *
 * Created: 15/11/2017
 * Author: wilsj878, hamer848, sabse455
 */

#ifndef _SPIAVR_H_
#define _SPIAVR_H_


/*
 * Initializes the spi on the ATmega128 and sets it up as a slave. 
 */
void spi_slave_init();

/*
 * Write len number of bytes to the master from the slave.
 */
void spi_slave_write(uint8_t *data, uint8_t len);

#endif
