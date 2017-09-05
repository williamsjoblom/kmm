#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer.h"
#include "serial.h"

/**
 * Step pins.
 */
#define MOTOR0_STP 0b00000001
#define MOTOR1_STP 0b00000010
#define MOTOR2_STP 0b00000100

/**
 * Direction pins.
 */
#define MOTOR0_BCK 0b00001000
#define MOTOR1_BCK 0b00010000
#define MOTOR2_BCK 0b00100000

/**
 * Direction.
 */
char direction = MOTOR0_BCK;

unsigned int motor0_ticks = 100;
unsigned int motor1_ticks = 200;
unsigned int motor2_ticks = 1000000;

unsigned int motor0_remaining = 0;
unsigned int motor1_remaining = 0;
unsigned int motor2_remaining = 0;


/**
 * Main.
 */
int main (void) {
  // Set PORTB as output.
  DDRB = 0xFF;

  motor0_remaining = motor0_ticks;
  motor1_remaining = motor1_ticks;
  motor2_remaining = motor2_ticks;

  serial_init();

  install_timer(10);

  while(1) {
    if (serial_rx_available()) {
  		PORTB |= _BV(1); // Turn on LED @ PB1
  		int in_byte = serial_read();
      serial_write(in_byte);
  		PORTB &= 253U; // Turn off LED
  	}
  }

  return 1;
}


/**
 * Timer interrupt.
 */
ISR(TIMER1_COMPA_vect) {
  motor0_remaining--;
  motor1_remaining--;
  motor2_remaining--;

  unsigned char step = direction;

  if (motor0_remaining == 0) {
    motor0_remaining = motor0_ticks;
    step |= MOTOR0_STP;
  }

  if (motor1_remaining == 0) {
    motor1_remaining = motor1_ticks;
    step |= MOTOR1_STP;
  }

  if (motor2_remaining == 0) {
    motor2_remaining = motor2_ticks;
    //step |= MOTOR2_STP;
  }

  PORTB = step;

  _delay_us(5);

  PORTB = direction;
}
