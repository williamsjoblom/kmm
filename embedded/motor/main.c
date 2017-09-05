#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <limits.h>

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
 * Timer interval (us).
 */
#define TIMER_INTERVAL 50

/**
 * Steps per revolution.
 */
#define STEPS_PER_REVOLUTION 800

void set_motor_speed(unsigned char motor, unsigned long speed);

/**
 * Direction.
 */
char direction = MOTOR0_BCK;

unsigned int motor0_ticks = UINT_MAX;
unsigned int motor1_ticks = UINT_MAX;
unsigned int motor2_ticks = UINT_MAX;

unsigned int motor0_remaining = 0;
unsigned int motor1_remaining = 0;
unsigned int motor2_remaining = 0;


/**
 * Main.
 */
int main() {
  // Set PORTB as output.
  DDRB = 0xFF;

  set_motor_speed(0, 100000);

  install_timer(TIMER_INTERVAL);

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
 * Set motor speed. 
 * (revolutions per second/1000)
 */
void set_motor_speed(unsigned char motor, unsigned long speed) {
  const unsigned long timer_freq = 1000000 / TIMER_INTERVAL;
  unsigned long step_freq = speed*STEPS_PER_REVOLUTION;

  unsigned int ticks = (unsigned int) ((timer_freq*1000)/step_freq);

  switch(motor) {
  case 0: motor0_ticks = ticks; break;
  case 1: motor1_ticks = ticks; break;
  case 2: motor2_ticks = ticks; break;
  }
}


/**
 * Timer interrupt.
 */
ISR(TIMER1_COMPA_vect) {
  motor0_remaining--;
  motor1_remaining--;
  motor2_remaining--;

  if (motor0_ticks != UINT_MAX) {
    motor0_remaining--;
    if (motor0_remaining == 0) {
      motor0_remaining = motor0_ticks;
      PORTB |= MOTOR0_STP;
    }
  }

  if (motor1_ticks != UINT_MAX) {
    motor1_remaining--;
    if (motor1_remaining == 0) {
      motor1_remaining = motor1_ticks;
      PORTB |= MOTOR1_STP;
    }
  }

  if (motor2_ticks != UINT_MAX) {
    motor2_remaining--;
    if (motor2_remaining == 0) {
      motor2_remaining = motor2_ticks;
      PORTB |= MOTOR2_STP;
    }
  }

  _delay_us(5);

  PORTB = direction;
}
