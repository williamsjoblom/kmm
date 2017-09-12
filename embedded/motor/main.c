#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <limits.h>

#include "timer.h"
#include "serial.h"
#include "packet.h"

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
#define TIMER_INTERVAL 125

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

    serial_init();
    
    //set_motor_speed(0, 2000);

    //install_timer(TIMER_INTERVAL);

    //printf("Hello printf!\n\r");
    
    while (1) { read_packet(); }
    
    /* while (1) { */
    /* 	read_packet(); */
    /* } */
    
    return 1;
}


/**
 * Set motor speed.
 * (revolutions per second/1000)
 */
void set_motor_speed(unsigned char motor, unsigned long speed) {
    const unsigned long timer_freq = 1000000 / TIMER_INTERVAL;
    unsigned long step_freq = speed*STEPS_PER_REVOLUTION;

    unsigned int ticks = (unsigned int) ((timer_freq*1000)/step_freq*2);
    
    switch(motor) {
    case 0: motor0_ticks = ticks; break;
    case 1: motor1_ticks = ticks; break;
    case 2: motor2_ticks = ticks; break;
    }

    serial_write_str("speed ");
    serial_write_uint(speed);
    serial_write_str(" => ticks ");
    serial_write_uint(ticks);
    serial_write_str("\n\r");
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

    _delay_us(20);

    PORTB = direction;
}
