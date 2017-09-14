#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <limits.h>

#include "main.h"
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


/**
 * Direction.
 */
unsigned char direction = 0;

unsigned char motor0_direction = 0;
unsigned char motor1_direction = 0;
unsigned char motor2_direction = 0;

/**
 * Speed.
 */
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
    
    install_timer(TIMER_INTERVAL);
        
    while (1) { read_packet(); }
    
    return 1;
}


/**
 * Set motor speed.
 * (revolutions per second/1000)
 */
void set_motor_speed(unsigned char motor, int speed) {
    const unsigned long timer_freq = 1000000 / TIMER_INTERVAL;
    unsigned long step_freq = abs(speed)*STEPS_PER_REVOLUTION;

    unsigned int ticks = (unsigned int) ((timer_freq*1000)/step_freq*2);
    
    switch(motor) {
    case 0:
	motor0_ticks = ticks;
	motor0_direction =  speed < 0 ? 1 : 0;
	break;
    case 1:
	motor1_ticks = ticks;
	motor1_direction = speed < 0 ? 1 : 0;
	break;
    case 2:
	motor2_ticks = ticks;
	motor2_direction = speed < 0 ? 1 : 0;
	break;
    }

    direction = (motor2_direction << 2) | (motor1_direction << 1) | motor0_direction;

    printf("motor %i = %i rps/1000\r\n", motor, speed);
    printf("dirmask = 0x%X\r\n", direction);
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
