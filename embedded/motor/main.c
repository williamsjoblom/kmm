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
#define TIMER_INTERVAL 125L

/**
 * Steps per revolution.
 */
#define STEPS_PER_REVOLUTION 800L


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

volatile unsigned int motor0_remaining = 0;
volatile unsigned int motor1_remaining = 0;
volatile unsigned int motor2_remaining = 0;

/**
 * Main.
 */
int main() {
    // Set PORTB as output.
    DDRB = 0xFF;

    serial_init();
    
    install_timer(TIMER_INTERVAL);
        
    while (1) {
	read_packet();
    }
    
    return 1;
}


/**
 * Set motor speed.
 * (revolutions per second/1000)
 */
void set_motor_speed(unsigned char motor, int speed) {
    unsigned int ticks = (unsigned int) ((1000L * 1000000L)/(TIMER_INTERVAL*STEPS_PER_REVOLUTION*speed));
        
    switch(motor) {
    case 0:
	motor0_remaining = ticks;
	motor0_ticks = ticks;
	motor0_direction =  speed < 0 ? MOTOR0_BCK : 0;
	break;
    case 1:
	motor1_remaining = ticks;
	motor1_ticks = ticks;
	motor1_direction = speed < 0 ? MOTOR1_BCK : 0;
	break;
    case 2:
	motor2_remaining = ticks;
	motor2_ticks = ticks;
	motor2_direction = speed < 0 ? MOTOR2_BCK : 0;
	break;
    }

    direction = motor2_direction | motor1_direction | motor0_direction;

    printf("motor %i = %i rps/1000\r\n", motor, speed);
    printf("N time quanta = %u\r\n", ticks);
    printf("dirmask = 0x%X\r\n", direction);
}



/**
 * Timer interrupt.
 */
ISR(TIMER1_COMPA_vect) {
    if (motor0_ticks != UINT_MAX) {
	motor0_remaining--;
	if (motor0_remaining == 0) {
	    motor0_remaining = motor0_ticks;
	    //PORTB |= MOTOR0_STP;
	    PORTB = 0xFF;
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

    for (int i = 0; i < 20; i++) { asm("nop"); }

    PORTB = 0x00;
    //PORTB = direction;
}
