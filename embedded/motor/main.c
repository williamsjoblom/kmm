#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <limits.h>

#include "main.h"
#include "timer.h"
#include "serial.h"
#include "packet.h"
#include "spi.h"


/**
 * Step pins.
 */
#define MOTOR0_STP 0b00000010 // PB1
#define MOTOR1_STP 0b00001000 // PD3
#define MOTOR2_STP 0b01000000 // PD6

/**
 * Direction pins.
 */
#define MOTOR0_BCK 0b00000001 // PC0
#define MOTOR1_BCK 0b00010000 // PD4
#define MOTOR2_BCK 0b01000000 // PD7

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

/**
 * Remaining ticks until next step.
 */
volatile unsigned int motor0_remaining = 0;
volatile unsigned int motor1_remaining = 0;
volatile unsigned int motor2_remaining = 0;


/**
 * Main.
 */
int main() {
    DDRB = 0b11001000;
    DDRD = 0b01111111;
    DDRC = 0b11111100;
    
    spi_slave_init();
        
    install_timer(TIMER_INTERVAL);

    set_motor_speed(0, 250);
    set_motor_speed(1, 500);
    set_motor_speed(2, 1000);
    
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
    unsigned int ticks = (unsigned int) ((1000L * 1000000L)/(TIMER_INTERVAL*STEPS_PER_REVOLUTION*abs(speed)));
        
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

#if ENABLE_TRACES
    printf("motor %i = %i rps/1000\r\n", motor, speed);
    printf("N time quanta = %u\r\n", ticks);
    printf("dirmask = 0x%X\r\n", direction);
#endif
}



/**
 * Timer interrupt.
 */
ISR(TIMER1_COMPA_vect) {
    int motor0_step = 0;
    int motor1_step = 0;
    int motor2_step = 0;
    
    if (motor0_ticks != UINT_MAX) {
	motor0_remaining--;
	if (motor0_remaining == 0) {
	    motor0_remaining = motor0_ticks;
	    motor0_step = MOTOR0_STP;
	}
    }

    if (motor1_ticks != UINT_MAX) {
	motor1_remaining--;
	if (motor1_remaining == 0) {
	    motor1_remaining = motor1_ticks;
	    motor1_step = MOTOR1_STP;
	}
    }

    if (motor2_ticks != UINT_MAX) {
	motor2_remaining--;
	if (motor2_remaining == 0) {
	    motor2_remaining = motor2_ticks;
	    motor2_step = MOTOR2_STP;
	}
    }

    PORTB =
	0b00000000 /* ENABLE MOTOR0 */ | motor0_step;
    PORTC =
	motor0_direction;
    
    PORTD =
	0b00000000 /* ENABLE MOTOR1 */ | motor1_step | motor1_direction |
	0b00000000 /* ENABLE MOTOR2 */ | motor2_step | motor2_direction;
    
    
    for (int i = 0; i < 20; i++) { asm("nop"); }

    
    PORTB = 0b00000000 /* ENABLE MOTOR0 */;
    
    PORTD =
	0b00000000 /* ENABLE MOTOR1 */ | motor1_direction |
	0b00000000 /* ENABLE MOTOR2 */ | motor2_direction;
}
