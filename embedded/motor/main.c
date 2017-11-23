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
#define MOTOR2_BCK 0b10000000 // PD7

/**
 * Enable pins.
 */
#define MOTOR0_DISABLE 0b00000001 // PB0
#define MOTOR1_DISABLE 0b00000100 // PD0
#define MOTOR2_DISABLE 0b00100000 // PD5

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
 * Enable.
 */
unsigned char motor0_enable = 0;
unsigned char motor1_enable = 0;
unsigned char motor2_enable = 0;

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
    DDRB = 0b00010011;
    DDRC = 0b00111111;
    DDRD = 0b11111110;
    
    spi_slave_init();
        
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
    
    /* Disable motor drivers when all motors are stopped. */
    int disable =
	motor0_ticks == UINT_MAX &&
	motor1_ticks == UINT_MAX &&
	motor2_ticks == UINT_MAX;
	
    if (disable) {
	motor0_enable = MOTOR0_DISABLE;
	motor1_enable = MOTOR1_DISABLE;
	motor2_enable = MOTOR2_DISABLE;
    } else {
	motor0_enable = 0;
	motor1_enable = 0;
	motor2_enable = 0;
    }
    
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
	motor0_enable /* ENABLE MOTOR0 */ | motor0_step;
    PORTC =
	motor0_direction;
    
    PORTD =
	motor1_enable /* ENABLE MOTOR1 */ | motor1_step | motor1_direction |
	motor2_enable /* ENABLE MOTOR2 */ | motor2_step | motor2_direction;
    
    
    for (int i = 0; i < 20; i++) { asm("nop"); }

    
    PORTB = motor0_enable /* ENABLE MOTOR0 */;
    
    PORTD =
	motor1_enable /* ENABLE MOTOR1 */ | motor1_direction |
	motor2_enable /* ENABLE MOTOR2 */ | motor2_direction;
}
