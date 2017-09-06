
#include <avr/io.h>
#include <avr/interrupt.h>

#include "timer.h"

/**
 * Timer resolution.
 */
#define TIMER_RESOLUTION 65536UL

/**
 * Timer period.
 */
unsigned short timer_period = 0;


/**
 * Install timer. 
 */
void install_timer(unsigned long us) {
    const unsigned long cycles = (F_CPU / 1000000) * us;

    unsigned char clock_select = _BV(CS11);  
  
    if (cycles < TIMER_RESOLUTION) {
	clock_select = _BV(CS10);
	timer_period = cycles;
    } else if (cycles < TIMER_RESOLUTION*8) {
	clock_select = _BV(CS11);
	timer_period = cycles/8;
    } else if (cycles < TIMER_RESOLUTION*64) {
	clock_select = _BV(CS11) | _BV(CS10);
	timer_period = cycles/64;
    } else if (cycles < TIMER_RESOLUTION*256) {
	clock_select = _BV(CS12);
	timer_period = cycles/256;
    } else if (cycles < TIMER_RESOLUTION*1024) {
	clock_select = _BV(CS12) | _BV(CS10);
	timer_period = cycles/1024;
    } else {
	clock_select = _BV(CS12) | _BV(CS10);
	timer_period = TIMER_RESOLUTION - 1;
    }
    
    ICR1 = timer_period;
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = _BV(WGM13) | _BV(WGM12) | clock_select;
    TIMSK1 = _BV(OCIE1A);

    sei();
}
