#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>

extern "C" {
#include "include/clk.h"
#include "include/gpio.h"
#include "include/dma.h"
#include "include/pwm.h"
#include "include/version.h"
#include "include/ws2811.h"
}

#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                12
#define DMA                     5
#define STRIP_TYPE              WS2811_STRIP_BRG
#define LED_COUNT               18
#define STRIP_COUNT             3

ws2811_led_t colors[] =
{
    0x00FF0000,  // red
    0x00FF7F00,  // orange
    0x00FFFF00,  // yellow
    0x0000FF00,  // green
    0x0000FFFF,  // lightblue
    0x000000FF,  // blue
    0x007F0017F,  // purple
    0x00FF007F,  // pink
};

bool running = true;

ws2811_t ledstring;

ws2811_led_t strip[LED_COUNT];
ws2811_channel_t ch[2];

ws2811_return_t init_ledstring() {
    ledstring.freq = TARGET_FREQ;
    ledstring.dmanum = DMA;

    ledstring.channel[0].gpionum = GPIO_PIN;
    ledstring.channel[0].count = LED_COUNT;
    ledstring.channel[0].invert = 0;
    ledstring.channel[0].brightness = 255;
    ledstring.channel[0].strip_type = STRIP_TYPE;
    
    ledstring.channel[1].gpionum = 0;
    ledstring.channel[1].count = 0;
    ledstring.channel[1].invert = 0;
    ledstring.channel[1].brightness = 0;

    return ws2811_init(&ledstring);
}


static void ctrl_c_handler(int signum)
{
    (void)(signum);
    running = false;
}

static void setup_handlers(void)
{
    struct sigaction sa;
    sa.sa_handler = ctrl_c_handler;
       
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}


void strip_clear() {
    for (int i = 0; i < LED_COUNT; i++) {
	strip[i] = 0;
    }
}

void strip_animate_spin() {
    static int c = 0;
    static int last_index = -1;
    
    for (int i = 0; i < LED_COUNT; i++) {
	if (i == last_index + 1) {
	    strip[i] = colors[c];
	} else {
	    strip[i] = 0;
	}
    }

    if (last_index == LED_COUNT - 1)
	last_index = -1;
    else
	last_index++;

    c = (c + 1) % 8;
}

void strip_animate_cycle() {
    static int c = 0;
    static int last_index = -1;
    
    for (int i = 0; i < LED_COUNT; i++) {
	strip[i] = colors[c];
    }

    c = (c + 1) % 8;
}

void strip_animate_knight() {
    static int current = 0;
    static bool forward = true;

    strip_clear();

    for (int s = 0; s < STRIP_COUNT; s++) {
	int i = current + s * (LED_COUNT / STRIP_COUNT);
	strip[i] = 0xFF0000;
    }

    if (forward) {
	if (current + 1 >= (LED_COUNT / STRIP_COUNT)) {
	    forward = false;
	    current--;
	} else {
	    current++;
	}
    } else {
	if (current - 1 <= 0) {
	    forward = true;
	    current++;
	} else {
	    current--;
	}
    }
}

void strip_animate() {
    static bool state = true;
    
    for (int i = 0; i < LED_COUNT; i++) {
	if (state == true)
	    strip[i] = 0x00FFFFFF;
	else
	    strip[i] = 0;
    }

    state = !state;
}

void strip_render()
{
    for (int i = 0; i < LED_COUNT; i++) {
	ledstring.channel[0].leds[i] = strip[i];
    }
}

int main() {
    setup_handlers();
    
    ws2811_return_t ret;
    
    std::cout << "Initializing LEDs on " << RPI_PWM_CHANNELS << " PWM channels" << std::endl;
    
    ret = init_ledstring();
    if (ret != WS2811_SUCCESS) {
	std::cout << "Initialization error: " << ws2811_get_return_t_str(ret);
	return 1;
    }

    std::cout << "Initialization OK!" << std::endl;
    std::cout << "Starting render loop" << std::endl;

    int frequency = 20;
    int period = 1000000 / frequency;

    int periods_per_effect = frequency * 20; // 20s
    int elapsed_periods = 0;
    int effect = 0; 
    
    while(running) {
	switch (effect) {
	case 0: strip_animate_cycle(); break;
	case 1: strip_animate(); break;
	case 2: strip_animate_knight(); break;
	case 3: strip_animate_spin(); break;
	}
	
	
	strip_render();
	
	ret = ws2811_render(&ledstring);
	if (ret != WS2811_SUCCESS) {
	    std::cout << "Render error: " << ws2811_get_return_t_str(ret);
	    return 1;
	}

	usleep(period);

	elapsed_periods++;
	if (elapsed_periods >= periods_per_effect) {
	    effect++; effect %= 4;
	    elapsed_periods = 0;
	}
    }

    strip_clear();
    strip_render();
    ws2811_render(&ledstring);
    
    ws2811_fini(&ledstring);
}
