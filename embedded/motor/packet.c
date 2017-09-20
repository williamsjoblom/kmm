
#include "serial.h"
#include "packet.h"
#include "main.h"

#define MAGIC0 42
#define MAGIC1 24

#define END_OF_TRANSMISSION 4

/**
 * Verify packet signature.
 */
int verify_packet(struct packet* p) {
    if (p->magic0 != MAGIC0 && p->magic1 != MAGIC1) return 1;
    return 0;
}


/**
 * Read packet from serial.
 */
struct packet read_packet() {
    struct packet p;
    
    for (unsigned int i = 0; i < sizeof(struct packet); i++) {
	*((unsigned char*) ((void*) &p + i)) = serial_read();
    }

    if (verify_packet(&p)) {
	//printf("Bad packet!\n\r");
    } else {
	//printf("Packet OK!\n\r");

	set_motor_speed(0, p.data[0]);
	set_motor_speed(1, p.data[1]);
	set_motor_speed(2, p.data[2]);
    }

    return p;
}
