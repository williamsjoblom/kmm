
#include "serial.h"
#include "packet.h"

#define MAGIC0 42
#define MAGIC1 24

int verify_packet(struct packet* p) {
    if (p->magic0 != MAGIC0 && p->magic1 != MAGIC1) return 1;
    return 0;
}


struct packet read_packet() {
    struct packet p;
    
    for (unsigned int i = 0; i < sizeof(struct packet); i++) {
	*((unsigned char*) ((void*) &p + i)) = serial_read();
    }

    if (verify_packet(&p))
	printf("Bad packet signature!\n\r");
    else
	printf("Packet signature OK!\n\r");

    printf("command = %i\r\n", p.cmd);
        
    for (int i = 0; i < 3; i++) {
	printf("data[%i] = %i\r\n", i, p.data[i]);
    }

    return p;
}
