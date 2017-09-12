
#ifndef PACKET_H
#define PACKET_H

struct packet {
    char magic0;
    
    int cmd;
    int data[3];
    
    char magic1;
};


struct packet read_packet();

#endif
