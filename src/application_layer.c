// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer l;
    l.baudRate=baudRate;
    l.nRetransmissions=nTries;

    if(role[0]=='t') l.role=LlTx;
    else l.role=LlRx;
    strcpy(l.serialPort, serialPort);
    l.timeout=timeout;

    llopen(l);

    if(l.role==LlRx) {
        unsigned char packet[MAX_PAYLOAD_SIZE];
        int size = llread(packet);
        for(int i = 0; i < size; i++) {
            printf("%x\n ", packet[i]);
        }

    }

    else {

        //PREPARING PACKETS
        printf("preparing packets\n");
        unsigned char buf[8] = {0}; // +1: Save space for the final '\0' char
        buf[0]=0x7e;
        buf[1]=0x7d;
        buf[2]=0xbb;
        buf[3]=0xaa;
        buf[4]=0x7f;
        buf[5]=0xac;
        buf[6]=0x45;
        buf[7]=0x05;

        //SENDING PACKETS
        printf("sending packets\n");

        int bytes = llwrite( buf, 8);

        printf("%d bytes written\n", bytes);





    }
}
