// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#define BUFFER_SIZE 256
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    alarmCount = 0;
    (void)signal(SIGALRM, alarmHandler);
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }
    enum State {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE};
    enum State state= START;
    unsigned char buf[5] = {0};
    unsigned char buf_[BUFFER_SIZE + 1] = {0};
    while (state != STOP_STATE && alarmCount < 4)
    {
        if (alarmEnabled == FALSE && connectionParameters.role==LlTx)
        {
            buf[0]= FLAG;
            buf[1]= ADDRESS_T;
            buf[2]= CONTROL_SET;
            buf[3]= buf[1] ^ buf[2];
            buf[4]= FLAG;
            if(writeBytes(buf, 5)) printf(" escrevi 5 bytes\n");
            alarm(3);
            alarmEnabled = TRUE;
        }
        int address, control = 0;
        if(connectionParameters.role == LlTx) {
            address = ADDRESS_R;
            control = CONTROL_UA;
        }
        else {
            address = ADDRESS_T;
            control = CONTROL_SET;
        }
        readByte((unsigned char *)buf_);
        switch (state)
            {
            case (START):
                if (buf_[0] == FLAG) {
                    state = FLAG_RCV;
                    printf("START -> FLAG_RCV\n");
                }
                break;
            case (FLAG_RCV):
                if (buf_[0] == address) {
                    state = A_RCV;
                    printf("FLAG_RCV -> A_RCV\n");
                    }
                else if (buf_[0] == FLAG) break;
                else state = START;
                break;
            case (A_RCV):
                if (buf_[0] == control) {
                    state = C_RCV;
                    printf("A_RCV -> C_RCV\n");
                    }
                else if (buf_[0] == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case (C_RCV):
                if (buf_[0] == address^control) {
                    state = BCC_OK;
                    printf("C_RCV -> BCC_OK\n");
                    }
                else if (buf_[0] == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case (BCC_OK):
                if (buf_[0] == FLAG) {
                    state = STOP_STATE;
                    alarm(0);
                    printf("BCC_OK -> STOP_STATE\n");
                    printf("Finished\n");
                    }
                else state = START;
                break;
            }
    }
    if (connectionParameters.role == LlRx) {
        buf[0]= FLAG;
        buf[1]= ADDRESS_R;
        buf[2]= CONTROL_UA;
        buf[3]= buf[1] ^ buf[2];
        buf[4]= FLAG;

        writeBytes((const unsigned char *)buf, 5);
        printf("5 bytes have been written\n");
    }
    printf("Stopped\n");

    sleep(1);

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
