// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

#define BUFFER_SIZE 256
// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int ns=0;
int nr =1;
int time_out=0;
int retraNum=0;
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
    State state = START;
    unsigned char buf[5] = {0};
    unsigned char buf_[BUFFER_SIZE + 1] = {0};
    retraNum= connectionParameters.nRetransmissions;
    time_out= connectionParameters.timeout;
    while (state != STOP_STATE && alarmCount < retraNum)
    {
        if (alarmEnabled == FALSE && connectionParameters.role == LlTx)
        {
            buf[0] = FLAG;
            buf[1] = ADDRESS_T;
            buf[2] = CONTROL_SET;
            buf[3] = buf[1] ^ buf[2];
            buf[4] = FLAG;
            if(writeBytes(buf, 5) != -1) printf(" escrevi 5 bytes\n");
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
        int r =readByte((unsigned char *)buf_);
        if( r > 0){
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
                    printf("BCC_OK -> STOP_STATE\n");
                    printf("Finished\n");
                    alarm(0);
                    }
                else state = START;
                break;
            }
    }
        }
        
    if (connectionParameters.role == LlRx) {
        buf[0] = FLAG;
        buf[1] = ADDRESS_R;
        buf[2] = CONTROL_UA;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        writeBytes((const unsigned char *)buf, 5);
        printf("5 bytes have been written\n");
    }
    printf("Stopped\n");
    if(alarmCount == retraNum ) return -1;
    sleep(1);

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int checkCF(){

    unsigned char byte, control = 0;
    State state = START;
    unsigned char c;
    while(state != STOP_STATE ){
        if ( readByte(&c) >0) {
            printf("Checking frame\n");
            switch (state) {
                case (START):
                    if (c == FLAG) {
                        state = FLAG_RCV;
                        printf("START -> FLAG_RCV\n");
                    }
                    break;
                case (FLAG_RCV):
                    if (c == ADDRESS_R) {
                        state = A_RCV;
                        printf("FLAG_RCV -> A_RCV\n");
                        }
                    else if (c == FLAG) break;
                    else state = START;
                    break;
                case (A_RCV):
                    if (c == CONTROL_DISC || c == CONTROL_REJ0 || c == CONTROL_REJ1 || c == CONTROL_RR0 || c == CONTROL_RR1) {
                        state = C_RCV;
                        control = c;
                        printf("A_RCV -> C_RCV\n");
                        }
                    else if (c == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case (C_RCV):
                    if (c == ADDRESS_R ^ control) {
                        state = BCC_OK;
                        printf("C_RCV -> BCC_OK\n");
                        }
                    else if (c == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case (BCC_OK):
                    if (c == FLAG) {
                        state = STOP_STATE;
                        printf("BCC_OK -> STOP_STATE\n");
                        printf("Finished checking\n");
                        }
                    else state = START;
                    break;
            }
        }
    }
    return control;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char *infoFrame =(unsigned char *)malloc(4+ bufSize * 2 + 2); //4 BYTES FOR FIRST HEADER THE THE FRAME TIMES 2 TO ACCOUNT FOR STUFFING THEN 2 FOR HEADER 2
    infoFrame[0] = FLAG;
    infoFrame[1] = ADDRESS_T;
    infoFrame[2] = ns << 6;
    infoFrame[3] = infoFrame[1] ^ infoFrame[2];
    int index = 4;
    unsigned char bcc2 = 0;

    for ( int i = 0 ; i < bufSize ; i++ ){
        if ( buf[i] == FLAG){
            infoFrame[index++] = ESCAPE;
            infoFrame[index++] = MOD_FLAG;
        }else if ( buf[i] == ESCAPE){
            infoFrame[index++] = ESCAPE;
            infoFrame[index++] = MOD_ESCAPE;
        }else {
            infoFrame[index++] = buf[i];
        }
        bcc2 ^= buf[i];
    }
    infoFrame[index]=bcc2;
    infoFrame[++index]=FLAG;
    int tries = 0;
    int acpt = 0;
    int rej = 0;
    alarmCount = 0;
    printf("num of retries is %d\n",retraNum);
    printf("tries %d\n",tries);
    alarmEnabled=FALSE;
    while(alarmCount < retraNum){
        rej = acpt = 0;
      
        while (!alarmEnabled){
            printf("alarm not enabled \n");
            int res = writeBytes(infoFrame, index+1);
            alarmEnabled=TRUE;
            alarm(3);
            printf(" res is %d", res);
            printf("wrote \n");
            printf("gave time out \n");
            if( res == -1){
                printf(" it failed :(\n");
            }
            int control = checkCF();
            printf("control is %x\n", control);
            if(!control){ //CHANGE IN FUTURE PLS
                printf("not control\n");
                continue;
            }
            else if(control == CONTROL_REJ0 || control == CONTROL_REJ1) {
                printf("rej\n");
                rej = 1;
                break;
            }
            else if(control == CONTROL_RR0 || control == CONTROL_RR1) {
                printf("accept\n");
                acpt = 1;
                ns = (ns+1) % 2;
                break;
            }
            else continue;
        }
        if (acpt) break;
        tries++;
    }

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // TO CHANGE IN THE FUTURE
{
    unsigned char c, control = 0;
    int i = 0;
    State state = START;
    unsigned char buf[5] = {0};

    printf("Starting llread...\n");

    while (state != STOP_STATE) {  
        if (readByte(&c) > 0) {
            printf("Read byte: %x\n", c);
            switch (state) {
                case START:
                    printf("State: START\n");
                    if (c == FLAG) {
                        state = FLAG_RCV;
                        printf("Transition to FLAG_RCV\n");
                    } else {
                        printf("Received byte is not FLAG\n");
                    }
                    break;

                case FLAG_RCV:
                    printf("State: FLAG_RCV\n");
                    if (c == ADDRESS_T) {
                        state = A_RCV;
                        printf("Transition to A_RCV\n");
                    } else if (c != FLAG) {
                        state = START;
                        printf("Received byte is not ADDRESS_T, resetting to START\n");
                    } else {
                        printf("Received byte is FLAG, staying in FLAG_RCV\n");
                    }
                    break;

                case A_RCV:
                    printf("State: A_RCV\n");
                    if (c == 0 || c == (1 << 6)) {
                        state = C_RCV;
                        control = c;
                        printf("Transition to C_RCV with control: %x\n", control);
                    } else if (c == FLAG) {
                        state = FLAG_RCV;
                        printf("Received FLAG, transitioning back to FLAG_RCV\n");
                    } else if (c == CONTROL_DISC) {
                        printf("Received DISC control\n");
                        buf[0] = FLAG;
                        buf[1] = ADDRESS_R;
                        buf[2] = CONTROL_DISC;
                        buf[3] = buf[1] ^ buf[2];
                        buf[4] = FLAG;

                        if (writeBytes(buf, 5)) printf("Sent 5 bytes for DISC response\n");
                        printf("Disconnected\n");
                        alarm(3);
                        alarmEnabled = TRUE;
                        return 0;
                    } else {
                        state = START;
                        printf("Invalid control byte, resetting to START\n");
                    }
                    break;

                case C_RCV:
                    printf("State: C_RCV\n");
                    if (c == (ADDRESS_T ^ control)) {
                        state = READING_DATA;
                        printf("Transition to READING_DATA\n");
                    } else if (c == FLAG) {
                        state = FLAG_RCV;
                        printf("Received FLAG, transitioning back to FLAG_RCV\n");
                    } else {
                        state = START;
                        printf("Invalid control, resetting to START\n");
                    }
                    break;

                case READING_DATA:
                    printf("State: READING_DATA\n");
                    if (c == ESCAPE) {
                        state = DATA_FOUND_ESC;
                        printf("Transition to DATA_FOUND_ESC\n");
                    } else if (c == FLAG) {
                        unsigned char bcc2 = packet[i - 1];
                        printf("Received FLAG, checking BCC\n");
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];

                        printf("BCC2: %x, calculated acc: %x\n", bcc2, acc);
                        if (bcc2 == acc) {
                            state = STOP_STATE;
                            printf("BCC check successful, transitioning to STOP_STATE\n");
                            buf[0] = FLAG;
                            buf[1] = ADDRESS_R;
                            buf[2] = nr ? CONTROL_RR1 : CONTROL_RR0;
                            buf[3] = buf[1] ^ buf[2];
                            buf[4] = FLAG;

                            if (writeBytes(buf, 5)) printf("Sent 5 bytes for RR response\n");
                            printf("Data read successfully.\n");
                            alarm(3);
                            alarmEnabled = TRUE;
                            nr = (nr + 1) % 2;
                            return i; 
                        } else {
                            printf("Error: retransmission required\n");
                            buf[0] = FLAG;
                            buf[1] = ADDRESS_R;
                            buf[2] = nr ? CONTROL_REJ1 : CONTROL_REJ0;
                            buf[3] = buf[1] ^ buf[2];
                            buf[4] = FLAG;

                            if (writeBytes(buf, 5)) printf("Sent 5 bytes for REJ response\n");
                            return -1;
                        }
                    } else {
                        packet[i++] = c;
                        printf("Appending data byte: %x\n", c);
                    }
                    break;

                case DATA_FOUND_ESC:
                    printf("State: DATA_FOUND_ESC\n");
                    state = READING_DATA;
                    if (c == ESCAPE || c == FLAG) {
                        packet[i++] = c;
                        printf("Appending unescaped byte: %x\n", c);
                    } else {
                        packet[i++] = ESCAPE;
                        packet[i++] = c;
                        printf("Escaped byte: %x\n", c);
                    }
                    break;

                default: 
                    printf("Unknown state encountered\n");
                    break;
            }
        }
    }

    printf("Exiting llread function\n");
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
