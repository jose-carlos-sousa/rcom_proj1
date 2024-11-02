// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <sys/time.h>

#define BUFFER_SIZE 256
#define _POSIX_SOURCE 1 // POSIX compliant source
volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
int ns=0;
int nr =1;
int lastSeqNumber = -1;
int time_out=0;
int retraNum=0;
int role=0;

extern int framesSent;
extern int framesRead;
extern int originalSize;
extern int newSize;
extern int fileSize;
extern struct timeval programStart, programEnd;
extern int totalAlarms;
extern int totalRejs;
extern int totalRRs;
extern int totalDups;

int sendSFrame(unsigned int control){
    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = ADDRESS_T;
    buf[2] = control;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    return writeBytes((const char *)buf, 5);
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
    totalAlarms++;

    if (alarmCount < 3) printf("Retry #%d\n", alarmCount);
    else printf("Max retries reached\n");
}
int send_and_wait( unsigned int control_send, unsigned int control_recieve){
        alarmCount=0;                               //inicializar alarm count para 0
        while(alarmCount < retraNum){               //enquanto não exeder o numero de alarm count 
            sendSFrame(control_send);
            alarm(time_out);
            alarmEnabled = TRUE;                    // Ativamos o alarme
            State state = START;                    //Colocar no estado inicial
            char c =0;
            while(state != STOP_STATE && alarmEnabled){ //Enquanto não chegar ao estado final e o alarme não apitar
                if ( readByte((char *)&c) > 0) {
                    switch (state) {
                        case (START):
                            if (c == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case (FLAG_RCV):
                            if (c == ADDRESS_T) {
                                state = A_RCV;
                                }
                            else if (c == FLAG) break;
                            else state = START;
                            break;
                        case (A_RCV):
                            if (c == control_recieve ) {
                                state = C_RCV;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (C_RCV):
                            if (c == (ADDRESS_T ^ control_recieve)) {
                                state = BCC_OK;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (BCC_OK):
                            if (c == FLAG) {
                                state = STOP_STATE;
                                alarmEnabled=FALSE;
                                return 0;               //Posso parar se tiver sucesso
                                }
                            else state = START;
                            break;
                        default:
                            break;
                    }
                }
            }

        }
        return -1;
}

int only_wait (unsigned int control_recieve){
     State state = START;                           //Colocar no estado inicial
     char c;
     while(state != STOP_STATE ){
                if ( readByte(( char *)&c) >0) {
                    switch (state) {
                        case (START):
                            if (c == FLAG) {
                                state = FLAG_RCV;
                            }
                            break;
                        case (FLAG_RCV):
                            if (c == ADDRESS_T) {
                                state = A_RCV;
                                }
                            else if (c == FLAG) break;
                            else state = START;
                            break;
                        case (A_RCV):
                            if (c == control_recieve) {
                                state = C_RCV;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (C_RCV):
                            if (c == (ADDRESS_T ^ control_recieve)) {
                                state = BCC_OK;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (BCC_OK):
                            if (c == FLAG) {
                                state = STOP_STATE;
                                return 0;
                                }
                            else state = START;
                            break;
                        default:
                            break;
                    }
                }
            }
    return -1; //Isto não devia acontecer
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

    retraNum = connectionParameters.nRetransmissions;
    time_out = connectionParameters.timeout;
    role=connectionParameters.role;
    if(connectionParameters.role ==LlTx){
        if(send_and_wait(CONTROL_SET, CONTROL_UA)==-1)return -1;
    }
    else if (connectionParameters.role == LlRx) {
        printf("will wait   \n");
        if(only_wait(CONTROL_SET)==-1)return -1;
        if(sendSFrame(CONTROL_UA)==-1) return -1;
    }
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int checkCF(){

    unsigned char  control = 0;
    State state = START;
    unsigned char c;
    while(state != STOP_STATE && alarmEnabled ){
        if ( readByte((char *)&c) >0) {
            switch (state) {
                case (START):
                    if (c == FLAG) {
                        state = FLAG_RCV;
                    }
                    break;
                case (FLAG_RCV):
                    if (c == ADDRESS_T) {
                        state = A_RCV;
                        }
                    else if (c == FLAG) break;
                    else state = START;
                    break;
                case (A_RCV):
                    if (c == CONTROL_DISC || c == CONTROL_REJ0 || c == CONTROL_REJ1 || c == CONTROL_RR0 || c == CONTROL_RR1) {
                        state = C_RCV;
                        control = c;
                        }
                    else if (c == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case (C_RCV):
                    if (c == (ADDRESS_T ^ control)) {
                        state = BCC_OK;
                        }
                    else if (c == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case (BCC_OK):
                    if (c == FLAG) {
                        state = STOP_STATE;
                        alarm(0);
                        alarmEnabled=FALSE;
                        }
                    else state = START;
                    break;
                default:
                    break;
            }
        }
    }
    return control;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    printf("Writing %d bytes\n", bufSize);
    originalSize += bufSize; 
    newSize += bufSize;
    unsigned char infoFrame [4+ bufSize * 2 + 3]; //4 bytes para o primeiro header 2 * bufSize para a data com stuffing e 3 para a tail (stuffing bcc2)
    infoFrame[0] = FLAG;
    infoFrame[1] = ADDRESS_T;
    infoFrame[2] = ns << 7;
    infoFrame[3] = infoFrame[1] ^ infoFrame[2];
    int index = 4;
    unsigned char bcc2 = 0;

    for ( int i = 0 ; i < bufSize ; i++ ){ //meter a info no frame com stuffing e calcular o bcc2
        if ( buf[i] == FLAG){
            infoFrame[index++] = ESCAPE;
            infoFrame[index++] = MOD_FLAG;
            newSize++;
        }else if ( buf[i] == ESCAPE){
            infoFrame[index++] = ESCAPE;
            infoFrame[index++] = MOD_ESCAPE;
            newSize++;
        }else {
            infoFrame[index++] = buf[i];
        }
        bcc2 ^= buf[i];
    }
    if( bcc2== FLAG){
        infoFrame[index++]=ESCAPE;
        infoFrame[index]=MOD_FLAG;
    }else if(bcc2 == ESCAPE) {
        infoFrame[index++]=ESCAPE;
        infoFrame[index]=MOD_ESCAPE;
    }else {
        infoFrame[index]=bcc2;
    }
    infoFrame[++index]=FLAG;
    alarmCount = 0;
    alarmEnabled=FALSE;
    while(alarmCount < retraNum){ // vou tentar escrever x vezes
            if(alarmEnabled == FALSE){
                int res = writeBytes((const char *)infoFrame, index+1); //escrevo
                framesSent++;
                alarmEnabled=TRUE; //ativo alarme
                alarm(time_out);
                if( res == -1){
                    printf("failed to write bytes\n");
                }
            }
            int control = checkCF(); // vou ver se receiver recebeu bem
            if(control == CONTROL_REJ0 || control == CONTROL_REJ1) {
                totalRejs++;
                printf("Frame rejected :(\n");
                alarmCount=0;
            }
            else if((((control == CONTROL_RR0) && (ns == 1)) )|| ((control == CONTROL_RR1) && (ns == 0))) {
                totalRRs++;
                printf("Frame accepted\n");
                alarmCount=0;
                ns = (ns+1) % 2; //alterno o ns para o proximo frame
                return 0;
            }
    }
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) // TO CHANGE IN THE FUTURE
{
    unsigned char c, control = 0;
    int i = 0;
    State state = START;

    while (state != STOP_STATE) {  
        if (readByte((char *)&c) > 0) {
            switch (state) {
                case START:
                    if (c == FLAG) {
                        state = FLAG_RCV;
                    } else {
                    }
                    break;

                case FLAG_RCV:
                    if (c == ADDRESS_T) {
                        state = A_RCV;
                    } else if (c != FLAG) {
                        state = START;
                    } else {
                    }
                    break;

                case A_RCV:
                    if (c == 0 || c == (1 << 7)) {
                        state = C_RCV;
                        control = c;
                        int seqNumber = (control >> 7) & 0x01;
                        if (seqNumber == lastSeqNumber) {
                            printf("Received frame with same sequence number\n");
                            if (sendSFrame(!lastSeqNumber ? CONTROL_RR1 : CONTROL_RR0)) printf("Sent RR response (duplicate)\n");
                            totalDups++;
                            framesRead++;
                            return 0;
                        }
                    } else if (c == CONTROL_SET){ // este é caso em que houve erro no UA
                        sendSFrame(CONTROL_UA);
                        state = START;
                    } else if (c == CONTROL_DISC){ // o transmissor fechou a ligação
                        if (sendSFrame(CONTROL_DISC)) printf("Reading stopped abruptly, transmissor is closing the connection (what)\n");
                        return -1; 
                    } else if (c == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;

                case C_RCV:
                    if (c == (ADDRESS_T ^ control)) {
                        state = READING_DATA;
                    } else if (c == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        state = START;
                    }
                    break;

                case READING_DATA:
                    if (c == ESCAPE) {
                        state = DATA_FOUND_ESC;
                    } else if (c == FLAG) {
                        unsigned char bcc2 = packet[i - 1];
                        i--;
                        packet[i] = '\0';
                        unsigned char acc = packet[0];

                        for (unsigned int j = 1; j < i; j++)
                            acc ^= packet[j];
                        if (bcc2 == acc) {
                            state = STOP_STATE;
                            if (sendSFrame(nr ? CONTROL_RR1 : CONTROL_RR0)) printf("Sent RR response\n");
                            totalRRs++;
                            framesRead++;

                            lastSeqNumber = (control >> 7) & 0x01;
                            nr = (nr + 1) % 2;
                            return i; 
                        } else {
                            printf("Error: retransmission required\n");
                            if (sendSFrame(nr ? CONTROL_REJ0 : CONTROL_REJ1)) printf("Sent REJ response\n");
                            totalRejs++;
                            framesRead++;

                            state = START;
                            bzero(packet,i);
                            i=0;
                        }
                    } else {
                        packet[i++] = c;
                    }
                    break;

                case DATA_FOUND_ESC:
                    state = READING_DATA;
                    if (c == MOD_ESCAPE || c == MOD_FLAG) {
                        packet[i++] = c == MOD_ESCAPE ? ESCAPE : FLAG;
                    } else {
                       packet[i++] = c; //este erro vai ser detetado pelo bcc2
                    }
                    break;
                default:
                    printf("Unknown state encountered\n");
                    return -1;
            }
        }
    }

    printf("Exiting llread function\n"); //isto não devia acontecer
    return 0;
}


////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{

    if (role==LlTx){ //se for o transmissor
        if(send_and_wait(CONTROL_DISC,CONTROL_DISC)==-1)return -1;
        sendSFrame(CONTROL_UA);

    }else if(role==LlRx){ //se for o recetor
            if(only_wait(CONTROL_DISC)==-1) return -1;
            if(send_and_wait(CONTROL_DISC,CONTROL_UA)==-1) return -1;
    }

    int clstat = closeSerialPort();
    if (clstat != -1) printf("Connection terminated successfully.\n");

    if (showStatistics) {
        printf("\n\nStatistics:\n\n");

        if (role == LlRx)
            printf("Role: Receiver \n");
        else
            printf("Role: Transmiter \n");

        if (role == LlTx)
        {
            printf("Frames sent: %d\n", framesSent);
            printf("File size: %d bytes\n", fileSize);
            printf("Original data size: %d bytes\n", originalSize);
            printf("Data size w/ stuffing: %d bytes\n", newSize);
            printf("Number of bytes stuffed: %d\n", newSize - originalSize);
            printf("Average data size per frame: %g bytes\n", (double)originalSize / totalRRs);
            printf("Average data size per frame w/ stuffing: %g bytes\n", (double)newSize / totalRRs);
            printf("Total number of timeouts: %d\n", totalAlarms);
        }
        else
        {
            printf("Frames read: %d\n", framesRead);
            printf("Number of accepted frames: %d\n", totalRRs);
            printf("Number of rejected frames: %d\n", totalRejs);
            printf("Number of duplicate frames: %d\n", totalDups);
        }

        double elapsed_time = (programEnd.tv_sec - programStart.tv_sec) + (programEnd.tv_usec - programStart.tv_usec) / 1e6;
        printf("Total time: %g seconds\n", elapsed_time);
    }

    return clstat;
}


