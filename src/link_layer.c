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
int role=0;
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
    role=connectionParameters.role;
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
            address = ADDRESS_T;
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
                }
                break;
            case (FLAG_RCV):
                if (buf_[0] == address) {
                    state = A_RCV;
                    }
                else if (buf_[0] == FLAG) break;
                else state = START;
                break;
            case (A_RCV):
                if (buf_[0] == control) {
                    state = C_RCV;
                    }
                else if (buf_[0] == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case (C_RCV):
                if (buf_[0] == address^control) {
                    state = BCC_OK;
                    }
                else if (buf_[0] == FLAG) state = FLAG_RCV;
                else state = START;
                break;
            case (BCC_OK):
                if (buf_[0] == FLAG) {
                    state = STOP_STATE;
                    alarm(0);
                    }
                else state = START;
                break;
            }
    }
        }
        
    if (connectionParameters.role == LlRx) {
        buf[0] = FLAG;
        buf[1] = ADDRESS_T;
        buf[2] = CONTROL_UA;
        buf[3] = buf[1] ^ buf[2];
        buf[4] = FLAG;

        writeBytes((const unsigned char *)buf, 5);
    }
    if(alarmCount == retraNum ) return -1;
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////

int checkCF(){

    unsigned char byte, control = 0;
    State state = START;
    unsigned char c;
    while(state != STOP_STATE && alarmEnabled ){
        if ( readByte(&c) >0) {
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
                    if (c == ADDRESS_T ^ control) {
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
            }
        }
    }
    return control;
}

int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char infoFrame [4+ bufSize * 2 + 2]; //4 BYTES FOR FIRST HEADER THE THE FRAME TIMES 2 TO ACCOUNT FOR STUFFING THEN 2 FOR HEADER 2
    infoFrame[0] = FLAG;
    infoFrame[1] = ADDRESS_T;
    infoFrame[2] = ns << 6;
    infoFrame[3] = infoFrame[1] ^ infoFrame[2];
    int index = 4;
    unsigned char bcc2 = 0;

    for ( int i = 0 ; i < bufSize ; i++ ){ //meter a info no frame tudo direitinho com stuffing e tudo
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
    int acpt = 0;
    int rej = 0;
    alarmCount = 0;
    alarmEnabled=FALSE;
    while(alarmCount < retraNum ){ // VOU TENTAR ESCREVER X VEZES
            if(alarmEnabled == FALSE){
                int res = writeBytes(infoFrame, index+1); //ESCREVO
                alarmEnabled=TRUE; //ATIVO ALARME
                alarm(time_out);
                if( res == -1){
                    printf(" it failed :(\n");
                }
            }
            int control = checkCF(); // VOU VER SE O GAJO DISSE QUE RECEBEU
            if(!control){ //N DISSE NADA DE JEITO O GAJO
                printf("not control\n");
            }
            else if(control == CONTROL_REJ0 || control == CONTROL_REJ1) {
                printf("rej\n");
                rej = 1; //rejeitou :( proxima tentativa
            }
            else if(control == CONTROL_RR0 || control == CONTROL_RR1) {
                printf("accept\n");
                acpt = 1;
                ns = (ns+1) % 2; //meto que agr vou para o outro
                return 0; //bazo
            }
    }
    if(acpt ==0) return -1;

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
            printf("bytes is %x\n",c);
            switch (state) {
                case START:
                    if (c == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        printf("Received byte is not FLAG\n");
                    }
                    break;

                case FLAG_RCV:
                    if (c == ADDRESS_T) {
                        state = A_RCV;
                    } else if (c != FLAG) {
                        state = START;
                    } else {
                        printf("Received byte is FLAG, staying in FLAG_RCV\n");
                    }
                    break;

                case A_RCV:
                    if (c == 0 || c == (1 << 6)) {
                        state = C_RCV;
                        control = c;
                    } else if (c == FLAG) {
                        state = FLAG_RCV;
                    } else if (c == CONTROL_DISC) {
                        buf[0] = FLAG;
                        buf[1] = ADDRESS_T;
                        buf[2] = CONTROL_DISC;
                        buf[3] = buf[1] ^ buf[2];
                        buf[4] = FLAG;

                        if (writeBytes(buf, 5)) printf("Sent 5 bytes for DISC response\n");
                        return 0;
                    }else if (c == CONTROL_SET){ // ESTE E O CASO EM QUE HOUVE ERRO NO MEU UA
                        //NESTE CASO O QUE FAÇO E VOLTO A MANDAR E VOU ESTADO INICIAL
                        buf[0] = FLAG;
                        buf[1] = ADDRESS_T;
                        buf[2] = CONTROL_UA;
                        buf[3] = buf[1] ^ buf[2];
                        buf[4] = FLAG;

                        writeBytes((const unsigned char *)buf, 5);
                        state = START;
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
                        if( bcc2 == FLAG || acc == FLAG){
                            printf("bcc2 é %x\n",bcc2);
                            printf("acc é %x\n",acc);
                        }
                        if (bcc2 == acc) {
                            state = STOP_STATE;
                            buf[0] = FLAG;
                            buf[1] = ADDRESS_T;
                            buf[2] = nr ? CONTROL_RR1 : CONTROL_RR0;
                            buf[3] = buf[1] ^ buf[2];
                            buf[4] = FLAG;

                            if (writeBytes(buf, 5)) printf("Sent 5 bytes for RR response\n");
                            nr = (nr + 1) % 2;
                            return i; 
                        } else {
                            printf("Error: retransmission required\n");
                            buf[0] = FLAG;
                            buf[1] = ADDRESS_T;
                            buf[2] = nr ? CONTROL_REJ1 : CONTROL_REJ0;
                            buf[3] = buf[1] ^ buf[2];
                            buf[4] = FLAG;

                            if (writeBytes(buf, 5)) printf("Sent 5 bytes for REJ response\n");
                            state = START;
                            memset(packet,i,0);
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
                       packet[i++] = c; //este erro vai ser detetado pelo bcc2 por isso no worries
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
    alarmCount=0;
    int sucess=0;
    if (role==LlTx){ // SE EU FOR TRASNMISSOE
        while(alarmCount < retraNum && !sucess){
            unsigned char buf[5]={0};
            buf[0] = FLAG;
            buf[1] = ADDRESS_T;
            buf[2] = CONTROL_DISC;
            buf[3] = buf[1] ^ buf[2];
            buf[4] = FLAG;
            writeBytes((const unsigned char *)buf, 5);
            alarm(time_out);
            alarmEnabled=TRUE;
            State state =START;
            char c =0;
            while(state != STOP_STATE && alarmEnabled ){
                if ( readByte(&c) >0) {
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
                            if (c == CONTROL_DISC ) {
                                state = C_RCV;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (C_RCV):
                            if (c == ADDRESS_T ^ CONTROL_DISC) {
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
                                sucess=1;
                                }
                            else state = START;
                            break;
                    }
                }
            }

        }
     //SEND UA
     printf("vou mandar o UA\n");
    unsigned char buf[5]={0};
    buf[0] = FLAG;
    buf[1] = ADDRESS_T;
    buf[2] = CONTROL_UA;
    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    writeBytes((const unsigned char *)buf, 5);



    }else if(role==LlRx){ // SE EU FOR RECETOR
                State state =START;
                char c =0;
                while(state != STOP_STATE ){
                if ( readByte(&c) >0) {
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
                            if (c == CONTROL_DISC ) {
                                state = C_RCV;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (C_RCV):
                            if (c == ADDRESS_T ^ CONTROL_DISC) {
                                state = BCC_OK;
                                }
                            else if (c == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case (BCC_OK):
                            if (c == FLAG) {
                                state = STOP_STATE;
                                }
                            else state = START;
                            break;
                    }
                }
            }
            while(alarmCount < retraNum && !sucess){
                unsigned char buf[5]={0};
                buf[0] = FLAG;
                buf[1] = ADDRESS_T;
                buf[2] = CONTROL_DISC;
                buf[3] = buf[1] ^ buf[2];
                buf[4] = FLAG;
                writeBytes((const unsigned char *)buf, 5);
                alarm(time_out);
                alarmEnabled=TRUE;
                State state =START;
                char c =0;
                while(state != STOP_STATE && alarmEnabled ){
                    if ( readByte(&c) >0) {
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
                                if (c == CONTROL_UA ) {
                                    state = C_RCV;
                                    }
                                else if (c == FLAG) state = FLAG_RCV;
                                else state = START;
                                break;
                            case (C_RCV):
                                if (c == ADDRESS_T ^ CONTROL_UA) {
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
                                    sucess=1;
                                    }
                                else state = START;
                                break;
                        }
                    }
                }

            }
    }
    int clstat = closeSerialPort();

    return clstat;
}
