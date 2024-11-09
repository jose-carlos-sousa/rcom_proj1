// Link layer header.
// NOTE: This file must not be changed.

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_
#define ADDRESS_T (0x03)
#define ADDRESS_R (0x01)
#define CONTROL_UA (0x07)
#define CONTROL_SET (0x03)
#define CONTROL_RR0 (0xAA)
#define CONTROL_RR1 (0xAB)
#define CONTROL_REJ0 (0x54)
#define CONTROL_REJ1 (0x55)
#define CONTROL_DISC (0x0B)
#define FLAG (0x7E)
#define ESCAPE 0x7D
#define MOD_FLAG 0x5e
#define MOD_ESCAPE 0x5d
#define BCC1_ERROR_LIKELYHOOD 0
#define BCC2_ERROR_LIKELYHOOD 0

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

typedef enum {START, FLAG_RCV, A_RCV, C_RCV, BCC_OK, STOP_STATE, READING_DATA, DATA_FOUND_ESC} State;
// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return "1" on success or "-1" on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or "-1" on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or "-1" on error.
int llread(unsigned char *packet);

// Close previously opened connection.
// if showStatistics == TRUE, link layer should print statistics in the console on close.
// Return "1" on success or "-1" on error.
int llclose(int showStatistics);

#endif // _LINK_LAYER_H_
