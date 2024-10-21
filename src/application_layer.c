#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>

#define DATA 2
#define C_START 1
#define C_END 3
#define T_FILESIZE 0
#define T_FILENAME 1
#define MAX_FILE_NAME 50
#define MAX_PACKET_SIZE (MAX_PAYLOAD_SIZE + 5)

typedef struct {
    size_t file_size;
    char file_name[MAX_FILE_NAME];
    size_t bytesRead;
} props;

static enum state stateReceive = TRANF_START;
static props mypros = {0, "", 0};

static void handleError(const char *msg) {
    perror(msg);
}

// Centralized error handling function
static void handleFatalError(const char *msg, FILE *file, int closeLinkLayer) {
    handleError(msg);
    if (file) {
        fclose(file);
    }
    if (closeLinkLayer) {
        llclose(FALSE);
    }
}

int sendData(size_t nBytes, unsigned char *data) {
    if (data == NULL) return -1;

    unsigned char packet[MAX_PACKET_SIZE];
    packet[0] = DATA;
    packet[1] = nBytes / 256;
    packet[2] = nBytes % 256;

    memcpy(packet + 3, data, nBytes);

    return llwrite(packet, nBytes + 3);
}

size_t uchartoi(unsigned char n, unsigned char *numbers) {
    size_t value = 0;
    for (size_t i = 0; i < n; i++) {
        value += numbers[i] << (i * 8);
    }
    return value;
}

int sendCtrl(unsigned char C, const char *filename, size_t file_size) {
    if (filename == NULL) return -1;

    unsigned char packet[MAX_PACKET_SIZE];
    size_t index = 0;

    packet[index++] = C;

    packet[index++] = T_FILESIZE;
    unsigned char L1 = 0;
    unsigned char V1[8];
    while (file_size && L1 < 8) {
        V1[L1++] = (unsigned char)(file_size % 256);
        file_size /= 256;
    }
    packet[index++] = L1;
    memcpy(packet + index, V1, L1);
    index += L1;

    packet[index++] = T_FILENAME;
    unsigned char L2 = (unsigned char)strlen(filename);
    packet[index++] = L2;
    memcpy(packet + index, filename, L2);
    index += L2;

    return llwrite(packet, index);
}

unsigned char *readData(unsigned char *buf, size_t *new_data_size) {
    if (buf == NULL) return NULL;
    *new_data_size = (buf[1] * 256) + buf[2];
    return buf + 3;
}

int readCtrl(unsigned char *buf) {
    if (buf == NULL) return -1;

    size_t index = 0;
    char file_name[MAX_FILE_NAME] = {0};

    if (buf[index] == C_START) stateReceive = TRANF_DATA;
    else if (buf[index] == C_END) stateReceive = TRANF_END;
    else return -1;

    index++;
    if (buf[index++] != T_FILESIZE) return -1;

    unsigned char L1 = buf[index++];
    unsigned char V1[8];
    memcpy(V1, buf + index, L1);
    index += L1;
    size_t file_size = uchartoi(L1, V1);

    if (buf[index++] != T_FILENAME) return -1;
    unsigned char L2 = buf[index++];
    memcpy(file_name, buf + index, L2);
    file_name[L2] = '\0';

    if (buf[0] == C_START) {
        mypros.file_size = file_size;
        strncpy(mypros.file_name, file_name, MAX_FILE_NAME - 1);
        mypros.file_name[MAX_FILE_NAME - 1] = '\0';
    } else if (buf[0] == C_END) {
        if (mypros.file_size != mypros.bytesRead) {
            handleError("Number of bytes read doesn't match size of file");
        }
        if (strcmp(mypros.file_name, file_name)) {
            handleError("File names don't match");
        }
    }
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer l = {
        .role = strcmp(role, "tx") ? LlRx : LlTx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    memcpy(l.serialPort, serialPort, sizeof(l.serialPort));

    if (llopen(l) == -1) {
        handleFatalError("Link layer error: Failed to open the connection.", NULL, TRUE);
        return;
    }

    if (l.role == LlTx) {
        printf("olaola\n");
        unsigned char buffer[MAX_PACKET_SIZE];
        FILE *file = fopen(filename, "rb");
        if (!file) {
            handleFatalError("File error: Unable to open the file for reading.", NULL, TRUE);
            return;
        }
        struct stat fileStat;
        if (stat(filename, &fileStat) < 0) {
            handleFatalError("File error: Unable to get file size.", file, TRUE);
            return;
        }
        size_t file_size = fileStat.st_size;

        if (sendCtrl(C_START, filename, file_size) == -1) {
            handleFatalError("Transmission error: Failed to send the START packet control.", file, TRUE);
            return;
        }

        size_t bytes_read;
        while ((bytes_read = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            if (sendData(bytes_read, buffer) == -1) {
                handleFatalError("Transmission error: Failed to send the DATA packet control.", file, TRUE);
                return;
            }
            mypros.bytesRead += bytes_read;
        }

        if (sendCtrl(C_END, filename, file_size) == -1) {
            handleFatalError("Transmission error: Failed to send the END packet control.", file, TRUE);
            return;
        }

        fclose(file);
    }

    if (l.role == LlRx) {
        unsigned char buf[MAX_PACKET_SIZE];
        FILE *file = fopen(filename, "wb");
        if (!file) {
            handleFatalError("File error: Unable to open the file for writing.", NULL, TRUE);
            return;
        }

        while (stateReceive != TRANF_END) {
            size_t bytes_read = llread(buf);
            if (bytes_read == -1) {
                handleFatalError("Link layer error: Failed to read from the link.", file, TRUE);
                return;
            }

            if (buf[0] == C_START || buf[0] == C_END) {
                if (readCtrl(buf) == -1) {
                    handleFatalError("Packet error: Failed to read control packet.", file, TRUE);
                    return;
                }
            } else if (buf[0] == DATA) {
                size_t dataSize;
                unsigned char *packet = readData(buf, &dataSize);
                if (!packet) {
                    handleFatalError("Packet error: Failed to read data packet.", file, TRUE);
                    return;
                }
                fwrite(packet, 1, dataSize, file);
                mypros.bytesRead += dataSize;
            }
        }

        fclose(file);
    }

    if (llclose(TRUE) == -1) {
        handleError("Link layer error: Failed to close the connection.");
    }
}
