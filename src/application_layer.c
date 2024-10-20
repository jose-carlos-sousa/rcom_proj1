#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#define DATA 2
#define C_START 1
#define C_END 3
#define T_FILESIZE 0
#define T_FILENAME 1
#define MAX_FILENAME 50
#define MAX_PACKET_SIZE (MAX_PAYLOAD_SIZE + 5)



typedef struct {
    size_t file_size;
    char file_name[MAX_FILENAME]; // Changed to static allocation
    size_t bytesRead;
} FileProps;

static enum state stateReceive = TRANF_START;
static FileProps fileProps = {0, "", 0}; // Initialize file name as an empty string

static void handleError(const char *msg) {
    perror(msg);
}

int sendPacketData(size_t nBytes, unsigned char *data) {
    if (!data) return -1;

    unsigned char packet[MAX_PACKET_SIZE]; // Stack allocation
    packet[0] = DATA;
    packet[1] = nBytes >> 8;
    packet[2] = nBytes & 0xFF;
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

int sendPacketControl(unsigned char C, const char *filename, size_t file_size) {
    if (!filename) return -1;

    unsigned char L1;
    unsigned char V1[8]; // Stack allocation for max 8 bytes
    size_t length = 0;

    do {
        V1[length++] = file_size & 0xFF;
        file_size >>= 8;
    } while (file_size && length < 8);

    unsigned char L2 = (unsigned char) strlen(filename);
    unsigned char packet[5 + length + L2]; // Stack allocation

    size_t index = 0;
    packet[index++] = C;
    packet[index++] = T_FILESIZE;
    packet[index++] = length;
    memcpy(packet + index, V1, length); index += length;
    packet[index++] = T_FILENAME;
    packet[index++] = L2;
    memcpy(packet + index, filename, L2);

    return llwrite(packet, index);
}

unsigned char *readPacketData(unsigned char *buff, size_t *newSize) {
    if (!buff || buff[0] != DATA) return NULL;

    *newSize = (buff[1] << 8) + buff[2];
    return buff + 3;
}

int readPacketControl(unsigned char *buff) {
    if (!buff) return -1;

    size_t index = 0;
    char file_name[MAX_FILENAME] = {0}; // Allocate directly on stack

    if (buff[index] == C_START) stateReceive = TRANF_DATA;
    else if (buff[index] == C_END) stateReceive = TRANF_END;
    else return -1;

    index++;
    if (buff[index++] != T_FILESIZE) return -1;

    unsigned char L1 = buff[index++];
    unsigned char V1[8]; // Stack allocation for max 8 bytes
    memcpy(V1, buff + index, L1); index += L1;
    size_t file_size = uchartoi(L1, V1);

    if (buff[index++] != T_FILENAME) return -1;
    unsigned char L2 = buff[index++];
    memcpy(file_name, buff + index, L2);
    file_name[L2] = '\0';

    if (buff[0] == C_START) {
        fileProps.file_size = file_size;
        strncpy(fileProps.file_name, file_name, MAX_FILENAME - 1); // Prevent overflow
        fileProps.file_name[MAX_FILENAME - 1] = '\0'; // Ensure null termination
        printf("[INFO] Started receiving file: '%s'\n", file_name);
    } else if (buff[0] == C_END) {
        if (fileProps.file_size != fileProps.bytesRead) {
            handleError("Number of bytes read doesn't match size of file");
        }
        if (strcmp(fileProps.file_name, file_name)) {
            handleError("File names don't match");
        }
        printf("[INFO] Finished receiving file: '%s'\n", file_name);
    }
    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    if (!serialPort || !role || !filename) {
        handleError("Initialization error: One or more required arguments are NULL.");
        return;
    }

    if (strlen(filename) > MAX_FILENAME) {
        printf("The length of the given file name is greater than what is supported: %d characters\n", MAX_FILENAME);
        return;
    }

    LinkLayer connectionParametersApp = {
        .role = strcmp(role, "tx") ? LlRx : LlTx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    strncpy(connectionParametersApp.serialPort, serialPort, sizeof(connectionParametersApp.serialPort) - 1);

    if (llopen(connectionParametersApp) == -1) {
        handleError("Link layer error: Failed to open the connection.");
        llclose(FALSE);
        return;
    }

    if (connectionParametersApp.role == LlTx) {
        unsigned char buffer[MAX_PACKET_SIZE]; // Stack allocation
        FILE *file = fopen(filename, "rb");
        if (!file) {
            handleError("File error: Unable to open the file for reading.");
            llclose(FALSE);
            return;
        }
        struct stat fileStat;
    if (stat(filename, &fileStat) < 0) {
        perror("File error: Unable to get file size.");
        fclose(file);
        free(buffer);
        llclose(FALSE);
        return;
    }
    size_t file_size = fileStat.st_size;


        if (sendPacketControl(C_START, filename, file_size) == -1) {
            handleError("Transmission error: Failed to send the START packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        size_t bytesRead;
        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            if (sendPacketData(bytesRead, buffer) == -1) {
                handleError("Transmission error: Failed to send the DATA packet control.");
                fclose(file);
                llclose(FALSE);
                return;
            }
        }

        if (sendPacketControl(C_END, filename, file_size) == -1) {
            handleError("Transmission error: Failed to send the END packet control.");
            fclose(file);
            llclose(FALSE);
            return;
        }

        fclose(file);
    } 

    if (connectionParametersApp.role == LlRx) {
        unsigned char buf[MAX_PACKET_SIZE]; // Stack allocation
        FILE *file = fopen(filename, "wb");
        if (!file) {
            handleError("File error: Unable to open the file for writing.");
            llclose(FALSE);
            return;
        }

        while (stateReceive != TRANF_END) {
            size_t bytes_readed = llread(buf);
            if (bytes_readed == -1) {
                handleError("Link layer error: Failed to read from the link.");
                fclose(file);
                llclose(FALSE);
                return;
            }

            if (buf[0] == C_START || buf[0] == C_END) {
                if (readPacketControl(buf) == -1) {
                    handleError("Packet error: Failed to read control packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
            } else if (buf[0] == DATA) {
                size_t dataSize;
                unsigned char *packet = readPacketData(buf, &dataSize);
                if (!packet) {
                    handleError("Packet error: Failed to read data packet.");
                    fclose(file);
                    llclose(FALSE);
                    return;
                }
                fwrite(packet, 1, dataSize, file);
                fileProps.bytesRead += dataSize;
            }
        }

        fclose(file);
    }

    if (llclose(TRUE) == -1) {
        handleError("Link layer error: Failed to close the connection.");
    }
}
