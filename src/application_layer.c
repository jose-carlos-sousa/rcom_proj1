#include "application_layer.h"
#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>



enum state recvState = APPLICATION_START;
props mypros = {0, "", 0, 0, 0};

int framesSent = 0, framesRead = 0, originalSize = 0, newSize = 0, fileSize = 0;
int totalAlarms = 0, totalRejs = 0, totalRRs = 0, totalDups = 0;

struct timeval programStart, programEnd;



// Função para dar handle a erros fatais
void handleFatalError(const char *msg, FILE *file, int closeLinkLayer) {
    if (file) {
        fclose(file);
    }
    printf("%s", msg);
}

int send_control_package(unsigned char control, unsigned int file_size, const char *file_name) {
    if (file_name == NULL) {
        return -1;
    }   

    unsigned char packet[MAX_PACKET_SIZE] = {0}; 
    packet[0] = control;
    packet[1] = T_FILESIZE;

    unsigned char size_bytes = 0;
    unsigned int size_copy = file_size;
    do {
        size_copy /= 256;
        size_bytes++;
    } while (size_copy > 0);

    if (size_bytes > 8) {
        return -1; 
    }

    packet[2] = size_bytes;

   
    for (int i = 0; i < size_bytes; i++) {
        packet[3 + i] = (file_size >> (8 * (size_bytes - i - 1))) & 0xFF;
    }

    unsigned char file_name_length = (unsigned char)strlen(file_name);
    
 
    if (5 + size_bytes + file_name_length > MAX_PACKET_SIZE) {
        return -1; 
    }

    packet[3 + size_bytes] = T_FILENAME;
    packet[4 + size_bytes] = file_name_length;
    memcpy(&packet[5 + size_bytes], file_name, file_name_length);

 
    int total_size = 5 + size_bytes + file_name_length;
    return llwrite(packet, total_size);
}

int read_control_package(const unsigned char *control_buffer) {
    if (control_buffer == NULL) return -1;

    if (control_buffer[0] == CTRL_START) 
        recvState = APPLICATION_START;
    else if (control_buffer[0] == CTRL_END) 
        recvState = APPLICATION_END;
    else 
        return -1;

    if (control_buffer[1] != 0) return -1;

    unsigned int file_size_bytes = control_buffer[2];
    unsigned int file_size = 0;
    for (int i = 0; i < file_size_bytes; i++) {
        file_size |= ((unsigned int)control_buffer[3 + i] << (8 * (file_size_bytes - i - 1)));
    }

    if (control_buffer[3 + file_size_bytes] != 1) return -1;
    
    unsigned char file_name_size = control_buffer[4 + file_size_bytes];
    if (file_name_size < MAX_FILE_NAME) {
        strncpy(mypros.file_name, (const char *)&control_buffer[5 + file_size_bytes], file_name_size);
        mypros.file_name[file_name_size] = '\0';
    } else {
        handleFatalError("File name too long\n", NULL, TRUE);
        return -1; 
    }

    if (control_buffer[0] == CTRL_START) {
        mypros.file_size = file_size;
        mypros.bytesRead = 0; 
    } 
    else if (control_buffer[0] == CTRL_END) {
        if (mypros.file_size != mypros.bytesRead) {
            handleFatalError("File size mismatch\n", NULL, TRUE);
            return -1;
        }
        if (strcmp(mypros.file_name, mypros.file_name) != 0) {
            handleFatalError("File name mismatch\n", NULL, TRUE);
            return -1;
        }
    }

    return 0;
}


int read_data_packet(const unsigned char *packet, unsigned char *data_buffer, unsigned int *data_size) {
    if (packet == NULL || data_buffer == NULL) return -1;

   
    if (packet[0] != CTRL_DATA) {
        handleFatalError("Invalid packet type\n", NULL, TRUE);
        return -1;
    }

    unsigned int sequence_number = packet[1];
    unsigned int l2 = packet[2];
    unsigned int l1 = packet[3];
    *data_size = (l2 << 8) | l1; 

    if (sequence_number != mypros.expected_sequence_number) {
        printf("Sequence number mismatch\n");
    }

    
    if (*data_size > MAX_PAYLOAD_SIZE) {
        handleFatalError("Data size too large\n", NULL, TRUE);
        return -1;
    }

    memcpy(data_buffer, &packet[4], *data_size); 
    return 0;
}


int send_data_packet(unsigned int nBytes, unsigned char *data) {
    if (data == NULL) return -1;

    unsigned char packet[MAX_PACKET_SIZE];
    packet[0] = CTRL_DATA;
    packet[1] = mypros.sequence_number; 
    packet[2] = nBytes / 256;
    packet[3] = nBytes % 256;

    memcpy(packet + 4, data, nBytes);

    mypros.sequence_number = (mypros.sequence_number + 1) % 256; // Incrementar numero de sequencia

    return llwrite(packet, nBytes + 4); 
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename) {
    gettimeofday(&programStart, NULL);

    LinkLayer l = {
        .role = strcmp(role, "tx") ? LlRx : LlTx,
        .baudRate = baudRate,
        .nRetransmissions = nTries,
        .timeout = timeout
    };
    strncpy(l.serialPort, serialPort, sizeof(l.serialPort));

    FILE *file; 

    if (llopen(l) == -1) {
        handleFatalError("Error opening the connection in the link layer.\n", NULL, FALSE);
        return;
    }

    if(l.role == LlRx) {
        recvState = APPLICATION_START;
        file = fopen(filename,"wb");
        if (file == NULL) {
            handleFatalError("Failed to open file in writing mode\n", NULL, TRUE);
            return;
        }

        unsigned char buffer[MAX_PAYLOAD_SIZE] = {0};
        while (recvState != APPLICATION_END) {
            int bytes_read = llread(buffer);
            if (bytes_read == -1) {
                handleFatalError("Failed to read buffer in the link layer.\n", file, TRUE);
                return;
            }
            switch (buffer[0]) {
                case CTRL_START:
                case CTRL_END:
                    if (read_control_package(buffer) == -1) {
                        handleFatalError("Failed to read control package.\n", file, TRUE);
                        return;
                    }
                    break;

                case CTRL_DATA: {
                    unsigned int data_size;
                    unsigned char data[MAX_PAYLOAD_SIZE]; // buffer for the read data
                    int result = read_data_packet(buffer, data, &data_size);
                    
                    if (result == -1) {
                        handleFatalError("Failed to read data packet\n", file, TRUE);
                        return;
                    }

                    unsigned int written = fwrite(data, 1, data_size, file); // write data to file
                    if (written != data_size) {
                        handleFatalError("Failed to write data to file.\n", file, TRUE);
                        return;
                    }
                    mypros.bytesRead += data_size;
                    mypros.expected_sequence_number = (mypros.expected_sequence_number + 1) % 256;
                    break;
                }

                default:
                    break;
            }

        }
        printf("File received successfully\n");
    } else { // Transmitter
        unsigned char buffer[MAX_PACKET_SIZE] = {0};
        file = fopen(filename,"rb");
        if (!file) {
            handleFatalError("Failed to open file in reading mode.\n", NULL, TRUE);
            return;
        }
        struct stat fileStat;

        if (stat(filename, &fileStat) < 0) {
            handleFatalError("Unable to get file size.\n", file, TRUE);
            return;
        }
        unsigned int file_size = fileStat.st_size;
        fileSize = file_size;

        if (send_control_package(CTRL_START, file_size, filename) == -1) {
            handleFatalError("Failed to send control package (CTRL_START)\n", file, TRUE);
            return;
        }

        while ((mypros.bytesRead < file_size)) {
            unsigned int bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file);
            if (bytesRead > 0) {
                if (send_data_packet(bytesRead, buffer) < 0) {
                    handleFatalError("Failed to send data packet\n", file, TRUE);
                    return;
                }
                mypros.bytesRead += bytesRead;
            } else {
                handleFatalError("Error reading file data.\n", file, TRUE);
                return;
            }
        }

        if (send_control_package(CTRL_END, mypros.bytesRead, filename) == -1) {
            handleFatalError("Failed to send control package (CTRL_END)\n", file, TRUE);
            return;
        }
        printf("File sent successfully\n");
    }
    fclose(file);
    gettimeofday(&programEnd, NULL);
    if (llclose(TRUE) == -1) {
        handleFatalError("Failed to close connection in link layer\n", NULL, FALSE);
        return;
    }


}

