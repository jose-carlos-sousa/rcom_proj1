// Application layer protocol header.
// NOTE: This file must not be changed.

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_
#define CTRL_START 0x01
#define CTRL_END 0x03
#define CTRL_DATA 0x02
#define MAX_FILE_NAME 50
#define T_FILESIZE 0
#define T_FILENAME 1
#define MAX_PACKET_SIZE MAX_PAYLOAD_SIZE+5
enum state {
    APPLICATION_START,
    APPLICATION_DATA,
    APPLICATION_END
};

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

#endif // _APPLICATION_LAYER_H_
