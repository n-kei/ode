#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include <termios.h>

#define ACK "/"
#define SYN "*"
#define FIN "+"

#define SERIAL_PORT_NUM 3
#define SERIAL_PORT_NAME_MAXLENGTH 8

void ScanOpenSerialPort(void);
int SerialBegin(char *PortName, speed_t BaudRate);
char *SerialReadData(int SerialHandle);
void SerialSendData(int SerialHandle, char *SendData);

#endif
