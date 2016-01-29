#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include "Serial4Arduino.h"
#include "run.h"

void ScanOpenSerialPort(void)
{
  system("dmesg | grep ttyACM | awk '{print $4}'");
}

int SerialBegin(char *PortName, speed_t BaudRate)
{
  struct termios tio;
  char PortPath[100];
  int fd;

  sprintf(PortPath, "/dev/%s", PortName);
  fd = open(PortPath, O_RDWR | CLOCAL | CS8 );
  if(fd < 0) {
    perror(PortName);
    exit(1);
  }

  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = CS8 | CLOCAL | CREAD;
  tio.c_cc[VTIME] = 100;

  cfsetispeed(&tio, BaudRate);
  cfsetospeed(&tio, BaudRate);

  tcsetattr(fd, TCSANOW, &tio);

  return(fd);
}

char *SerialReadData(int SerialHandle)
{
  static int i = 0;
  char sBuf[2];
  char *str;
  unsigned long nn;
  
  str = (char *)malloc(sizeof(char) * 100);
  
  nn = read(SerialHandle, sBuf, 1);
  sBuf[1] = '\0';

  if(nn == 1) {
    if(strcmp(sBuf, SYN) != 0) {
      return(NULL);
    }
  } else {
    perror("SerialReadData");
    return(NULL);
  }
  
  write(SerialHandle, ACK, strlen(ACK));			
  while(1) {
    nn = read(SerialHandle, sBuf, sizeof(sBuf));
    sBuf[1] = '\0';
    if(nn == 1) {
      if(strcmp(sBuf, FIN) == 0) {
	if( i != 0) {
	  str[i] = '\0';
	  i = 0;
	  return(str);
	}
      } else {
	str[i] = sBuf[0];
	i++;
      }
    }
  }
}

void SerialSendData(int SerialHandle, char *SendData)
{
  char sBuf[2];
  unsigned long nn;
  
  memset(sBuf, 0, sizeof(sBuf));
  while(1) {
    write(SerialHandle, SYN, strlen(SYN));
    nn = read(SerialHandle, sBuf, sizeof(sBuf));
    sBuf[1] = '\0';
    if(strcmp(sBuf, ACK) == 0)
      break;
  }
  
  write(SerialHandle, SendData, strlen(SendData));
  write(SerialHandle, FIN, strlen(FIN));
}

