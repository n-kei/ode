#include <stdio.h>
#include <stdlib.h>
#include "Serial4Arduino.h"

int main(void)
{
  char *str = NULL;
  char PortName[100];
  int h;
  
  ScanOpenSerialPort();

  printf("利用するシリアルポートを指定してください:");
  scanf("%s", PortName);
  h = SerialBegin(PortName, 9600);
  
  while(1) {
    while(str == NULL) 
      str = SerialReadData(h);
    fprintf(stderr, "str = %s\n", str);
    str = NULL;
  }
  
  return(0);  
}
