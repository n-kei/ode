CC=g++
TARGET=Serial4Arduino
OBJS=main.o Serial4Arduino.o
INCLUDES=run.h Serial4Arduino.h
CFLAGS=-g -Wall -O0

.SUFFIXES: .c.o

.c.o:
	$(CC) -c $<

$(TARGET): $(OBJS) $(INCLUDES)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET)

.PHONY: clean
clean:
	rm -Rf *.o
