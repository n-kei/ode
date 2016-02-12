CC=g++
TARGET=rover
OBJS=main.o Serial4Arduino.o control.o run.o
INCLUDES=run.h Serial4Arduino.h control.h
CFLAGS=-g -Wall -O0
LIBS=/usr/local/lib/libdrawstuff.a -lX11 /usr/local/lib/libode.so -lGLU -lGL -lm -lpthread
INCS=-I/usr/include/ode -I/usr/local/include/drawstuff -DdTRIMESH_ENABLED -DdDOUBLE 

vpath %.h Serial4Arduino
vpath %.o Serial4Arduino
vpath %.c Serial4Arduino

.SUFFIXES: .c.o

.c.o:
	$(CC) -c $< 

$(TARGET): $(OBJS) $(INCLUDES)
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(INCS) $(LIBS)

.PHONY: clean
clean:
	rm -Rf *.o
