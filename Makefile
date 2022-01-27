# Single lora testing app

CC=g++
CFLAGS=-c -Wall -pedantic -pthread
LIBS=-lpigpio -lrt
PYTHONVER=python3.9

all: loratest

app: loratest

loratest: main.o lora.o
	$(CC) main.o lora.o  $(LIBS) -L/usr/lib/$(PYTHONVER)/ -o loratest.exe

lora.o: lora.cpp
	$(CC) $(CFLAGS) -I/usr/include/$(PYTHONVER) lora.cpp
	
main.o: main.cpp
	$(CC) $(CFLAGS) -I/usr/include/$(PYTHONVER) main.cpp

clean:
	rm -f *.o *.so *.exe

