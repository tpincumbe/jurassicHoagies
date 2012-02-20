// SerialTerm.cpp : Serial Port Communication Interface
// 
#include "Logger.h"

//#define PLEO_COMM_SYNC 
// Uncomment above line to start synchronous mode. 
// Connects to serial port c_port, with baudrate 115200 and N-8-1
int serialterm_init(const char * c_port_specifier,Logger *logFile);

// Send data over serial port
void serialterm_send(const char* data, int data_size);

// Receive data over serial port in Asynchronous mode
void serialterm_read(char* data, int *data_size);

// Create a thread to receive data stream, synchronous mode
void serialterm_receive_thread();

// Block till all threads exit
void serialterm_wait();

// Close all connections
void serialterm_close();