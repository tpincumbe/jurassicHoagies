// SerialTerm.cpp : Serial Port Communication Interface
// Doc: http://msdn.microsoft.com/library/default.asp?url=/library/en-us/dnfiles/html/msdn_serial.asp

#include "SerialTerm.h"
#include <iostream>
#include <stdio.h>
#include <windows.h>
#include <process.h>
#include <fstream>
#include <assert.h>
#include <string.h>

DCB dcb;
DWORD d_bytes_recv;
DWORD d_bytes_sent;
DWORD d_modem_status;
HANDLE h_port;
OVERLAPPED o_read = {0};
OVERLAPPED o_write = {0};
HANDLE t[2]; 
Logger *m_serial_logFile;

// Serial terminal read in syncrhonous mode, independent thread, faster!
void _recv(void*) {
	BYTE byte;
	char byte_r[100] = {0};
	char empty[100] = {0};

	int i = 0; 
	int j = 0;
	int ret = 0, wret = 0;

	printf("SerialTerm receiver is ON\n");
	while(1) {
		i++;
		wret = WaitCommEvent(h_port, &d_modem_status, &o_read);
		if(wret == 0) {
			if(GetLastError() == ERROR_IO_PENDING)
			ret = GetOverlappedResult(o_read.hEvent, &o_read, &d_bytes_recv, TRUE);
		}

		if(ret || wret) {
			if(d_modem_status & EV_RXCHAR) {
				while((d_bytes_recv) > 0 && ReadFile(h_port, &byte, 1, &d_bytes_recv, &o_read)) {
					printf("%c", byte);
					// Parse stream here, update sensors
				}
			}

			else if(d_modem_status & EV_ERR) {
				printf("Error character received");
			}
		}
		else {
			printf("Serial Term I/O function error\n");
			break;
		}
			
	} // end while

	CloseHandle(o_read.hEvent);
}

// Send data over serial port. Non-blocking in asynchronous mode, blocking in synchronous mode
void serialterm_send(const char* data, int data_size) {
	char s_data[30] = {0};
	memcpy(s_data, data, data_size);
	s_data[data_size] = '\n';
//	m_serial_logFile->out("[OUT] %s",s_data);
	//	printf("%s", s_data);


#ifdef PLEO_COMM_SYNC
	if(GetOverlappedResult(h_port, &o_write, &d_bytes_sent, TRUE)) {
		WriteFile(h_port, &s_data, strlen(s_data), &d_bytes_sent, &o_write);
	}
#else
	WriteFile(h_port, &s_data, strlen(s_data), &d_bytes_sent, NULL);
#endif
}

// Serial terminal read in asyncrhonous mode
void serialterm_read(char* data, int *data_size) {
#ifndef PLEO_COMM_SYNC
	char prev_sensor[512];
	char sensor[512];
	int prev_size = 0;
	int size = 0;
	int reset = 0;
	BYTE byte;
	WaitCommEvent(h_port, &d_modem_status, NULL);
	while(ReadFile(h_port, &byte, 1, &d_bytes_recv, NULL)) 
	{
		m_serial_logFile->out("[IN] %c",byte);
//		printf("%c", byte);
// Parse input stream to get string between #..#
		sensor[size] = byte;
		size = (size + 1)%512;
		if(byte == '#') {
			prev_size = size;
			memcpy(prev_sensor, sensor, prev_size);
			size = 0;
		}
// End parsing, records the last string between #'s
		if(d_bytes_recv == 0) break;
	}

	if(prev_size > 1) {
		memcpy(data, prev_sensor, prev_size-1);
		*data_size = prev_size-1;
	}
#endif
}

// Connects to serial port c_port, with baudrate 115200 and N-8-1
int serialterm_init(const char * c_port,Logger *logFile)
{
	COMMTIMEOUTS timeouts;
	DWORD flags = NULL;
#ifdef PLEO_COMM_SYNC 
	flags = FILE_FLAG_OVERLAPPED; // for syncrhronous tx rx only
#endif
	m_serial_logFile = logFile;
	// Create a file handle to the c_port buffer
	int port_len = lstrlenA(c_port);
	BSTR ustr = SysAllocStringLen(NULL,port_len);
	::MultiByteToWideChar(CP_ACP,0,c_port,port_len,ustr,port_len);
	h_port = CreateFile(ustr, GENERIC_READ | GENERIC_WRITE, 
						0, NULL, OPEN_EXISTING,
						flags,
						NULL);
	::SysFreeString(ustr);

	FillMemory(&dcb, sizeof(dcb), 0);
	if(!GetCommState(h_port, &dcb))
		return 0;

	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	// Disable hardware flow control
	dcb.fOutxCtsFlow = false;
	dcb.fOutxDsrFlow = false;
	dcb.fDsrSensitivity = false;
	dcb.fNull = true;
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	dcb.fBinary = true;
	// Disable software flow control
	dcb.fOutX = false;
	dcb.fInX = false;

	

	if(!SetCommState(h_port, &dcb))
		return 0;

	// Set timeouts
	timeouts.ReadIntervalTimeout = 500; 
	timeouts.ReadTotalTimeoutMultiplier = 500;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 500;
	timeouts.WriteTotalTimeoutConstant = 100;

	if (!SetCommTimeouts(h_port, &timeouts))
		return 0;

	// Spawn events when character is received, transmitter buffer is empty or error occurs in _recv thread 
	if (!SetCommMask(h_port, EV_RXCHAR | EV_TXEMPTY | EV_ERR))
		return 0;

	// Purge garbage characters in receiver and transmitter buffer
	PurgeComm(h_port, PURGE_TXCLEAR & PURGE_RXCLEAR & PURGE_RXABORT & PURGE_TXABORT);

//	printf("%s Port Set\n", c_port);

	// Create Event handle for receiver and transmitter
	// Event handles are used to query status of a read/write request
	o_read.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	o_read.Internal = 0;
    o_read.InternalHigh = 0;
    o_read.Offset = 0;
    o_read.OffsetHigh = 0;

	assert(o_read.hEvent);

	o_write.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

	assert(o_write.hEvent);

	return 1;
}

// Create a thread to receive data stream
void serialterm_receive_thread() {
#ifdef PLEO_COMM_SYNC
	t[0]= (HANDLE) _beginthread(_recv, 0, NULL);
#endif
}

// Block till all threads exit
void serialterm_wait() {
#ifdef PLEO_COMM_SYNC
	// Wait till handle t[0] exits or infinite time.
	WaitForSingleObject(t[0], INFINITE);
#endif
}

// Close all connections
void serialterm_close() {
//	printf("\nPort Closing...");

	CloseHandle(t[0]);
	CancelIo(h_port);
	CloseHandle(h_port);
}

