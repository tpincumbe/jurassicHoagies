#include "MonitorSubsystem.h"
#include <fcntl.h>

MonitorSubsystem::MonitorSubsystem(void)
{
	m_clientSock = -1;
	m_logFile = new Logger("monitor.log");
	m_logFile->out("Monitor Subsystem starting up...");

	WSADATA wsaData;
	if (WSAStartup(MAKEWORD(2,0),&wsaData) != 0 )
	{
		m_logFile->out("error calling WSAStartup");
	}

	// start out listener socket
	if ((m_sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
	{
		m_logFile->out("error creating socket!");
	}

	/* Construct local address structure */
    memset(&m_echoServAddr, 0, sizeof(m_echoServAddr));   /* Zero out structure */
    m_echoServAddr.sin_family = AF_INET;                /* Internet address family */
    m_echoServAddr.sin_addr.s_addr = htonl(INADDR_ANY); /* Any incoming interface */
    m_echoServAddr.sin_port = htons(7676);      /* Local port */

    /* Bind to the local address */
    if (bind(m_sock, (struct sockaddr *) &m_echoServAddr, sizeof(m_echoServAddr)) < 0)
	{
		m_logFile->out("bind failed!");
	}

    if (listen(m_sock, 10) < 0)
	{
		m_logFile->out("listen failed!");
	}

	unsigned long blocking = 1;
	if ( ioctlsocket(m_sock,FIONBIO,&blocking) < 0 )
	{
		m_logFile->out("ioctlsocket failed!");
	}

	m_logFile->out("socket bound and ready for incoming messages");
}

MonitorSubsystem::~MonitorSubsystem(void)
{
}

void MonitorSubsystem::Tick(size_t tick)
{
	if ( m_clientSock == -1 )
	{
		struct sockaddr_in clntAddr;
		/* Set length of client address structure (in-out parameter) */
		int clntAddrLen = sizeof(clntAddr);
		m_clientSock = accept(m_sock, (struct sockaddr *) &clntAddr, &clntAddrLen);
	}
	else
	{
		char buffer[1024];
	    int messagelen = 0;
		int numBytesRcvd;
		memset(buffer,0,1024);
        
		/* read some data from the socket */
		numBytesRcvd = recv(m_clientSock, buffer, 1024, 0);
		if ( numBytesRcvd > 0 )
		{
			m_logFile->out("got a message of size %d from the client: %s",numBytesRcvd,buffer);
		}
	}
}

void MonitorSubsystem::Execute(string behavior, string argument)
{
	if ( behavior.compare("monitorLog") == 0 )
	{
		if ( m_clientSock >= 0 )
		{
			int numBytes = send(m_clientSock, argument.c_str(), argument.length(),0);
			if ( numBytes == -1 )
			{
				m_logFile->out("client socket closed or something");
				closesocket(m_clientSock);
				m_clientSock = -1;
			}
		}
	}
}

void MonitorSubsystem::Shutdown()
{
}

string MonitorSubsystem::MonitorMessage()
{
	return "";
}