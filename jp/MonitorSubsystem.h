#pragma once
#include "subsystem.h"
#include <winsock.h>

class MonitorSubsystem :
	public Subsystem
{
public:
	MonitorSubsystem(void);
	~MonitorSubsystem(void);
	void Tick(size_t tick);
	void Execute(string behavior, string argument);
	void Shutdown();
	string MonitorMessage();
private:
	int m_sock;
	int m_clientSock;
    struct sockaddr_in m_echoServAddr; /* Local address */
};
