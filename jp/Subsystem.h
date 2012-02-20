//--------------------------------------------------------------------
//
// Team: Jurassic Pork
//
// Members:
//	David House
//  John Turner
//  Shashank Chamoli
//  Tim Pincumbe
//
// Note: Portions of our code have been copied & modified from the 
// MiGIO-2 template provided by Dr. Stillman
//
//--------------------------------------------------------------------
#pragma once
#include "Logger.h"
#include <windows.h>
#include <process.h>
#include <string>
#include "SystemQueue.h"
using namespace std;

#define TICKS_PER_MONITOR_MESSAGE 25

class Subsystem
{
public:
	Subsystem(void);
	~Subsystem(void);

	void Start(string name, SystemQueue* sysQueue);
	void Stop();
	bool IsRunning();
	void ThreadProc();
	virtual void Tick(size_t tick) = 0;
	virtual void Execute(string behavior, string argument) = 0;
	virtual void Shutdown() = 0;
	virtual string MonitorMessage() = 0;
protected:
	bool m_running;
	bool m_stopPending;
	int m_tickSleep;
	Logger *m_logFile;
	string m_name;
	SystemQueue *m_sysQueue;
	size_t m_lastMonitorTick;
	bool m_monitorActive;
	CRITICAL_SECTION m_cs;

	void SendMessage(string destination,vector<string> msg);
	void SendNotification(vector<string> msg);
};
