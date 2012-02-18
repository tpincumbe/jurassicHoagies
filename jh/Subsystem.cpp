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
#include "Subsystem.h"

extern "C"
{
	static void __cdecl ThreadHelper(void *data)
	{
		Subsystem *sys = reinterpret_cast<Subsystem *>(data);
		sys->ThreadProc();
	}
}


Subsystem::Subsystem(void)
{
	m_tickSleep = 100;
	m_monitorActive = true;
}

Subsystem::~Subsystem(void)
{
}

void Subsystem::ThreadProc()
{
	size_t tick = 0;
	m_running = true;
	m_stopPending = false;
	m_lastMonitorTick = 0;

	while ( !m_stopPending )
	{
		// Check the queue to see if we have any requests. If so, execute them.
		if ( m_sysQueue->HasMessage(m_name) )
		{
			vector<string> msg = m_sysQueue->PopMessage(m_name);
			m_logFile->out("got a message on the queue, now processing it!");
			if ( msg.size() > 2 )
				Execute(msg[1],msg[2]);
			else
				Execute(msg[1],"");
		}

		// Tick the subsystem
		Tick(tick);
		tick++;

		if ( tick > (m_lastMonitorTick + TICKS_PER_MONITOR_MESSAGE) )
		{
//			string monitorMsg = MonitorMessage();
//			if ( monitorMsg.length() > 0 )
//			{
//				vector<string> msg;
//				msg.push_back("monitor");
//				msg.push_back("monitorLog");
//				msg.push_back(monitorMsg);
//				SendNotification(msg);
//			}
			m_lastMonitorTick = tick;
		}

		// Sleep
		Sleep(m_tickSleep);
	}
	m_logFile->out("shutting down");
	Shutdown();
	EnterCriticalSection(&m_cs);
	m_running = false;
	LeaveCriticalSection(&m_cs);
}

void Subsystem::Start(string name, SystemQueue* sysQueue)
{
	m_name = name;
	m_sysQueue = sysQueue;
	 _beginthread(&ThreadHelper,0,reinterpret_cast< void* >(this)); 
}

void Subsystem::Stop()
{
	EnterCriticalSection(&m_cs);
	m_logFile->out("setting stop pending to true");
	m_stopPending = true;
	LeaveCriticalSection(&m_cs);
}

bool Subsystem::IsRunning()
{
	EnterCriticalSection(&m_cs);
	bool result = m_running;
	LeaveCriticalSection(&m_cs);
	return result;
}
void Subsystem::SendMessage(string destination,vector<string> msg)
{
	m_sysQueue->PushMessage(destination,msg);
}

void Subsystem::SendNotification(vector<string> msg)
{
	m_sysQueue->PushToAll(msg);
}
