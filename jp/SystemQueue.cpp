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
#include "SystemQueue.h"
#include <iostream>

SystemQueue::SystemQueue(void)
{
	InitializeCriticalSection(&m_cs);
}

SystemQueue::~SystemQueue(void)
{
}

//
// RegisterSystem
//
// This method will create a new Queue object and place it into our
// map<> of queues using the passed in name as the key
//
void SystemQueue::RegisterSystem(string name)
{
	Queue *newQueue = new Queue();
	EnterCriticalSection(&m_cs);
	m_queues[name] = newQueue;
	LeaveCriticalSection(&m_cs);
}

//
// HasMessage
//
// This methd will simply check the named Queue to see if it has
// any messages
//
bool SystemQueue::HasMessage(string name)
{
	EnterCriticalSection(&m_cs);
	Queue *q = m_queues[name];
	bool rtn = !q->IsEmpty();
	LeaveCriticalSection(&m_cs);
	return rtn;
}

//
// PopMessage
//
// This method will remove a message from a named queue
// and return it
//
vector<string> SystemQueue::PopMessage(string name)
{
	EnterCriticalSection(&m_cs);
	Queue *q = m_queues[name];
	vector<string> msg = q->Pop();
	LeaveCriticalSection(&m_cs);
	return msg;
}

//
// PushMessage
//
// This method will add a message to a named queue
//
void SystemQueue::PushMessage(string name,vector<string> msg)
{
	EnterCriticalSection(&m_cs);
	Queue *q = m_queues[name];
	if ( q )
	{
		q->Push(msg);
	}
	LeaveCriticalSection(&m_cs);
}

//
// PushToAll
//
// This method will push a message into all the queues. This is
// used for system wide notifications
//
void SystemQueue::PushToAll(vector<string> msg)
{
	EnterCriticalSection(&m_cs);
	for ( map<string,Queue *>::iterator it = m_queues.begin(); it != m_queues.end(); ++it )
	{
		string name = (*it).first;
		Queue *q = (*it).second;
		q->Push(msg);
	}
	LeaveCriticalSection(&m_cs);
}
