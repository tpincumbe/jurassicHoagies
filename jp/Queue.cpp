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
#include "Queue.h"

Queue::Queue(void)
{
}

Queue::~Queue(void)
{
}

void Queue::Push(vector<string> msg)
{
	m_queue.push(msg);
}

vector<string> Queue::Pop()
{
	vector<string> msg = m_queue.front();
	m_queue.pop();
	return msg;
}

bool Queue::IsEmpty()
{
	return m_queue.empty();
}