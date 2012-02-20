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
#include <windows.h>
#include <vector>
#include <queue>
#include <map>
#include "Queue.h"
using namespace std;

class SystemQueue
{
public:
	SystemQueue(void);
	~SystemQueue(void);
	void RegisterSystem(string name);
	bool HasMessage(string name);
	vector<string> PopMessage(string name);
	void PushMessage(string name,vector<string> msg);
	void PushToAll(vector<string> msg);
private:
	CRITICAL_SECTION m_cs;
	map<string,Queue *> m_queues;
};
