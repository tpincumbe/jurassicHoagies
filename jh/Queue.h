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
#include <vector>
#include <queue>
#include <string>
using namespace std;

class Queue
{
public:
	Queue(void);
	~Queue(void);
	void Push(vector<string> msg);
	vector<string> Pop();
	bool IsEmpty();
private:
	queue<vector<string>> m_queue;
};
