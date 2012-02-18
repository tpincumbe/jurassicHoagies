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
#include "subsystem.h"

class PlannerSubsystem : public Subsystem
{
public:
	PlannerSubsystem(void);
	~PlannerSubsystem(void);
	void Tick(size_t tick);
	void Execute(string behavior, string argument);
	void Shutdown();
	string MonitorMessage();
private:
	vector<string> m_pleoSequence;
	size_t m_currentSequence;
};
