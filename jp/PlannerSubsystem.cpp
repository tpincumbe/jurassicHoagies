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
#include "PlannerSubsystem.h"
#include "SerialTerm.h"

PlannerSubsystem::PlannerSubsystem(void)
{
	m_logFile = new Logger("planner.log");
	m_logFile->out("Planner Subsystem starting up...");
}

PlannerSubsystem::~PlannerSubsystem(void)
{
}

void PlannerSubsystem::Shutdown()
{
}


void PlannerSubsystem::Tick(size_t tick)
{
}

void PlannerSubsystem::Execute(string behavior, string argument)
{
	// assume argument is just a string for now
	m_logFile->out("behavior: %s arg: %s",behavior.c_str(),argument.c_str());

	if ( behavior.compare("findFruit") == 0 )
	{
		// Let the camera subsystem know that we are about to perform a square maneuver, so it
		// needs to draw an expected path and then start tracking.
		vector<string> cameraMsg;
		cameraMsg.push_back("camera");
		cameraMsg.push_back("startTracking");
		cameraMsg.push_back("fruit");
		SendMessage("camera",cameraMsg);
		
	}
	else if ( behavior.compare("grabBackground") == 0 )
	{
		vector<string> cameraMsg;
		cameraMsg.push_back("camera");
		cameraMsg.push_back("getBackground");
		SendMessage("camera",cameraMsg);
	}
	else if ( behavior.compare("grabObstacles") == 0 )
	{
		vector<string> cameraMsg;
		cameraMsg.push_back("camera");
		cameraMsg.push_back("getObstacles");
		SendMessage("camera",cameraMsg);
	}
	else if ( behavior.compare("performSquare") == 0 )
	{
		// Here we are going to get Pleo to perform a square maneuver, getting him to 
		// walk straight, then turn 90 degrees, then repeat until he has reached the 
		// origin point. This will be done using dead reckoning, so no camera input
		// is required, although the camera will be tracking his progress.


		// Let the camera subsystem know that we are about to perform a square maneuver, so it
		// needs to draw an expected path and then start tracking.
		vector<string> cameraMsg;
		cameraMsg.push_back("camera");
		cameraMsg.push_back("startTracking");
		cameraMsg.push_back("square");
		SendMessage("camera",cameraMsg);

		//we need to be able to load a file with a similar format and then execute it.  ideally behavior 
		// like this would be able to be created and edited external to code.
		m_pleoSequence.clear();
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("turnRight");
		m_pleoSequence.push_back("8");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("walkForward");
		m_pleoSequence.push_back("7");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("turnRight");
		m_pleoSequence.push_back("8");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("walkForward");
		m_pleoSequence.push_back("7");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("turnRight");
		m_pleoSequence.push_back("8");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("walkForward");
		m_pleoSequence.push_back("7");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("moo");
		m_pleoSequence.push_back("");
		m_currentSequence = 0;

		m_logFile->out("sending first command to pleo");
		vector<string> firstMsg;
		firstMsg.push_back("pleo");
		firstMsg.push_back("walkForward");
		firstMsg.push_back("7");
		SendMessage("pleo",firstMsg);
	}
	else if ( behavior.compare("performTriangle") == 0 )
	{
		// Here we are going to get Pleo to perform a triang maneuver, getting him to 
		// walk straight, then turn 90 degrees, then repeat until he has reached the 
		// origin point. This will be done using dead reckoning, so no camera input
		// is required, although the camera will be tracking his progress.

		// Let the camera subsystem know that we are about to perform a square maneuver, so it
		// needs to draw an expected path and then start tracking.
		vector<string> cameraMsg;
		cameraMsg.push_back("camera");
		cameraMsg.push_back("startTracking");
		cameraMsg.push_back("triangle");
		SendMessage("camera",cameraMsg);

		//we need to be able to load a file with a similar format and then execute it.  ideally behavior 
		// like this would be able to be created and edited external to code.
		m_pleoSequence.clear();
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("turnRight");
		m_pleoSequence.push_back("9");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("walkForward");
		m_pleoSequence.push_back("8");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("turnRight");
		m_pleoSequence.push_back("9");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("walkForward");
		m_pleoSequence.push_back("7");
		m_pleoSequence.push_back("normalize");
		m_pleoSequence.push_back("");
		m_pleoSequence.push_back("howl");
		m_pleoSequence.push_back("");
		m_currentSequence = 0;

		m_logFile->out("sending first command to pleo");
		vector<string> firstMsg;
		firstMsg.push_back("pleo");
		firstMsg.push_back("walkForward");
		firstMsg.push_back("7");
		SendMessage("pleo",firstMsg);
	}
	else if ( behavior.compare("pleoFinishedBehavior") == 0 )
	{
		// make sure we have a current sequence to tell pleo to do
		if ( ( m_currentSequence >= 0 ) && ( m_pleoSequence.size() > 0) )
		{
			m_logFile->out("sending next command to pleo");
			vector<string> msg;
			msg.push_back("pleo");
			msg.push_back(m_pleoSequence[m_currentSequence]);
			m_currentSequence++;
			msg.push_back(m_pleoSequence[m_currentSequence]);
			m_currentSequence++;
			SendMessage("pleo",msg);

			m_logFile->out("seq: %d  size: %d",m_currentSequence,m_pleoSequence.size());
			if (m_currentSequence >= m_pleoSequence.size() )
			{
				m_currentSequence = 0;
				m_pleoSequence.clear();
				m_logFile->out("sequence finished");
			}
		}
	}
}

string PlannerSubsystem::MonitorMessage()
{
	return "";
}