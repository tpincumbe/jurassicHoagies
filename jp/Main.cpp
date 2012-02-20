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

#include <stdio.h>
#include "Logger.h"
#include <string>
#include <iostream>
#include <windows.h>
#include <process.h>
#include <queue>
#include <vector>
#include <map>
#include "Subsystem.h"
#include "PleoSubsystem.h"
#include "CameraSubsystem.h"
#include "PlannerSubsystem.h"
#include "SystemQueue.h"
#include "MonitorSubsystem.h"
using namespace std;

// a couple of global variables
map<string,Subsystem *> activeSystems;
SystemQueue *m_systemQueue;

//
// Take a string that has arguments separated by spaces and return
// them as a list
//
vector<string> SeparateArguments(string line)
{
	// convert the line entered into a vector of arguments
	vector<string> args;
	size_t  start = 0, end = 0;    
	while ( end != string::npos )   
	{     
		end = line.find( " ", start );
		// If at end, use length=maxLength.  Else use length=end-start.     
		args.push_back( line.substr( start, (end == string::npos) ? string::npos : end - start ) );
		// If at end, use start=maxSize.  Else use start=end+delimiter.     
		start = (   ( end > (string::npos - 1) )               
			?  string::npos  :  end + 1    );   
	}
	return args;
}

//
// Process a command that was entered on the command line, or from the startup
// file, or some other source
//
void ProcessCommand(vector<string> args)
{
	// Just show the user the help text
	if ( args[0].compare("help") == 0 )
	{
		// print out the help
		cout << "Available commands: " << endl;
		cout << "  help" << endl;
		cout << "  start [pleo,camera,planner]" << endl;
		cout << "  [pleo,camera,planner] [behavior] [argument]" << endl;
		cout << "  quit" << endl;
	}
	else if ( args[0].compare("start") == 0 )
	{
		// Here we need to start a subsystem. Pretty simple, just create the subsystem object,
		// register it with the system queue, tell it to start, then add it to our map<> of
		// active systems.
		if ( args[1].compare("camera") == 0 )
		{
			m_systemQueue->RegisterSystem("camera");
			CameraSubsystem *camera = new CameraSubsystem();
			camera->Start("camera",m_systemQueue);
			activeSystems["camera"] = camera;
		}
		else if ( args[1].compare("pleo") == 0 )
		{
			m_systemQueue->RegisterSystem("pleo");
			PleoSubsystem *pleo = new PleoSubsystem();
			pleo->Start("pleo",m_systemQueue);
			activeSystems["pleo"] = pleo;
		}
		else if ( args[1].compare("planner") == 0 )
		{
			m_systemQueue->RegisterSystem("planner");
			PlannerSubsystem *planner = new PlannerSubsystem();
			planner->Start("planner",m_systemQueue);
			activeSystems["planner"] = planner;
		}
		else if ( args[1].compare("monitor") == 0 )
		{
			m_systemQueue->RegisterSystem("monitor");
			MonitorSubsystem *monitor = new MonitorSubsystem();
			monitor->Start("monitor",m_systemQueue);
			activeSystems["monitor"] = monitor;
		}
	}
	else if ( args[0].compare("pleo") == 0 )
	{
		// user wants to send a message to the pleo subsystem
		if ( activeSystems["pleo"] )
		{
			m_systemQueue->PushMessage("pleo",args);
		}
	}
	else if ( args[0].compare("camera") == 0 )
	{
		// user wants to send a message to the camera subsystem
		if ( activeSystems["camera"] )
		{
			m_systemQueue->PushMessage("camera",args);
		}
	}
	else if ( args[0].compare("planner") == 0 )
	{
		// user wants to send a message to the camera subsystem
		if ( activeSystems["planner"] )
		{
			m_systemQueue->PushMessage("planner",args);
		}
	}
}

//
// When the app starts up, read in commands from the startup.txt file
// and execute them as if the user had typed them in.
//
void ProcessStartupFile()
{
	string line;
	ifstream myfile ("startup.txt");
	if (myfile.is_open())
	{
		cout << "PROCESSING STARTUP COMMANDS:" << endl;
		while ( myfile.good() )
		{
			getline (myfile,line);
			cout << line << endl;
			vector<string> args = SeparateArguments(line);
			ProcessCommand(args);
		}
	}
	myfile.close();
}

int main(void)
{
	string instr;

	// setup a system queue where our subsystems can communicate in a thread-safe manner
	m_systemQueue = new SystemQueue();

	// process any commands in the startup.txt file
	ProcessStartupFile();

	instr = "";
	while ( true )
	{
		// get a command from the user
		cout << "JPORK>> ";
		getline(cin,instr);

		// convert the line entered into a vector of arguments
		vector<string> args = SeparateArguments(instr);

		if ( args.size() > 0 )
		{
			ProcessCommand(args);

			// Also check for the quit command and if found, just quit the program
			if ( args[0].compare("quit") == 0 )
			{
				// we are quitting the app. But first try to stop all the subsystems cleanly
				cout << "Stopping subsystems... please wait..." << endl;

				// first tell each of the subsystems to stop
				for ( map<string,Subsystem *>::iterator it = activeSystems.begin(); it != activeSystems.end(); ++it )
				{
					string name = (*it).first;
					Subsystem *sys = (*it).second;
					vector<string> quitMsg;
					quitMsg.push_back(name);
					quitMsg.push_back("quit");
					m_systemQueue->PushMessage(name,quitMsg);
				}
				Sleep(400);

				for ( map<string,Subsystem *>::iterator it = activeSystems.begin(); it != activeSystems.end(); ++it )
				{
					string name = (*it).first;
					Subsystem *sys = (*it).second;
					sys->Stop();
				}
				Sleep(100);

				cout << "told all the subsystems to stop. now we wait for them to really stop!" << endl;
				// now we just wait until they have all stopped
				bool stillRunning = false;
				int waitCount = 0;
				do
				{
					for ( map<string,Subsystem *>::iterator it = activeSystems.begin(); it != activeSystems.end(); ++it )
					{
						string name = (*it).first;
						Subsystem *sys = (*it).second;
						if ( sys->IsRunning() == true )
						{
							stillRunning = true;
						}
					}
					
					waitCount++;
					if ( waitCount > 1000 )
					{
						stillRunning = false;
					}

				} while (stillRunning == true);

				cout << "all stopped. bye!" << endl;

				// we are done
				break;
			}
		}
	}

	return 0;
}

