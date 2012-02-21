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
#include "PleoSubsystem.h"
#include "SerialTerm.h"
#include <iostream>
#include <fstream>
using namespace std;

PleoSubsystem::PleoSubsystem(void)
{

	m_logFile = new Logger("pleo.log");
	m_logFile->out("Pleo Subsystem starting up...");

	// initialize all the joints
	m_jointData[0] = 'm';
	for ( int i = 1; i < 15; i++ )
	{
		m_actuatorValues[i-1] = 0;
		m_jointData[i] = _angle(0);
	}
	m_jointData[15] = '!';

	// initialize a few other variables
	m_lastSensorTick = 0;
	m_currentBehavior = "";
	m_status = "";
	m_currentMovement = -1;
	m_repeatMovement = false;
	m_movementSpeed = 1;
	m_lastMovementTick = 0;
	m_movementDelta = 0;
	m_lastMonitorTick = 0;
	m_moving = false;
	m_tickSleep = 100;

	// open up the serial port connection to the robot!
	serialterm_init(PLEO_PORT,m_logFile);

	// find the ranges of all the joints
	m_logFile->out("joint range data");
	char jointranges[512];
	int jointrange_size = 0;
	char data[4];
	memset(jointranges,0,512);
	sprintf_s(data, "r!");
	serialterm_send(data, 2);
	serialterm_read(jointranges, &jointrange_size); 
	if ( jointrange_size > 0 )
	{
		m_logFile->out("%d %s",jointrange_size,jointranges);

		vector<string> ranges;
		string range(jointranges);
		size_t  start = 0, end = 0;    
		while ( end != string::npos )   
		{     
			end = range.find( ",", start );
			// If at end, use length=maxLength.  Else use length=end-start.     
			ranges.push_back( range.substr( start, (end == string::npos) ? string::npos : end - start ) );
			// If at end, use start=maxSize.  Else use start=end+delimiter.     
			start = (   ( end > (string::npos - 1) )               
						?  string::npos  :  end + 1    ); 
		} 
		m_logFile->out("%d joint ranges",ranges.size());

		int pos = 0;
		for ( int i = 0; i < MAX_JOINTS; i++ )
		{
			m_jointMin[i] = atoi(ranges[pos].c_str());
			m_jointMedian[i] = atoi(ranges[pos+1].c_str());
			m_jointMax[i] = atoi(ranges[pos+2].c_str());
			pos += 3;
			m_logFile->out("Joint %d  min %d median %d max %d",i,m_jointMin[i],m_jointMedian[i],m_jointMax[i]);
		}
	}	
	else
	{
		m_logFile->out("ERROR getting joint ranges");
	}

	// open up the status window
	m_windowOpened = false;
	fontTitles = new CvFont();
	fontText = new CvFont();
	cvInitFont(fontTitles, CV_FONT_HERSHEY_TRIPLEX, .4, .4, 0, 1, CV_AA);
	cvInitFont(fontText, CV_FONT_HERSHEY_PLAIN, .7, .7, 0, 1, CV_AA);
}

PleoSubsystem::~PleoSubsystem(void)
{
	serialterm_wait();
	serialterm_close();
	delete m_logFile;
}

void PleoSubsystem::Shutdown()
{
}

void PleoSubsystem::Tick(size_t tick)
{
	//m_logFile->out("Pleo Subsystem tick at %d",tick);

	// see if we should check the sensors
	if ( tick > (m_lastSensorTick + TICKS_PER_SENSOR_READING) )
	{
		// check the sensor values
		CheckSensors();			
		// check the actual values from Pleo
//		CheckCurrentActuators();
//		m_lastSensorTick = tick;
	}

	// check for any movements that are queued up
	if ( tick > (m_lastMovementTick + m_movementSpeed) )
	{
		if ( m_currentMovement != -1 )
		{
			// if the robot is moving, check our deltas and wait to issue the next
			// command until we have reached our destination
			if ( m_moving == true )
			{
				// calculate a delta from the values we are trying to get pleo to move to
				// versues what we just got back from Pleo
//				CheckCurrentActuators();
//				CalculateMovementDelta();
				m_logFile->out("movement delta: %d",m_movementDelta);
				m_movementDelta = 0;

				// if we are done, then move on to the next set of joints
				if ( m_movementDelta < 1 )
				{
					m_moving = false;

					// Move to the next movement & check for the end of the movements. Repeat if specified...
					m_currentMovement++;
					m_logFile->out("currentMovement %d total %d",m_currentMovement,m_movements.size());
					if ( m_currentMovement >= m_movements.size() )
					{
						if ( m_repeatMovement == true )
						{
							m_currentMovement = 0;
						}
						else
						{
							// We have finished all the movements for this behavior, so we need to reset
							// some interal variables, and then send out a notification to the other subsystems
							vector<string> msg;
							msg.push_back("pleo");
							msg.push_back("pleoFinishedBehavior");
							msg.push_back(m_currentBehavior);
							SendNotification(msg);
							m_currentMovement = -1;
							m_currentBehavior = "";
						}
					}
				}
				else
				{
					// delta still high. reissue command
					SendMovementData();
				}
			}
			else
			{
				// Send movement data to the robot
				SendMovementData();
			}
		}
		m_lastMovementTick = tick;
	}

	UpdateStatusDisplay();
}

void PleoSubsystem::Execute(string behavior, string argument)
{
	// assume argument is just a string for now
	m_logFile->out("behavior: %s arg: %s",behavior.c_str(),argument.c_str());

	m_currentBehavior = behavior;

	if ( behavior.compare("openEyes") == 0 )
	{
		// move the head joint all the way up
		ClearMovements();
		AddMovement(JOINT_HEAD,-25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("closeEyes") == 0 )
	{
		// move the head joint all the way up
		ClearMovements();
		AddMovement(JOINT_HEAD,25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("liftHead") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_VERTICAL,25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("lowerHead") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_VERTICAL,-25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("headLeft") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_HORIZONTAL,-25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("headRight") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_HORIZONTAL,25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("lookLeftAndRight") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_HORIZONTAL,-25,5);
		AddMovement(JOINT_NECK_HORIZONTAL,0,5);
		AddMovement(JOINT_NECK_HORIZONTAL,25,5);
		AddMovement(JOINT_NECK_HORIZONTAL,0,5);
		StartMovement(false);
	}
	else if ( behavior.compare("neckTest") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_HORIZONTAL,-25,5);
		AddMovement(JOINT_NECK_HORIZONTAL,25,5);
		AddMovement(JOINT_NECK_HORIZONTAL,0,5);
		AddMovement(JOINT_NECK_VERTICAL,-25,5);
		AddMovement(JOINT_NECK_VERTICAL,25,5);
		AddMovement(JOINT_NECK_VERTICAL,0,5);
		StartMovement(false);
	}
	else if ( behavior.compare("centerHead") == 0 )
	{
		ClearMovements();
		vector<int> joints;
		vector<int> angles;

		joints.push_back(JOINT_NECK_VERTICAL);
		angles.push_back(0);
		joints.push_back(JOINT_NECK_HORIZONTAL);
		angles.push_back(0);
		AddMultiMovement(joints,angles,1);

		StartMovement(false);
	}
	else if ( behavior.compare("liftTail") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_TAIL_VERTICAL,25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("lowerTail") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_TAIL_VERTICAL,-25,1);
		StartMovement(false);
	}
	else if ( behavior.compare("normalize") == 0 )
	{
		ClearMovements();

		vector<int> joints;
		vector<int> angles;

		for ( int i = 0; i < MAX_JOINTS; i++ )
		{
			joints.push_back(i);
			angles.push_back(0);
		}
		AddMultiMovement(joints,angles,1);

		StartMovement(false);
	}
	else if ( behavior.compare("wagTail") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_TAIL_HORIZONTAL,-40,3);
		AddMovement(JOINT_TAIL_HORIZONTAL,0,3);
		AddMovement(JOINT_TAIL_HORIZONTAL,40,3);
		AddMovement(JOINT_TAIL_HORIZONTAL,0,3);
		StartMovement(true);
	}
	else if ( behavior.compare( "frontTest") == 0 )
	{
		ClearMovements();
		
		vector<int> joints1;
		vector<int> angles1;
		joints1.push_back(JOINT_RIGHT_SHOULDER);
		angles1.push_back(45);
		joints1.push_back(JOINT_RIGHT_ELBOW);
		angles1.push_back(35);
		AddMultiMovement(joints1,angles1,5);

		vector<int> joints2;
		vector<int> angles2;
		joints2.push_back(JOINT_RIGHT_SHOULDER);
		angles2.push_back(0);
		joints2.push_back(JOINT_RIGHT_ELBOW);
		angles2.push_back(0);
		AddMultiMovement(joints2,angles2,5);

		vector<int> joints3;
		vector<int> angles3;
		joints3.push_back(JOINT_LEFT_SHOULDER);
		angles3.push_back(45);
		joints3.push_back(JOINT_LEFT_ELBOW);
		angles3.push_back(35);
		AddMultiMovement(joints3,angles3,5);

		vector<int> joints4;
		vector<int> angles4;
		joints4.push_back(JOINT_LEFT_SHOULDER);
		angles4.push_back(0);
		joints4.push_back(JOINT_LEFT_ELBOW);
		angles4.push_back(0);
		AddMultiMovement(joints4,angles4,5);

		StartMovement(false);
	}
	else if ( behavior.compare( "backTest") == 0 )
	{
		ClearMovements();
		
		vector<int> joints1;
		vector<int> angles1;
		joints1.push_back(JOINT_RIGHT_HIP);
		angles1.push_back(0);
		joints1.push_back(JOINT_RIGHT_KNEE);
		angles1.push_back(-35);
		AddMultiMovement(joints1,angles1,5);

		vector<int> joints2;
		vector<int> angles2;
		joints2.push_back(JOINT_RIGHT_HIP);
		angles2.push_back(0);
		joints2.push_back(JOINT_RIGHT_KNEE);
		angles2.push_back(0);
		AddMultiMovement(joints2,angles2,5);

		vector<int> joints3;
		vector<int> angles3;
		joints3.push_back(JOINT_LEFT_HIP);
		angles3.push_back(0);
		joints3.push_back(JOINT_LEFT_KNEE);
		angles3.push_back(-35);
		AddMultiMovement(joints3,angles3,5);

		vector<int> joints4;
		vector<int> angles4;
		joints4.push_back(JOINT_LEFT_HIP);
		angles4.push_back(0);
		joints4.push_back(JOINT_LEFT_KNEE);
		angles4.push_back(0);
		AddMultiMovement(joints4,angles4,5);

		StartMovement(false);
	}
	else if ( behavior.compare("raiseRightLeg") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_RIGHT_SHOULDER,15,1);
		StartMovement(false);
	}
	else if ( behavior.compare("lowerRightLeg") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_RIGHT_SHOULDER,0,1);
		StartMovement(false);
	}
	else if ( behavior.compare("raiseLeftLeg") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_LEFT_SHOULDER,45,1);
		StartMovement(false);
	}
	else if ( behavior.compare("lowerLeftLeg") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_LEFT_SHOULDER,0,1);
		StartMovement(false);
	}
	else if ( behavior.compare("bendRightKnee") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_RIGHT_KNEE,-45,1);
		StartMovement(false);
	}
	else if ( behavior.compare("sit") == 0 )
	{
		ClearMovements();

		vector<int> joints;
		vector<int> angles;

		joints.push_back(JOINT_RIGHT_KNEE);
		angles.push_back(-45);

		joints.push_back(JOINT_LEFT_KNEE);
		angles.push_back(-45);

		joints.push_back(JOINT_RIGHT_HIP);
		angles.push_back(45);

		joints.push_back(JOINT_LEFT_HIP);
		angles.push_back(45);

		AddMultiMovement(joints,angles,1);

		StartMovement(false);
	}
	else if ( behavior.compare("bite") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_HEAD,-45,10);
		AddMovement(JOINT_HEAD,0,10);
		StartMovement(false);
	}
	else if ( behavior.compare("yawn") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_NECK_VERTICAL,45,5);
		AddMovement(JOINT_HEAD,75,5);
		AddMovement(JOINT_HEAD,0,5);
		AddMovement(JOINT_NECK_VERTICAL,0,5);
		StartMovement(false);
	}
	else if ( behavior.compare("stop") == 0 )
	{
		m_currentMovement = -1;
		m_currentBehavior = "";
	}
	else if ( behavior.compare("motion") == 0 )
	{
		ClearMovements();
		AddMovementFromFile(argument);
		StartMovement(false);
	}
	else if ( behavior.compare("hump") == 0 )
	{
		ClearMovements();

		AddMovement(JOINT_TORSO,35,3);
		AddMovement(JOINT_TORSO,0,3);
		AddMovement(JOINT_TORSO,-35,3);
		AddMovement(JOINT_TORSO,0,3);

		StartMovement(true);
	}
	else if ( behavior.compare("aww") == 0 )
	{
		ClearMovements();

		vector<int> joints;
		vector<int> angles;

		joints.push_back(JOINT_NECK_VERTICAL);
		angles.push_back(-25);

		joints.push_back(JOINT_HEAD);
		angles.push_back(45);

		AddMultiMovement(joints,angles,1);

		StartMovement(false);
	}
	else if ( behavior.compare("party") == 0 )
	{
		ClearMovements();

		vector<int> joints;
		vector<int> angles;

		AddMovement(JOINT_NECK_VERTICAL,-15,1);
		AddMovement(JOINT_NECK_VERTICAL,0,1);

		StartMovement(true);
	}
	else if ( behavior.compare("huh") == 0 )
	{
		ClearMovements();

		vector<int> joints;
		vector<int> angles;

		joints.push_back(JOINT_NECK_VERTICAL);
		angles.push_back(15);
		joints.push_back(JOINT_NECK_HORIZONTAL);
		angles.push_back(20);
		joints.push_back(JOINT_HEAD);
		angles.push_back(-45);

		AddMultiMovement(joints,angles,1);

		StartMovement(false);
	}
	else if ( behavior.compare("moo") == 0 )
	{
		int sensor_size = 0;
		char data[4] = {0};
		sprintf_s(data, "a1!");
		serialterm_send(data, 3);
	}
	else if ( behavior.compare("growl") == 0 )
	{
		int sensor_size = 0;
		char data[4] = {0};
		sprintf_s(data, "a2!");
		serialterm_send(data, 3);
	}
	else if ( behavior.compare("howl") == 0 )
	{
		int sensor_size = 0;
		char data[4] = {0};
		sprintf_s(data, "a3!");
		serialterm_send(data, 3);
	}
	else if ( behavior.compare("torsoRight") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_TORSO,15,1);
		StartMovement(false);
	}
	else if ( behavior.compare("torsoLeft") == 0 )
	{
		ClearMovements();
		AddMovement(JOINT_TORSO,-15,1);
		StartMovement(false);
	}
	else if ( behavior.compare("turnXRight") == 0 )
	{
		ClearMovements();

		//
		// Left Shoulder forward
		// Right Hip forward
		AddMultiMovement(1,10,JOINT_RIGHT_SHOULDER,-15,
							 JOINT_RIGHT_ELBOW,15,
							 JOINT_RIGHT_HIP,15,
							 JOINT_RIGHT_KNEE,-15,

							 JOINT_LEFT_SHOULDER,15,
							 JOINT_LEFT_ELBOW,25,
							 JOINT_LEFT_HIP,-15,
							 JOINT_LEFT_KNEE,0,
							
							 JOINT_NECK_HORIZONTAL,45,

							 JOINT_TORSO,15);

		//
		// Right hip stomp
		// Left front stomp
		AddMultiMovement(1,10,JOINT_RIGHT_SHOULDER,-15,
							 JOINT_RIGHT_ELBOW,15,
							 JOINT_RIGHT_HIP,15,
							 JOINT_RIGHT_KNEE,-15,

							 JOINT_LEFT_SHOULDER,15,
							 JOINT_LEFT_ELBOW,0,
							 JOINT_LEFT_HIP,-15,
							 JOINT_LEFT_KNEE,0,
							
							 JOINT_NECK_HORIZONTAL,45,

							 JOINT_TORSO,15);

		// Right Shoulder forward
		// Left Hip forward
		AddMultiMovement(1,10,JOINT_RIGHT_SHOULDER,-15,
							 JOINT_RIGHT_ELBOW,0,
							 JOINT_LEFT_SHOULDER,-15,
							 JOINT_LEFT_ELBOW,0,

							 JOINT_LEFT_HIP,-15,
							 JOINT_LEFT_KNEE,-45,
							 JOINT_RIGHT_HIP,15,
							 JOINT_RIGHT_KNEE,0,
							 
							 JOINT_NECK_HORIZONTAL,45,
							 
							 JOINT_TORSO,0);

		AddMultiMovement(1,10,JOINT_RIGHT_SHOULDER,-15,
							 JOINT_RIGHT_ELBOW,0,
							 JOINT_RIGHT_HIP,15,
							 JOINT_RIGHT_KNEE,0,

							 JOINT_LEFT_SHOULDER,-15,
							 JOINT_LEFT_ELBOW,0,
							 JOINT_LEFT_HIP,15,
							 JOINT_LEFT_KNEE,-45,
							
							 JOINT_NECK_HORIZONTAL,45,

							 JOINT_TORSO,0);

		//
		// Left hip stomp
		// Right front stomp
		AddMultiMovement(1,10,JOINT_RIGHT_SHOULDER,-15,
							 JOINT_RIGHT_ELBOW,15,
							 JOINT_RIGHT_HIP,15,
							 JOINT_RIGHT_KNEE,-15,

							 JOINT_LEFT_SHOULDER,-15,
							 JOINT_LEFT_ELBOW,0,
							 JOINT_LEFT_HIP,15,
							 JOINT_LEFT_KNEE,0,

							 JOINT_NECK_HORIZONTAL,45,

							 JOINT_TORSO,0);

		StartMovement(false);
	}
	else if ( behavior.compare("turnRightOLD") == 0 )
	{
		ClearMovements();

		int cycles = atoi(argument.c_str());

		//if no parameter passed, default to 1
		cycles = (cycles == 0) ? 1 : cycles;
		
		//needs elbow joint for right arm?
		for ( int i = 0; i < cycles; i++ )
		{

			//step 1

			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,5,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,-45,

								 JOINT_NECK_HORIZONTAL,45,

								 JOINT_TORSO,15);
			//step 2


			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,5,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,35,
								 JOINT_LEFT_KNEE,-45,

								 JOINT_NECK_HORIZONTAL,45,

								 JOINT_TORSO,15);

			//step 3



			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,5,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,30,
								 JOINT_LEFT_HIP,35,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,45,
								 
								 JOINT_TORSO,15);
			//step 4



			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,5,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,30,
								 JOINT_LEFT_HIP,35,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,45,
								 
								 JOINT_TORSO,15);

			//step 5

			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,25,
								 JOINT_RIGHT_HIP,15,
								 JOINT_RIGHT_KNEE,-15,

								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,35,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,45,
								 
								 JOINT_TORSO,15);


			//step 6

			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,25,
								 JOINT_RIGHT_HIP,5,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,45,
								 
								 JOINT_TORSO,15);




		}
		StartMovement(false);
	}
	else if ( behavior.compare("walkForward") == 0 )
	{
		int cycles = atoi(argument.c_str());	
		cycles = (cycles == 0) ? 1 : cycles;

		walkForward(cycles);		
	}
	else if ( behavior.compare("turnRight") == 0 )
	{
		ClearMovements();
		int cycles = atoi(argument.c_str());\
		cycles = (cycles == 0) ? 1 : cycles;	
		turnRight(cycles);
	}
	else if ( behavior.compare("turnRightHard") == 0 )
	{
		ClearMovements();
		int cycles = atoi(argument.c_str());
		cycles = (cycles == 0) ? 1 : cycles;	
		turnRightHard(cycles);
	}
	else if ( behavior.compare("turnLeft") == 0 )
	{
		ClearMovements();
		int cycles = atoi(argument.c_str());
		cycles = (cycles == 0) ? 1 : cycles;
		turnLeft(cycles);
	}
	else if ( behavior.compare("turnLeftHard") == 0 )
	{
		ClearMovements();
		int cycles = atoi(argument.c_str());
		cycles = (cycles == 0) ? 1 : cycles;	
		turnLeftHard(cycles);
	}
	else if ( behavior.compare("walkForwardOLD") == 0 )
	{
		ClearMovements();
	
		int cycles = atoi(argument.c_str());

		//if no parameter passed, default to 1
		cycles = (cycles == 0) ? 1 : cycles;


		for ( int i = 0; i < cycles; i++ )
		{

			//throwing head is necessary to limit hind leg scraping
			//on carpet - head balances over forward-moving hand and pivots
			//body, lifting opposite hind leg as it is coming forward

			//right shoulder goes further than left forward to help counter
			//listing to the left

			// Left Shoulder forward
			// Right Hip forward
			//bend right knee to prevent scraping
			//AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
			//					 JOINT_RIGHT_ELBOW,0,
			//					 JOINT_RIGHT_HIP,-10,
			//					 JOINT_RIGHT_KNEE,-25,

			//					 JOINT_LEFT_SHOULDER,8,
			//					 JOINT_LEFT_ELBOW,8,
			//					 JOINT_LEFT_HIP,-10,
			//					 JOINT_LEFT_KNEE,0,

			//					 JOINT_NECK_HORIZONTAL,-15,

			//					 JOINT_TORSO,-10);

			//
			// Left Shoulder forward
			// Right Hip forward
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_RIGHT_HIP, 25,
								 JOINT_RIGHT_KNEE,-35,

								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,-15,

								 JOINT_TORSO, torsoAngle);

			//
			// Right hip stomp
			// Left front stomp
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,15,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,-5,

								 JOINT_TORSO, torsoAngle);


			// Bend left knee to get out of the way
			// and prevent scraping on carpet
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,0,
								 JOINT_RIGHT_ELBOW,15,
								 JOINT_RIGHT_HIP,-15,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,-35,

								 JOINT_NECK_HORIZONTAL,25,

								 JOINT_TORSO, torsoAngle);


			// Right Shoulder forward
			// Left Hip forward
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_RIGHT_HIP,-15,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,25,
								 JOINT_LEFT_KNEE,-40,

								 JOINT_NECK_HORIZONTAL,25,

								 JOINT_TORSO, torsoAngle);

			//
			// Left hip stomp
			// Right front stomp
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_RIGHT_HIP,-15,
								 JOINT_RIGHT_KNEE,0,

								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,15,
								 JOINT_LEFT_HIP,15,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,5,

								 JOINT_TORSO, torsoAngle);

			// Left Shoulder forward
			// Right Hip forward
			//bend right knee to prevent scraping
			AddMultiMovement(delay,10,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_RIGHT_HIP,-15,
								 JOINT_RIGHT_KNEE,-35,

								 JOINT_LEFT_SHOULDER,0,
								 JOINT_LEFT_ELBOW,15,
								 JOINT_LEFT_HIP,-15,
								 JOINT_LEFT_KNEE,0,

								 JOINT_NECK_HORIZONTAL,-15,

								 JOINT_TORSO, torsoAngle);

		}



		StartMovement(false);
	}
	else if ( behavior.compare("walk2Forward2") == 0 )
	{
		ClearMovements();

		int cycles = atoi(argument.c_str());

		//if no parameter passed, default to 1
		cycles = (cycles == 0) ? 1 : cycles;


		for ( int i = 0; i < cycles; i++ )
		{
			// old walk

			// Left Shoulder forward
			// Right Hip forward
			AddMultiMovement(1,9,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,25,
								 JOINT_LEFT_HIP,0,
								 JOINT_LEFT_KNEE,0,
								 JOINT_RIGHT_HIP,15,
								 JOINT_RIGHT_KNEE,-35,
								 JOINT_TORSO,15);

			//
			// Right hip stomp
			// Left front stomp
			AddMultiMovement(1,9,JOINT_RIGHT_SHOULDER,-15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_LEFT_SHOULDER,15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,0,
								 JOINT_LEFT_KNEE,0,
								 JOINT_RIGHT_HIP,15,
								 JOINT_RIGHT_KNEE,0,
								 JOINT_TORSO,15);

			// Right Shoulder forward
			// Left Hip forward
			AddMultiMovement(1,9,JOINT_RIGHT_SHOULDER,15,
								 JOINT_RIGHT_ELBOW,25,
								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,15,
								 JOINT_LEFT_KNEE,-35,
								 JOINT_RIGHT_HIP,0,
								 JOINT_RIGHT_KNEE,0,
								 JOINT_TORSO,-15);

			//
			// Left hip stomp
			// Right front stomp
			AddMultiMovement(1,9,JOINT_RIGHT_SHOULDER,15,
								 JOINT_RIGHT_ELBOW,0,
								 JOINT_LEFT_SHOULDER,-15,
								 JOINT_LEFT_ELBOW,0,
								 JOINT_LEFT_HIP,15,
								 JOINT_LEFT_KNEE,0,
								 JOINT_RIGHT_HIP,0,
								 JOINT_RIGHT_KNEE,0,
								 JOINT_TORSO,-15);
		}



		StartMovement(false);
	}
	else if ( behavior.compare("moveRightShoulder") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_RIGHT_SHOULDER,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveRightElbow") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_RIGHT_ELBOW,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveLeftShoulder") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_LEFT_SHOULDER,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveLeftElbow") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_LEFT_ELBOW,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveLeftHip") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_LEFT_HIP,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveLeftKnee") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_LEFT_KNEE,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveRightHip") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_RIGHT_HIP,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveRightKnee") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_RIGHT_KNEE,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveTorso") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_TORSO,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveTailHorizontal") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_TAIL_HORIZONTAL,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveTailVertical") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_TAIL_VERTICAL,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveNeckHorizontal") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_NECK_HORIZONTAL,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveNeckVertical") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_NECK_VERTICAL,angle,1);
		StartMovement(false);
	}
	else if ( behavior.compare("moveHead") == 0 )
	{
		ClearMovements();
		int angle = atoi(argument.c_str());
		AddMovement(JOINT_HEAD,angle,1);
		StartMovement(false);
	}
}


void PleoSubsystem::UpdateStatusDisplay()
{
	int yPos;

	if ( m_windowOpened == false )
	{
		cvNamedWindow("PLEOStatus");
		m_windowOpened = true;
	}

	IplImage *statusImage = cvLoadImage("PleoHeader.jpg");
	if ( !statusImage )
		m_logFile->out("problem loading status image");
	
	// output Actuator stuff
	cvPutText(statusImage,"Actuators", cvPoint(20,220), fontTitles, cvScalar(0,0,0, 0));

	yPos = 230;
	ActuatorOutput(statusImage,"RIGHT SHOULDER",0,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"RIGHT ELBOW",1,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"LEFT SHOULDER",2,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"LEFT ELBOW",3,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"LEFT HIP",4,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"LEFT KNEE",5,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"RIGHT HIP",6,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"RIGHT KNEE",7,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"TORSO",8,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"TAIL HORIZONTAL",9,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"TAIL VERTICAL",10,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"NECK HORIZONTAL",11,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"NECK VERTICAL",12,yPos);
	yPos += 10;
	ActuatorOutput(statusImage,"HEAD",13,yPos);
	yPos += 10;

	// output Sensor stuff
	cvPutText(statusImage,"Sensors",cvPoint(320,220),fontTitles,cvScalar(0,0,0,0));

	yPos = 230;
	SensorOutput(statusImage,"HEAD",SENSOR_HEAD,yPos);
	yPos += 10;
	SensorOutput(statusImage,"CHIN",SENSOR_CHIN,yPos);
	yPos += 10;
	SensorOutput(statusImage,"BACK",SENSOR_BACK,yPos);
	yPos += 10;
	SensorOutput(statusImage,"LEFT LEG",SENSOR_LEFT_LEG,yPos);
	yPos += 10;
	SensorOutput(statusImage,"RIGHT LEG",SENSOR_RIGHT_LEG,yPos);
	yPos += 10;
	SensorOutput(statusImage,"LEFT ARM",SENSOR_LEFT_ARM,yPos);
	yPos += 10;
	SensorOutput(statusImage,"RIGHT ARM",SENSOR_RIGHT_ARM,yPos);
	yPos += 10;
	SensorOutput(statusImage,"TAIL",SENSOR_TAIL,yPos);

	cvPutText(statusImage,"Behavior: ", cvPoint(20,420), fontTitles, cvScalar(0,0,0, 0));
	cvPutText(statusImage,m_currentBehavior.c_str(),cvPoint(20,430),fontText,cvScalar(0,0,0,0));

	cvPutText(statusImage,"Status: ", cvPoint(20,440), fontTitles, cvScalar(0,0,0, 0));
	cvPutText(statusImage,m_status.c_str(),cvPoint(20,450),fontText,cvScalar(0,0,0,0));

	cvPutText(statusImage,"Delta: ", cvPoint(20,460), fontTitles, cvScalar(0,0,0, 0));
	char deltaMsg[255];
	sprintf_s(deltaMsg,"%d",m_movementDelta);
	cvPutText(statusImage,deltaMsg,cvPoint(20,470),fontText,cvScalar(0,0,0,0));

	cvShowImage("PLEOStatus",statusImage);
	cvWaitKey(3);
	cvReleaseImage(&statusImage);
}

void PleoSubsystem::SensorOutput(IplImage *image, const char *sensor,int index,int yPos)
{
	char msg[255];
	sprintf_s(msg,"%s: %s",sensor,m_sensorData[index] == 48 ? "OFF" : "ON");
	cvPutText(image,msg,cvPoint(320,yPos),fontText,cvScalar(0,0,0,0));
}

void PleoSubsystem::ActuatorOutput(IplImage *image, const char *actuator, int index, int yPos)
{
	char msg[255];
	sprintf_s(msg,"%s: %d [%d]",actuator,m_actuatorValues[index],m_jointCurrent[index]);
	cvPutText(image,msg,cvPoint(20,yPos),fontText,cvScalar(0,0,0,0));
}


void PleoSubsystem::SetJointAngle(int joint, int angle)
{
	m_actuatorValues[joint] = angle;
	m_jointData[joint+1] = _angle(angle);
}

void PleoSubsystem::ClearMovements()
{
	m_movements.clear();
}

void PleoSubsystem::AddMovement(int joint, int angle,int duration)
{
	for ( int i = 0; i < duration; i++ )
	{
		vector<int> move;
		move.push_back(joint);
		move.push_back(angle);
		m_movements.push_back(move);
	}
}

void PleoSubsystem::AddMultiMovement(vector<int> joints, vector<int> angles,int duration)
{
	for ( int i = 0; i < duration; i++ )
	{
		vector<int> move;
		for ( size_t t = 0; t < joints.size(); t++ )
		{
			move.push_back(joints[t]);
			move.push_back(angles[t]);
		}
		m_movements.push_back(move);
	}
}

void PleoSubsystem::AddMultiMovement(int duration,int count,...)
{
	vector<int> move;
	va_list listPtr;
	va_start(listPtr,count);

	for ( int i = 0; i < count; i++ )
	{
		int joint = va_arg(listPtr,int);
		int angle = va_arg(listPtr,int);
		
		move.push_back(joint);
		move.push_back(angle);
	}

	for ( int x = 0; x < duration; x++ )
	{
		vector<int> newMove = move;
		m_movements.push_back(newMove);
	}


	va_end(listPtr);
}

void PleoSubsystem::AddMovementFromFile(string movementfile)
{
	string line;
	ifstream myfile (movementfile.c_str());
	if (myfile.is_open())
	{
		// ignore the first 7 lines except for the 2nd one
		getline (myfile,line);
		getline (myfile,line);
		m_movementSpeed = atoi(line.c_str());
		getline (myfile,line);
		getline (myfile,line);
		getline (myfile,line);
		getline (myfile,line);
		getline (myfile,line);

		while ( myfile.good() )
		{
			getline (myfile,line);

			vector<string> args;
			size_t  start = 0, end = 0;    
			while ( end != string::npos )   
			{     
				end = line.find( ",", start );
				// If at end, use length=maxLength.  Else use length=end-start.     
				args.push_back( line.substr( start, (end == string::npos) ? string::npos : end - start ) );
				// If at end, use start=maxSize.  Else use start=end+delimiter.     
				start = (   ( end > (string::npos - 1) )               
					?  string::npos  :  end + 1    );   
			} 

			m_logFile->out("args found on line: %d",args.size());

			if ( args.size() == 17 )
			{
				vector<int> joints;
				vector<int> angles;

				angles.push_back(atoi(args[2].c_str()));
				joints.push_back(JOINT_NECK_VERTICAL);

				angles.push_back(atoi(args[3].c_str()));
				joints.push_back(JOINT_NECK_HORIZONTAL);

				angles.push_back(atoi(args[4].c_str()));
				joints.push_back(JOINT_HEAD);

				angles.push_back(atoi(args[5].c_str()));
				joints.push_back(JOINT_LEFT_SHOULDER);

				angles.push_back(atoi(args[6].c_str()));
				joints.push_back(JOINT_LEFT_ELBOW);

				angles.push_back(atoi(args[7].c_str()));
				joints.push_back(JOINT_LEFT_HIP);

				angles.push_back(atoi(args[8].c_str()));
				joints.push_back(JOINT_LEFT_KNEE);

				angles.push_back(atoi(args[9].c_str()));
				joints.push_back(JOINT_RIGHT_SHOULDER);

				angles.push_back(atoi(args[10].c_str()));
				joints.push_back(JOINT_RIGHT_ELBOW);

				angles.push_back(atoi(args[11].c_str()));
				joints.push_back(JOINT_RIGHT_HIP);

				angles.push_back(atoi(args[12].c_str()));
				joints.push_back(JOINT_RIGHT_KNEE);

				angles.push_back(atoi(args[13].c_str()));
				joints.push_back(JOINT_TORSO);

				angles.push_back(atoi(args[14].c_str()));
				joints.push_back(JOINT_TAIL_HORIZONTAL);

				angles.push_back(atoi(args[15].c_str()));
				joints.push_back(JOINT_TAIL_VERTICAL);

				AddMultiMovement(joints,angles,1);
			}
		}
		myfile.close();
	}
}

void PleoSubsystem::StartMovement(bool repeat)
{
	m_currentMovement = 0;
	m_repeatMovement = repeat;
}

void PleoSubsystem::CheckSensors()
{
	m_logFile->out("checking sensors");
	int sensor_size = 0;
	char data[4] = {0};
	sprintf_s(data, "s!");
	serialterm_send(data, 2);
	serialterm_read(m_sensorData, &sensor_size); 
	if ( sensor_size == 0 )
	{
		m_status = "ERROR: 0 bytes returned from pleo";
	}
	else
		m_status = "OK";
	m_logFile->out("sensor call returned %d characters",sensor_size);
}

void PleoSubsystem::CheckCurrentActuators()
{
	m_logFile->out("joint position data");
	char jointpos[512];
	char data[4] = {0};
	int jointpos_size = 0;
	memset(jointpos,0,512);
	sprintf_s(data, "j!");
	serialterm_send(data, 2);
	serialterm_read(jointpos, &jointpos_size); 
	if ( jointpos_size > 0 )
	{
		m_logFile->out("%d %s",jointpos_size,jointpos);

		vector<string> positions;
		string joint(jointpos);
		size_t  start = 0, end = 0;    
		while ( end != string::npos )   
		{
			end = joint.find( ",", start );
			// If at end, use length=maxLength.  Else use length=end-start.     
			positions.push_back( joint.substr( start, (end == string::npos) ? string::npos : end - start ) );
			// If at end, use start=maxSize.  Else use start=end+delimiter.     
			start = (   ( end > (string::npos - 1) )               
					?  string::npos  :  end + 1    ); 
		}
		m_logFile->out("%d joint positions",positions.size());

		if ( positions.size() >= (MAX_JOINTS*2) )
		{
			int pos = 0;
			for ( int i = 0; i < MAX_JOINTS; i++ )
			{
				m_jointCurrent[i] = atoi(positions[pos].c_str());
				m_jointMoving[i] = atoi(positions[pos+1].c_str());
				pos += 2;
				m_logFile->out("Joint %d  current %d moving %d",i,m_jointCurrent[i],m_jointMoving[i]);
			}
		}
		else
		{
			m_logFile->out("invalid return from j! command");
		}
	}
	else
	{
		m_logFile->out("ERROR getting joint positions");
	}
}

void PleoSubsystem::CalculateMovementDelta()
{
	m_movementDelta = 0;
	for ( int i = 0; i < MAX_JOINTS; i++ )
	{
		if ( abs(m_jointCurrent[i] - m_actuatorValues[i]) > 15 )
		{
			m_movementDelta += 1;
		}
	}
}

string PleoSubsystem::MonitorMessage()
{
	char pleoMsg[1024];
	sprintf_s(pleoMsg,"PLEO,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s~~",
		m_currentBehavior.c_str(),m_status.c_str(),
		m_sensorData[SENSOR_HEAD] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_CHIN] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_BACK] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_LEFT_LEG] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_RIGHT_LEG] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_LEFT_ARM] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_RIGHT_ARM] == 48 ? "OFF" : "ON",
		m_sensorData[SENSOR_TAIL] == 48 ? "OFF" : "ON"		
		);

	string msg = pleoMsg;
	return msg;
}

void PleoSubsystem::SendMovementData()
{
	// Take the values from the movement vector and add it to the array
	vector<int> current = m_movements[m_currentMovement];
	for ( size_t i = 0; i < current.size(); i += 2 )
	{
		SetJointAngle(current[i],current[i+1]);
	}

	// print out the joint data to the log file
	m_logFile->out("Updating joints: ");
	char msg[255];
	sprintf_s(msg,"SENT: %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
		m_jointData[1]-32-45,m_jointData[2]-32-45,m_jointData[3]-32-45,m_jointData[4]-32-45,
		m_jointData[5]-32-45,m_jointData[6]-32-45,m_jointData[7]-32-45,m_jointData[8]-32-45,
		m_jointData[9]-32-45,m_jointData[10]-32-45,m_jointData[11]-32-45,m_jointData[12]-32-45,
		m_jointData[13]-32-45,m_jointData[14]-32-45);
	m_logFile->out("[%s]",msg);

	// send the joint data to the serial port
	int size = 16;
	serialterm_send(m_jointData, size);
	m_moving = true;
	
}


void PleoSubsystem::walkForward(int cycles)
{
	char *command = "motion command walkForward";
	int clen = strlen(command);

	for ( int i = 0; i < cycles; i++ )
	{
		serialterm_send(command, clen);
		Sleep(3250);
	}
}

void PleoSubsystem::turnRight(int cycles)
{
	char *command = "motion command turnRight";
	int clen = strlen(command);

	for ( int i = 0; i < cycles; i++ )
	{
		serialterm_send(command, clen);
		Sleep(3400);
	}
}

void PleoSubsystem::turnRightHard(int cycles)
{
	char *command = "motion command turnRightHard";
	int clen = strlen(command);

	for ( int i = 0; i < cycles; i++ )
	{
		serialterm_send(command, clen);
		Sleep(3100);
	}
}

void PleoSubsystem::turnLeft(int cycles)
{
	char *command = "motion command turnLeft";
	int clen = strlen(command);

	for ( int i = 0; i < cycles; i++ )
	{
		serialterm_send(command, clen);
		Sleep(3500);
	}
}

void PleoSubsystem::turnLeftHard(int cycles)
{
	char *command = "motion command turnLeftHard";
	int clen = strlen(command);

	for ( int i = 0; i < cycles; i++ )
	{
		serialterm_send(command, clen);
		Sleep(3600);
	}
}