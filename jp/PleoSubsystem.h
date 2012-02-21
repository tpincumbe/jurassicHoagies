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
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#define PLEO_PORT "COM4"


static string joint_name_ara[14] = {"Right Shoulder", 
						"Right Elbow", 
						"Left Shoulder",
						"Left Elbow",
						"Left Hip",
						"Left Knee",
					    "Right Hip",
					    "Right Knee",
					    "Torso",
					    "Tail Horizontal",
					    "Tail Vertical",
					    "Neck Horizontal",
					    "Neck Vertical",
						"Head"};


enum sensor_name 
{
    SENSOR_NONE          =    -1, /** name used to specify 'no sensor' */
    SENSOR_MIN           =     0, /** number of the lowest defined sensor (useful for loops) */
    SENSOR_BATTERY       =     2, /** Battery level (0-100) */
    SENSOR_IR            =     3, /** IR data received */
    SENSOR_IR_ACTIVITY   =     4, /** IR has received a code of. DISABLED */
    SENSOR_TOUCH_FIRST   =     6, /** Alias for the first touch sensor */ 
    SENSOR_HEAD          =     6, /** Head touch sensor 0=not touched 1=touched */
    SENSOR_CHIN          =     7, /** Chin touch sensor 0=not touched 1=touched */
    SENSOR_BACK          =     8, /** Back touch sensor 0=not touched 1=touched */
    SENSOR_LEFT_LEG      =     9, /** Left Leg touch sensor 0=not touched 1=touched */
    SENSOR_RIGHT_LEG     =    10, /** Right Leg touch sensor 0=not touched 1=touched */
    SENSOR_LEFT_ARM      =    11, /** Left Arm touch sensor 0=not touched 1=touched */
    SENSOR_RIGHT_ARM     =    12, /** Right Arm touch sensor 0=not touched 1=touched */
    SENSOR_ARSE          =    13, /** Arse touch sensor 0=not touched 1=touched */
    SENSOR_TAIL          =    13, /** Alias for arse touch sensor 0=not touched 1=touched */
    SENSOR_TOUCH_LAST    =    13, /** Alias for the last touch sensor  */
  
    SENSOR_FOOT_FIRST    =    14, /** Alias for the first foot switch */
    SENSOR_FRONT_LEFT    =    14, /** front left foot switch */
    SENSOR_FRONT_RIGHT   =    15, /** front right foot switch */
    SENSOR_BACK_LEFT     =    16, /** back left foot switch */
    SENSOR_BACK_RIGHT    =    17, /** back right foot switch */
    SENSOR_FOOT_LAST     =    17, /** Alias for the last foot switch */

    SENSOR_CARD_DETECT   =    18, /** SD Card is present */
    SENSOR_WRITE_PROTECT =    19, /** SD Cards write protect tab is set (software only)*/
    SENSOR_LEFT_LOUD     =    20, /** Absolute loudness of left microphone*/
    SENSOR_LIGHT         =    21, /** Absolute light light (0-100)*/
    SENSOR_RIGHT_LOUD    =    22, /** Absolute loudness of right microphone*/
    SENSOR_OBJECT        =    23, /** Object in front of Pleo (reflected IR)*/
    SENSOR_MOUTH         =    24, /** Something blocking mouth IR*/
    SENSOR_SOUND_DIR     =    26, /** Sound direction */
    SENSOR_LIGHT_CHANGE  =    27, /** Light level change, lighten or darken*/
    SENSOR_SOUND_LOUD    =    28, /** Absolute loudness of ambient sound (0-100)*/
    SENSOR_TILT          =    29, /** Current orientation (per tilt_name enum)*/
    SENSOR_TERMINAL      =    30, /** line of text from terminal/serial*/
    SENSOR_POWER_DETECT  =    31, /** Is the charger plugged in?*/
    SENSOR_USB_DETECT    =    32, /** Is the USB cable onnected*/
    SENSOR_WAKEUP        =    33, /** Wakeup / Mom button*/
    SENSOR_BATTERY_TEMP  =    34, /** Battery at critical temp?*/
    SENSOR_CHARGER_STATE =    35, /** charger state */
    SENSOR_SHAKE         =    36, /** Shake sensor activated */
    SENSOR_SOUND_LOUD_CHANGE =37, /** 1 = went above trig, 0 is went below aux_trig*/
    SENSOR_BEACON        =    38, /** value = ID of other Pleo*/
    SENSOR_BATTERY_CURRENT =  39, /** electrical current draw from battery*/
    SENSOR_PACKET        =    40, /** virtual sensor to get the packet data from the NXP processor*/
    SENSOR_MAX
};

enum joint_name {
  JOINT_NONE            =    -1,
  JOINT_MIN             =     0,
  JOINT_RIGHT_SHOULDER  =     0,
  JOINT_RIGHT_ELBOW     =     1,
  JOINT_LEFT_SHOULDER   =     2,
  JOINT_LEFT_ELBOW      =     3,
  JOINT_LEFT_HIP        =     4,
  JOINT_LEFT_KNEE       =     5,
  JOINT_RIGHT_HIP       =     6,
  JOINT_RIGHT_KNEE      =     7,
  JOINT_TORSO           =     8,
  JOINT_TAIL_HORIZONTAL =     9,
  JOINT_TAIL_VERTICAL   =    10,
  JOINT_NECK_HORIZONTAL =    11,
  JOINT_NECK_VERTICAL   =    12,
  JOINT_HEAD            =    13,
  JOINT_MAX_PHYSICAL    =    13,
  JOINT_TRANSITION      =    14,
  JOINT_SOUND           =    15,
  JOINT_MAX
};


#define _angle(a) a+32+45
#define MAX_SENSORS 512
#define MAX_JOINTS 14

#define TICKS_PER_SENSOR_READING 5


class PleoSubsystem :
	public Subsystem
{
public:
	PleoSubsystem();
	~PleoSubsystem(void);
	void Tick(size_t tick);
	void Execute(string behavior,string argument);
	void Shutdown();
	string MonitorMessage();
private:
	char m_sensorData[MAX_SENSORS];
	char m_jointData[MAX_JOINTS+2];
	int m_jointMin[MAX_JOINTS];
	int m_jointMedian[MAX_JOINTS];
	int m_jointMax[MAX_JOINTS];
	int m_jointCurrent[MAX_JOINTS];
	int m_jointMoving[MAX_JOINTS];
	int m_actuatorValues[MAX_JOINTS];

	size_t m_lastSensorTick;
	string m_currentBehavior;
	string m_status;
	vector<vector<int>> m_movements;
	size_t m_currentMovement;
	bool m_repeatMovement;
	size_t m_movementSpeed;
	size_t m_lastMovementTick;
	int m_movementDelta;
	bool m_moving;

	CvFont *fontTitles;
	CvFont *fontText;
	bool m_windowOpened;

	void UpdateStatusDisplay();
	void SensorOutput(IplImage *image, const char *sensor,int index,int yPos);
	void ActuatorOutput(IplImage *image, const char *actuator, int index, int yPos);

	void SetJointAngle(int joint, int angle);
	void ClearMovements();
	void AddMovement(int joint, int angle,int duration);
	void AddMultiMovement(vector<int> joints, vector<int> angles,int duration);
	void AddMovementFromFile(string movementfile);
	void StartMovement(bool repeat);
	void AddMultiMovement(int duration,int count,...);
	void CheckSensors();
	void CheckCurrentActuators();
	void CalculateMovementDelta();
	void SendMovementData();

	void walkForward(int cycles);
	void turnRight(int cycles);
	void turnRightHard(int cycles);
	void turnLeft(int cycles);
	void turnLeftHard(int cycles);

	#define delay 1
	#define torsoAngle 0
};
