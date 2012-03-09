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
#include "ColorBlobDetector.h"
#include "LocsCalc.h"
#include <vector>
using namespace std;

class CameraSubsystem :
	public Subsystem
{
public:
	CameraSubsystem(void);
	~CameraSubsystem(void);
	void Tick(size_t tick);
	void Execute(string behavior, string argument);
	void Shutdown();
	string MonitorMessage();
	int HttpFetch( const char *url, const char *filename );
private:
	bool m_httpcapture;
	bool m_localcapture;
	CvCapture *m_capture;
	vector<vector<int>> m_expectedPath;
	vector<vector<int>> m_locations;
	ColorBlobDetector cbd;
	LocsCalc lc;
};
