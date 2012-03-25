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
#include "CameraSubsystem.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include "curl/curl.h"  

CameraSubsystem::CameraSubsystem(void) 
{
	m_logFile = new Logger("camera.log");
	m_logFile->out("Camera Subsystem starting up...");
	m_httpcapture = false;
	m_localcapture = false;
}

CameraSubsystem::~CameraSubsystem(void)
{
}

void CameraSubsystem::Shutdown()
{

}


void CameraSubsystem::Tick(size_t tick)
{
	// if capture is true then we should grab an image from the camera and display it
	if ( m_httpcapture == true )
	{
		HttpFetch("http://localhost:8080/cam_1.jpg","camera.jpg");
		IplImage *img = cvLoadImage("camera.jpg");
		cvShowImage("camera",img);
		cvWaitKey(1);
	}

	if ( m_localcapture == true )
	{
		VideoCapture camera(0);
		Mat camImage;
		camera >> camImage;
		imshow("camera", camImage);
	}
}

void CameraSubsystem::Execute(string behavior, string argument)
{
	// assume argument is just a string for now
	m_logFile->out("behavior: %s arg: %s",behavior.c_str(),argument.c_str());
	if ( behavior.compare("capture") == 0 )
	{
		cvNamedWindow("camera");
		m_httpcapture = true;
	}
	else if ( behavior.compare("localcapture") == 0 )
	{
		int cameraNumber = atoi(argument.c_str());
		m_capture = cvCaptureFromCAM(cameraNumber);
		m_localcapture = true;
	}
	else if ( behavior.compare("startTracking") == 0 )
	{
		if (0 == argument.compare("fruit"))
		{
			lc.resetPath();
			vector<string> actMsg;

			lc.detect(2, m_sysQueue);	
		}else
		{
			//cbd.resetPath(argument);
		}
	}
	else if ( behavior.compare("getBackground") == 0 )
	{
		lc.grabBackground();
	}
	else if ( behavior.compare("getObstacles") == 0 )
	{
		lc.grabObstacles();
	}
	else if ( behavior.compare("quit") == 0 )
	{
		m_logFile->out("shutting down camera");
		if ( m_localcapture == true )
		{
			m_logFile->out("releasing capture");
			cvReleaseCapture(&m_capture);
			m_localcapture = false;
			m_capture = NULL;
		}
		m_logFile->out("finished quitting");
	}
}

int CameraSubsystem::HttpFetch( const char *url, const char *filename ) 
{
	int result;
        CURL *curl = curl_easy_init();  
        int do_file = filename && strlen(filename) >= 1;
        
        FILE * fptr;
        if( do_file ) {
            fptr = fopen( filename, "wb" );
        }

	curl_easy_setopt(curl, CURLOPT_USERAGENT, "Mozilla/5.0 (Windows; U; Windows NT 5.1; en-US; rv:1.7.5) Gecko/20041107 Firefox/1.0");
	curl_easy_setopt(curl, CURLOPT_URL, url);
	curl_easy_setopt(curl, CURLOPT_HEADER, 0);
	curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
        if( do_file ) {
            /* send all data to this function  */ 
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, 
                             fwrite);
            
            /* we pass our 'chunk' struct to the callback function */ 
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, 
                             (void *)fptr);
        }

	// Attempt to retrieve the remote page
	result = curl_easy_perform(curl);


        if( do_file ) {
            fflush( fptr );
            fclose( fptr );
        }
	curl_easy_cleanup(curl); 
	return result;
}

string CameraSubsystem::MonitorMessage()
{
	return "";
}
