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
#include <fstream>
using namespace std;

class Logger
{
public:
	Logger(const char *filename);
	~Logger();
	void out(const char *format, ...);
private:
	ofstream m_stream;
};
