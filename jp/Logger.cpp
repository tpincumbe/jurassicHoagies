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
#include "Logger.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

Logger::Logger(const char *filename)
{
	m_stream.open(filename);
}

Logger::~Logger()
{
	m_stream.close();
}

void Logger::out(const char *format, ...)
{
    va_list arg;
	char cbuffer[1024];
    va_start(arg,format);
	vsnprintf(cbuffer,1024,format,arg);
    va_end(arg);
	m_stream << cbuffer << endl;
	m_stream.flush();
}
