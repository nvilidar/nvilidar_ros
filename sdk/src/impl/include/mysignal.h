#pragma once 

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <inttypes.h>
#if defined(_WIN32)
	#include <mmsystem.h>
	#include <sysinfoapi.h>
	#include <WinSock2.h>
	#include <windows.h>
#else 
	#include <iostream>
	#include <signal.h>
#endif 

namespace nvilidar
{
	bool runnig_state = true;

#if defined(_WIN32)

	BOOL WINAPI  consoleHandler(DWORD signal)
	{
		if (signal == CTRL_C_EVENT)
		{
			//printf("key press!");
			runnig_state = false;
		}
		return true;
	}

	inline void sigInit(void)
	{
		//printf("\nERROR: Could not set control handler");
		if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
			//printf("\nERROR: Could not set control handler");
			return;
		}
		//printf("signal init OK");
	}

	inline bool isOK()
	{
		return runnig_state;
	}
#else 
	void consoleHandler( int signal)
	{
		if (signal == SIGINT)
		{
			//printf("key press!");
			runnig_state = false;
		}
	}

	inline void sigInit(void)
	{
		//printf("\nERROR: Could not set control handler");
		signal(SIGINT,consoleHandler);
		//printf("signal init OK");
	}

	inline bool isOK()
	{
		return runnig_state;
	}
#endif 

}
