#include <stdio.h>
#include <iostream>
#include "nvilidar_process.h"
#include "myconsole.hpp"
#include "mytimer.hpp"
//for signal check 
#if defined(_WIN32)
	#include <mmsystem.h>
	#include <sysinfoapi.h>
	#include <WinSock2.h>
	#include <windows.h>
#else 
	#include <iostream>
	#include <signal.h>
#endif 

using namespace vp100_lidar;

#if defined(_MSC_VER)
#pragma comment(lib, "vp100_lidar_driver.lib")
#endif

//get stamp 
uint64_t  get_stamp_callback(){
	return vp100_lidar::TimeStamp::getStamp();
}

//signal to stop value 
bool nvilidar_checking_signal = false;
//singal to stop 
#if defined(_WIN32)
	BOOL WINAPI consoleHandler(DWORD signal){
		if (signal == CTRL_C_EVENT){
			nvilidar_checking_signal = true;
		}
		return TRUE;
	}
#else 
	void consoleHandler(int signal){
		if (signal == SIGINT){
			nvilidar_checking_signal = true;
		}
	}
#endif 

//signal init 
void sigInit(void){
	#if defined(_WIN32)
		SetConsoleCtrlHandler(consoleHandler, TRUE);
	#else 
		signal(SIGINT,consoleHandler);
	#endif 
}

int main()
{
	printf(" _   ___      _______ _      _____ _____          _____ \n");
	printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
	printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
	printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
	printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
	printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
	printf("\n");
	fflush(stdout);

	//init signal,for ctrl+c
	sigInit();

	//introduce 
	printf("Current sdk supports vp100 lidar \n");
	printf("the srialport baudrate can be 115200bps or 230400bps\n");

	//para 0-serialname  1-baud  2-timestamp callback 4- if ns then 1e9 ,if us the 1e6 ...
	#if defined(_WIN32)
		vp100_lidar::LidarProcess lidar("COM5", 115200, get_stamp_callback, 1e9);
	#else 
		vp100_lidar::LidarProcess lidar("/dev/ttyUSB0",230400,get_stamp_callback,1e9);
	#endif 

	//init lidar,include sync lidar para 
	if (false == lidar.LidarInitialialize()){
		return 0;
	}

	//open lidar to output point 
	bool ret = lidar.LidarTurnOn();

	//point data analysis 
	while (ret && (false == nvilidar_checking_signal)){
		LidarScan scan;

		if (lidar.LidarSamplingProcess(scan)){
			if(scan.points.size() > 0){
				for (size_t i = 0; i < scan.points.size(); i++){
					//every point data print 
					#if 0
						float angle = scan.points.at(i).angle;
						float dis = scan.points.at(i).range;
						uint64_t stamp = scan.points.at(i).stamp;
						printf("a:%f,d:%f,stamp:%llu\n", angle, dis,stamp);
					#endif 
				}
				//print the range 
				#if defined(_WIN32)
					printf("Scan received[start:%llu | stop:%llu | differ:%llu]: %u ranges is [%f]Hz\n",
						scan.stamp_start, scan.stamp_stop,scan.stamp_differ,(unsigned int)scan.points.size(),
						1.0 / scan.config.scan_time);
				#else 
					printf("Scan received[start:%lu | stop:%lu | differ:%lu]: %u ranges is [%f]Hz\n",
						scan.stamp_start, scan.stamp_stop,scan.stamp_differ,(unsigned int)scan.points.size(),
						1.0 / scan.config.scan_time);
				#endif 
			}
			else {
				vp100_lidar::Console::warning("Lidar Data Invalid!");
			}
		}
		else{
			vp100_lidar::Console::error("Failed to get Lidar Data!");
			break;
		}

		vp100_lidar::TimeStamp::sleepMS(1);
	}
	lidar.LidarTurnOff();       //stop scan 
	vp100_lidar::Console::message("Lidar is Stopping......");
	lidar.LidarCloseHandle();   //close connect 

	vp100_lidar::TimeStamp::sleepMS(100);
	
	return 0;
}
