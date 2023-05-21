#include <stdio.h>
#include <iostream>
#include "nvilidar_process.h"
#include "mysignal.h"

using namespace vp100_lidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif

//get stamp 
uint64_t  get_stamp_callback(){
	return getStamp();
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
	vp100_lidar::sigInit();

	//introduce 
	printf("Current sdk supports vp100 lidar \n");
	printf("the srialport baudrate can be 115200bps or 230400bps\n");

	//para 0-serialname  1-baud  2-timestamp callback 4- if ns then 1e9 ,if us the 1e6 ...
	#if 1
		vp100_lidar::LidarProcess lidar("/dev/ttyUSB0",115200,get_stamp_callback,1e9);
	#else 
		vp100_lidar::LidarProcess lidar("COM5", 115200, get_stamp_callback, 1e9);
	#endif 

	//init lidar,include sync lidar para 
	if (false == lidar.LidarInitialialize())		
	{
		return 0;
	}

	//open lidar to output point 
	bool ret = lidar.LidarTurnOn();

	//point data analysis 
	while (ret && (vp100_lidar::isOK()))
	{
		LidarScan scan;

		if (lidar.LidarSamplingProcess(scan))
		{
			if(scan.points.size() > 0)
			{
				for (size_t i = 0; i < scan.points.size(); i++)
				{
					// float angle = scan.points.at(i).angle;
					// float dis = scan.points.at(i).range;
					// uint64_t stamp = scan.points.at(i).stamp;
					// printf("a:%f,d:%f,stamp:%lu\n", angle, dis,stamp);
				}
				printf("Scan received[start:%lu | stop:%lu | differ:%lu]: %u ranges is [%f]Hz\n",
					scan.stamp_start, scan.stamp_stop,scan.stamp_differ,(unsigned int)scan.points.size(),
					1.0 / scan.config.scan_time);
			}
			else 
			{
				vp100_lidar::console.warning("Lidar Data Invalid!");
			}
		}
		else
		{
			vp100_lidar::console.error("Failed to get Lidar Data!");
			break;
		}

		delayMS(1);		//add sleep,otherwise high cpu 
	}
	lidar.LidarTurnOff();       //stop scan 
	vp100_lidar::console.message("Lidar is Stopping......");
	lidar.LidarCloseHandle();   //close connect 

	delayMS(100);

	return 0;
}
