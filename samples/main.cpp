#include <stdio.h>
#include <iostream>
#include "nvilidar_process.h"
#include "mysignal.h"


using namespace std;
using namespace vp100_lidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif


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


	vp100_lidar::LidarProcess lidar("/dev/ttyUSB0",115200);

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
					//float angle = scan.points.at(i).angle;
					//float dis = scan.points.at(i).range;
					//printf("a:%f,d:%f\n", angle, dis);
				}
				vp100_lidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
					scan.stamp, (unsigned int)scan.points.size(),
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

		delayMS(5);		//add sleep,otherwise high cpu 
	}
	lidar.LidarTurnOff();       //stop scan 
	vp100_lidar::console.message("Lidar is Stopping......");
	lidar.LidarCloseHandle();   //close connect 

	delayMS(100);

	return 0;
}
