# NVILIDAR SDK DRIVER

## How to build NVILIDAR SDK samples

### 1.Get the SDK code
    1) Clone this project to your catkin's workspace src folder
    	$ git clone https://gitee.com/nvilidar/vp100_sdk.git       
		or
		$ git clone https://github.com/nvilidar/vp100_sdk_sdk.git

    2) download the sdk code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin


### 2.build the SDK
	1) linux
		$ cd sdk
		$ cd ..
		$ mkdir build
		$ cd build
		$ cmake ../vp100_sdk
		$ make			
	2) windows
		$ cd sdk
		$ cd ..
		$ mkdir build
		$ cd build
		$ cmake ../vp100_sdk
		$ make	
		then you can open "Project.sln" to open the visual studio.
	you can also Open the "CMakeLists.txt" directly with VS2017 or later

### 3.Serialport configuration
    1) if you use the lidar device name,you must give the permissions to user.
        ---whoami
       get the user name.link ubuntu.
        ---sudo usermod -a -G dialout ubuntu
       ubuntu is the user name.
        ---sudo reboot   

## ROS Parameter Configuration
### 1. Lidar Support
    VP100 is a serial interface lidar,
    current ros support VP100 Lidar,the baudrate can be 115200bsp or 230400bps 


### 2. Choice the SDKCommunication interface

	(1). if you want use the serialport,you neet to change the code from "nvilidar_node.cpp",change the code like this:

## Interface function definition
### 1. bool LidarProcess::LidarInitialialize()
    Initialize the lidar, including opening the serial/socket interface and synchronizing the radar with the SDK parameter information
	if initial fail return false.
### 2. bool LidarProcess::LidarTurnOn()
	Turn on lidar scanning so that it can output point cloud data.
### 3. bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	Real-time radar data output interface.
	The LidarScan variables are described as follows:
 
|  value   | Element | define  |
|  :----:  | :----:  | :----:  |
|  stamp   | none    |  lidar stamps, unit ns|
|  config  | min_angle   | lidar angle min value, 0~2*PI,Unit Radian|
|          | max_angle   | lidar angle max value, 0~2*PI,Unit Radian|
|          | angle_increment   | angular interval between 2 points, 0~2PI|
|          | scan_time   | time interval of 2 turns of data|
|          | min_range   | Distance measurement min, unit m|
|          | max_range   | Distance measurement max, unit m|
|  points  | angle       | lidar angle,0~2PI|
|  | range       | lidar distance, unit m|
### 4. bool LidarProcess::LidarTurnOff()
	lidar turn off the scanning data 
### 5. void LidarProcess::LidarCloseHandle()
	lidar close serialport/socket 

## How to run NVILIDAR SDK samples
    $ cd samples

linux:

	$ ./nvilidar_test

windows:

	$ nvilidar_test.exe

You should see NVILIDAR's scan result in the console:

   _   ___      _______ _      _____ _____          _____ 
	| \ | \ \    / /_   _| |    |_   _|  __ \   /\   |  __ \
	|  \| |\ \  / /  | | | |      | | | |  | | /  \  | |__) |
	| . ` | \ \/ /   | | | |      | | | |  | |/ /\ \ |  _  / 
	| |\  |  \  /   _| |_| |____ _| |_| |__| / ____ \| | \ \
	|_| \_|   \/   |_____|______|_____|_____/_/    \_\_|  \ \

	Current sdk supports vp100 lidar 
	the srialport baudrate can be 115200bps or 230400bps
	[NVILidar]: [NVILIDAR INFO] Now NVILIDAR is scanning ......
	[NVILidar]: Scan received[1683876754134474857]: 499 ranges is [5.789838]Hz
	[NVILidar]: Scan received[1683876754307191244]: 499 ranges is [6.162893]Hz
	[NVILidar]: Scan received[1683876754469452693]: 499 ranges is [6.160737]Hz
	[NVILidar]: Scan received[1683876754631770939]: 499 ranges is [5.798543]Hz
	[NVILidar]: Scan received[1683876754804228050]: 500 ranges is [6.171116]Hz
	[NVILidar]: Scan received[1683876754966273297]: 499 ranges is [5.813266]Hz
	[NVILidar]: Scan received[1683876755138293634]: 501 ranges is [6.195767]Hz
	[NVILidar]: Scan received[1683876755299694141]: 501 ranges is [6.181950]Hz
	[NVILidar]: Scan received[1683876755461455401]: 503 ranges is [5.811990]Hz
	[NVILidar]: Scan received[1683876755633513508]: 501 ranges is [5.816296]Hz
	[NVILidar]: Scan received[1683876755805444218]: 502 ranges is [6.182780]Hz
	[NVILidar]: Scan received[1683876755967183747]: 501 ranges is [6.165780]Hz
	[NVILidar]: Scan received[1683876756129369224]: 502 ranges is [5.807845]Hz
	[NVILidar]: Scan received[1683876756301550129]: 503 ranges is [5.804401]Hz



## NVILIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| auto_reconnect  | lidar auto connect,if it is disconnet in case |
| reversion  | lidar's point revert|
| inverted  | lidar's point invert|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:64.0 meters|
| range_min  | lidar's min measure distance,default:0.0 meters|
| aim_speed  | lidar's run speed,default:10.0 Hz|
| sampling_rate  | lidar's sampling rate,default:10.0 K points in 1 second|
| angle_offset_change_flag  | angle offset enable to set,default:false|
| angle_offset  | angle offset,default:0.0|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°-60° and 90°-120° points in the view|