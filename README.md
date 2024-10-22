# VP100/T10 SDK DRIVER

## How to build SDK samples

### 1. Lidar Support
VP100 is a serial interface lidar,
current ros support VP100 Lidar,the baudrate can be 115200bsp or 230400bps 

### 2.Get the SDK code
    1) Clone this project to your catkin's workspace src folder
    	$ git clone https://gitee.com/nvilidar/vp100_sdk.git       
		or
		$ git clone https://github.com/nvilidar/vp100_sdk.git

    2) download the sdk code from our webset,  http://www.nvistar.com/?jishuzhichi/xiazaizhongxin

### 3.build the SDK
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

    and you can also use vscode to build and run.

### 4.Serialport configuration

#### linux

if you use the lidar device name,you must give the permissions to user.
```shell
whoami
```
get the user name.link ubuntu.
```shell
sudo usermod -a -G dialout ubuntu
```
ubuntu is the user name.
```shell
sudo reboot   
```

#### windows 
if you want to use the serialport to get the lidar,you neet to use usb to serialport tool.we suggest [CP2102](https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads)

### 5.How to run VP100 Lidar SDK samples
    $ cd example

linux:

	$ ./lidar_sdk_example

windows:

	$ lidar_sdk_example

You should see scan result in the console:

```shell
 _   ___      _______  _____ _______       _____
| \ | \ \    / /_   _|/ ____|__   __|/\   |  __ \
|  \| |\ \  / /  | | | (___    | |  /  \  | |__) |
| . ` | \ \/ /   | |  \___ \   | | / /\ \ |  _  /
| |\  |  \  /   _| |_ ____) |  | |/ ____ \| | \ \
|_| \_|   \/   |_____|_____/   |_/_/    \_\_|  \_\

lidar is scanning...

speed(RPM):360.203125, size:500, timestamp_start:1729565824470524300, timestamp_stop:1729565824635999000, timestamp_differ:165474700
speed(RPM):360.781250, size:500, timestamp_start:1729565824635999000, timestamp_stop:1729565824803479800, timestamp_differ:167480800
speed(RPM):360.281250, size:500, timestamp_start:1729565824803479800, timestamp_stop:1729565824969486000, timestamp_differ:166006200
speed(RPM):360.453125, size:500, timestamp_start:1729565824969486000, timestamp_stop:1729565825134193700, timestamp_differ:164707700
speed(RPM):360.437500, size:501, timestamp_start:1729565825134193700, timestamp_stop:1729565825302385900, timestamp_differ:168192200
speed(RPM):360.078125, size:501, timestamp_start:1729565825302385900, timestamp_stop:1729565825470086300, timestamp_differ:167700400
speed(RPM):360.046875, size:501, timestamp_start:1729565825470086300, timestamp_stop:1729565825633854600, timestamp_differ:163768300
speed(RPM):359.828125, size:501, timestamp_start:1729565825633854600, timestamp_stop:1729565825801537800, timestamp_differ:167683200
speed(RPM):359.703125, size:500, timestamp_start:1729565825801537800, timestamp_stop:1729565825968564000, timestamp_differ:167026200
speed(RPM):360.312500, size:501, timestamp_start:1729565825968564000, timestamp_stop:1729565826136569500, timestamp_differ:168005500

```

```shell

 _   ___      _______  _____ _______       _____
| \ | \ \    / /_   _|/ ____|__   __|/\   |  __ \
|  \| |\ \  / /  | | | (___    | |  /  \  | |__) |
| . ` | \ \/ /   | |  \___ \   | | / /\ \ |  _  /
| |\  |  \  /   _| |_ ____) |  | |/ ____ \| | \ \
|_| \_|   \/   |_____|_____/   |_/_/    \_\_|  \_\

lidar is scanning...

speed(RPM):357.390625, size:503, timestamp_start:1729565992497166800, timestamp_stop:1729565992664471400, timestamp_differ:167304600
angle:1.27, distance:1932.00, intensity:328.00, stamp:1729565992497166800
angle:1.97, distance:1228.00, intensity:171.00, stamp:1729565992497499412
angle:2.68, distance:1228.00, intensity:160.00, stamp:1729565992497832024
angle:3.39, distance:1226.00, intensity:117.00, stamp:1729565992498164636
angle:4.10, distance:1927.00, intensity:432.00, stamp:1729565992498497248
angle:4.82, distance:1935.00, intensity:431.00, stamp:1729565992498829860
angle:5.53, distance:1917.00, intensity:422.00, stamp:1729565992499162472

```


## SDK interface

### 1.lidar_register
```cpp
void Lidar::lidar_register(lidar_interface_t* interface)
```
the function is used to register the timestamp and communicate interface to analysis

the 'lidar_interface_t' struct:
```cpp
//callback function,it to be serailport,socket,etc...
typedef struct{
  std::function<int(const uint8_t* data,int length)> write;
  std::function<int(uint8_t *data,int max_length)>  read;
  std::function<void(void)> flush;
}lidar_transmit_interface_t;
//callback function
typedef struct{
  lidar_transmit_interface_t  transmit;
  std::function<uint64_t(void)> get_timestamp;
}lidar_interface_t;
```
### 2.lidar_get_scandata
```cpp
lidar_scan_status_t Lidar::lidar_get_scandata(lidar_scan_period_t &scan, uint32_t timeout)
```
the function is used to get the lidar points data 

```cpp
//single point info 
typedef struct{
    double    angle;      //degree
    double    distance;   //mm
    double    intensity;  
    uint64_t  timestamp;  //from callback
}lidar_scan_point_t;
//point info for 1 period 
typedef struct{
  int       model_code;                   //lidar model code 
  std::vector<lidar_scan_point_t> points; //one period points 
  bool      intensity_flag;               //intensity?
  double    speed;                        //RPM
  int       error_code;                   //error code 
  uint64_t  timestamp_start;              //stamp start 
  uint64_t  timestamp_stop;               //stamp stop 
}lidar_scan_period_t;
```

the function is used to get the status 

```cpp
//lidar return status 
typedef enum{
  LIDAR_SCAN_OK = 0,
  LIDAR_SCAN_WAITING,
  LIDAR_SCAN_TIMEOUT,
  LIDAR_SCAN_ERROR_MOTOR_LOCK,
  LIDAR_SCAN_ERROR_UP_NO_POINT,
  LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT,
  LIDAR_SCAN_ERROR_RESET,
}lidar_scan_status_t;
```