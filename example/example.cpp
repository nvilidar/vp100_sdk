/*
 * @Version      : V1.0
 * @Date         : 2024-10-16 19:17:00
 * @Description  : example 
 */

#include "interface/serial/interface_serial.hpp"
#include "interface/console/interface_console.hpp"
#include "lidar.hpp"
#include <chrono>
#include <cstdio>
#include <inttypes.h>
#include <thread>
#if defined(_WIN32)
  #include <windows.h>
#else 
	#include <signal.h>
  #include <unistd.h>
#endif 

//define 
nvistar::InterfaceSerial *_serial;
nvistar::Lidar  *_lidar;
nvistar::InterfaceConsole *_console;

//comm interface 
int serial_write(const uint8_t* data,int length){
  return _serial->serial_write(data, length);
}
int serial_read(uint8_t* data,int length){
  return _serial->serial_read(data, length);
}
void serial_flush(){
  _serial->serial_flush();
}
//timestamp 
uint64_t get_stamp(){
  auto now = std::chrono::system_clock::now();  
  auto duration = now.time_since_epoch();  
  uint64_t nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();  
  return nanoseconds;
}
//signal 
bool signal_flag = false;
#if defined(_WIN32)
	BOOL WINAPI consoleHandler(DWORD signal){
		if (signal == CTRL_C_EVENT){
			signal_flag = true;
		}
		return TRUE;
	}
#else 
	void consoleHandler(int signal){
		if (signal == SIGINT){
			signal_flag = true;
		}
	}
#endif 
void sig_init(void){
	#if defined(_WIN32)
		SetConsoleCtrlHandler(consoleHandler, TRUE);
	#else 
		signal(SIGINT,consoleHandler);
	#endif 
}


int main(){
  printf(" _   ___      _______  _____ _______       _____ \n");
	printf("| \\ | \\ \\    / /_   _|/ ____|__   __|/\\   |  __ \\\n");
	printf("|  \\| |\\ \\  / /  | | | (___    | |  /  \\  | |__) |\n");
	printf("| . ` | \\ \\/ /   | |  \\___ \\   | | / /\\ \\ |  _  /\n");
	printf("| |\\  |  \\  /   _| |_ ____) |  | |/ ____ \\| | \\ \\\n");
	printf("|_| \\_|   \\/   |_____|_____/   |_/_/    \\_\\_|  \\_\\\n");
	printf("\n");
	fflush(stdout);                                                

  bool ret = false;
  nvistar::lidar_scan_period_t scan;
  int  timeout_count = 0;
 //nvistar::lidar_scan_ros_format_t scan_ros;

  _serial = new nvistar::InterfaceSerial();
  _lidar = new nvistar::Lidar();
  _console = new nvistar::InterfaceConsole();

  //callback function
  nvistar::lidar_interface_t  _interface = {
    {
      serial_write,
      serial_read,
      serial_flush,
    },
    get_stamp
  };
  //show sdk version 
  _console->print_normal("the lidar sdk version is : %s\n",_lidar->get_sdk_version().c_str());
  //serial open 
  ret = _serial->serial_open("/dev/ttyUSB0", 230400);
  //lidar register 
  if(ret){
    _lidar->lidar_register(&_interface);
    _console->print_noerr("lidar is scanning...\n");
  }else{
    _console->print_error("lidar serial open failed!\n");
    _serial->serial_close();
  }
  //loop to get point
  while (ret && (!signal_flag)) {
    nvistar::lidar_scan_status_t status = _lidar->lidar_get_scandata(scan);
    //_lidar->lidar_raw_to_ros_format(scan, scan_ros);
    switch(status){
      case nvistar::LIDAR_SCAN_OK:{
        timeout_count = 0;
         _console->print_noerr("speed(RPM):%f, size:%zu, timestamp_start:%" PRIu64 ", timestamp_stop:%" PRIu64 ", timestamp_differ:%" PRIu64
              , scan.speed, scan.points.size(), scan.timestamp_start, scan.timestamp_stop, scan.timestamp_stop - scan.timestamp_start);
        //output the points 
        #if 0
          for(size_t i = 0; i<scan.points.size(); i++){
            _console->print_normal("angle:%.6f, distance:%.3f, intensity:%.2f, stamp:%" PRIu64 ""
                      ,scan.points[i].angle, scan.points[i].distance, scan.points[i].intensity, scan.points[i].timestamp);
          } 
        #endif 
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_LOCK: {
        timeout_count = 0;
        _console->print_warn("lidar motor lock!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT: {
        timeout_count = 0;
        _console->print_warn("lidar motor short circuit!");
        break;
      }
      case nvistar::LIDAR_SCAN_ERROR_UP_NO_POINT: {
        timeout_count = 0;
        _console->print_warn("lidar upboard no points!");
        break;
      }
      case nvistar::LIDAR_SCAN_TIMEOUT: {
        _console->print_warn("lidar data timeout!");
        //reconnect 
        timeout_count++;
        if(timeout_count >= 10){
          timeout_count = 0;
          _serial->serial_reopen();
          _console->print_noerr("lidar serial reopen!");
        }
        break;
      }
      default:{
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  _console->print_normal("lidar is stoping...\n");
  //delete
  delete _lidar;
  _lidar = nullptr;
  delete _serial;
  _serial = nullptr;
  delete _console;
  _console = nullptr;
}
