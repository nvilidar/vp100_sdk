/*
 * @Version      : V1.0
 * @Date         : 2024-10-15 17:24:46
 * @Description  : lidar process
 */

#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "lidar/lidar_protocol.hpp"

namespace nvistar{

#ifndef DLL_EXPORT
  #ifdef _MSC_VER
    #define DLL_EXPORT __declspec(dllexport)
  #else
    #define DLL_EXPORT
  #endif 
#endif 

class LidarImpl;     //forward declaration

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

class DLL_EXPORT Lidar{
  public:
    Lidar();
    ~Lidar();
    void lidar_register(lidar_interface_t* interface);
    void lidar_unregister();
    lidar_scan_status_t lidar_get_scandata(lidar_scan_period_t &scan, uint32_t timeout = 2000);
  private:
    LidarImpl *_impl;
    LidarProtocol *_protocol = nullptr;
};

}

#endif