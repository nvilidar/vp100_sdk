/*
 * @Version      : V1.0
 * @Date         : 2024-10-15 17:25:02
 * @Description  : 
 */
#include "lidar.hpp"
#include "lidar/lidar_protocol.hpp"
#include <atomic>
#include <chrono>
#include <cstddef>

namespace nvistar{
//pre define
class LidarImpl{
public:
  std::atomic<bool> lidar_pointcloud_ready_flag = {false};
  lidar_scan_period_t  scan_period;   //one period points
  bool  lidar_scandata_update_flag = false;  //lidar data update flag 
  std::chrono::steady_clock::time_point   last_point_update_time = std::chrono::steady_clock::now();    //last point    
};

Lidar::Lidar() : _impl(new LidarImpl){
  _protocol = new LidarProtocol();
}

Lidar::~Lidar(){
  _protocol->lidar_protocol_unregister();
  delete _impl;
  delete _protocol;
}

/**
 * @Function: lidar_register
 * @Description: lidar register comm interface 
 * @Return: void
 * @param {lidar_interface_t*} interface
 */
void Lidar::lidar_register(lidar_interface_t* interface){
  //callback function 
  auto pointcloud_callback = [this](lidar_scan_period_t rawdata_output){
    //read finish, then enable to read next package 
    if(_impl->lidar_pointcloud_ready_flag.load() == false){
      //calc every point stamp value
      if(rawdata_output.points.size() > 1){
        uint64_t  timestamp_differ = rawdata_output.timestamp_stop - rawdata_output.timestamp_start;
        uint64_t  timestamp_gap = (timestamp_differ / rawdata_output.points.size() - 1);
        for(size_t i = 0; i<rawdata_output.points.size(); i++){
          rawdata_output.points[i].timestamp = rawdata_output.timestamp_start + i*timestamp_gap;
        }
      }

      //update points 
      _impl->scan_period = rawdata_output;

      //change flag 
      if(rawdata_output.timestamp_start > 0){
        _impl->lidar_pointcloud_ready_flag.store(true);   //finished
      }
    }
  };
  _protocol->lidar_protocol_register(interface, pointcloud_callback);
}

/**
 * @Function: lidar_unregister
 * @Description: lidar unregister 
 * @Return: void
 */
void Lidar::lidar_unregister(){
  _protocol->lidar_protocol_unregister();
}

/**
 * @Function: lidar_get_scandata
 * @Description: lidar get scan data 
 * @Return: lidar_scan_status_t --- status 
 * @param {lidar_scan_period_t} &scan
 * @param {uint32_t} timeout
 */
lidar_scan_status_t Lidar::lidar_get_scandata(lidar_scan_period_t &scan, uint32_t timeout){
  //update info 
  if(_impl->lidar_pointcloud_ready_flag.load()){
    //update the last upate time 
    _impl->last_point_update_time = std::chrono::steady_clock::now();
    //update scan data 
    scan = _impl->scan_period;
    //clear the flag 
    _impl->lidar_pointcloud_ready_flag.store(false);
    //error code judge
    if(scan.error_code == LidarProtocol::ERROR_CODE_MOTOR_LOCK){
      return LIDAR_SCAN_ERROR_MOTOR_LOCK;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_UP_NO_POINT){
      return LIDAR_SCAN_ERROR_UP_NO_POINT;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_MOTOR_SHORTCIRCUIT) {
      return LIDAR_SCAN_ERROR_MOTOR_SHORTCIRCUIT;
    }else if(scan.error_code == LidarProtocol::ERROR_CODE_RESET){
      return LIDAR_SCAN_ERROR_RESET;
    }
    return LIDAR_SCAN_OK;
  }else{
    //check timeout
    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _impl->last_point_update_time).count() > timeout){
      _impl->last_point_update_time = std::chrono::steady_clock::now();   //update last time 
      return LIDAR_SCAN_TIMEOUT;
    }
  }
  //waiting...
  return LIDAR_SCAN_WAITING;
}
}