#pragma once

#include "spdlog/spdlog.h"
#include "spdlog/sinks/rotating_file_sink.h"

namespace vp100_lidar{
    class LidarLog{
        public:
            //log init 
            static void LogInit(bool enable);
            //log test 
            static void LogTest();
            //log info 
            static void LogWriteInfo(std::string info);
            //log error 
            static void LogWriteError(std::string info);
            //log warnning
            static void LogWriteWarn(std::string info);
        private:
    };
}