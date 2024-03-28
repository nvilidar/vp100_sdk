#include "nvilidar_log.h"

namespace vp100_lidar{
    static std::shared_ptr<spdlog::logger> logger;
    static bool  logger_enable_flag = false;

    //log init 
    void LidarLog::LogInit(bool enable){
        //enable flag
        logger_enable_flag = enable;
        if(false == enable){
            return;
        }
        //max 5MB size,max 2 log file 
        auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("log/vp100_log.txt", 1024 * 1024 * 5, 2); 
        //log get 
        logger = std::make_shared<spdlog::logger>("console", rotating_sink);
        spdlog::register_logger(logger);

        //set log level 
        logger->set_level(spdlog::level::debug);
        //log write to log fast 
        logger->flush_on(spdlog::level::debug);
        //set the log pattern 
        logger->set_pattern("%v", spdlog::pattern_time_type::local);
        logger->set_pattern("[%Y-%m-%d %H:%M:%S.%f] %v",
                            spdlog::pattern_time_type::local);
    }

    //log test 
    void LidarLog::LogTest(){
        if(false == logger_enable_flag){
            return;
        }
        logger->trace("This is a trace message");
        logger->debug("This is a debug message");
        logger->info("This is an info message");
        logger->warn("This is a warning message");
        logger->error("This is an error message");
        logger->critical("This is a critical message");
    }

    //log write info 
    void LidarLog::LogWriteInfo(std::string info){
        if(false == logger_enable_flag){
            return;
        }
        std::string string_write = "[info] ";
        string_write += info;
        logger->info(string_write.c_str());
    }

    //log write error 
    void LidarLog::LogWriteError(std::string info){
        if(false == logger_enable_flag){
            return;
        }
        std::string string_write = "[error] ";
        string_write += info;
        logger->info(string_write.c_str());
    }

    //log write warnning 
    void LidarLog::LogWriteWarn(std::string info){
        if(false == logger_enable_flag){
            return;
        }
        std::string string_write = "[warn] ";
        string_write += info;
        logger->info(string_write.c_str());
    }
}