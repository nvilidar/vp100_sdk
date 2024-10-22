cmake_minimum_required(VERSION 3.5)
project(LidarSDK)

# set base info
set(CMAKE_CXX_STANDARD 11)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)  
if(SUPPORTS_FDIAGNOSTICS_COLOR)  
    add_compile_options(-fdiagnostics-color=always)  
endif()

# add include
include_directories(include)

# add src
FILE(GLOB LIDAR_SDK_SRC 
  "src/lidar/lidar_protocol.cpp"
  "src/lidar.cpp"
  "src/interface/console/interface_console.cpp"
)

if(WIN32)  
  list(APPEND LIDAR_SDK_SRC "src/interface/serial/win/interface_serial.cpp")  
elseif(UNIX)  
  list(APPEND LIDAR_SDK_SRC "src/interface/serial/unix/interface_serial.cpp")  
endif()

# example 
add_subdirectory(example)  

# build
add_library(lidar_sdk_driver SHARED ${LIDAR_SDK_SRC})

if(WIN32) 
  target_link_libraries(lidar_sdk_driver setupapi ws2_32)
elseif(UNIX)  
  target_link_libraries(lidar_sdk_driver pthread)
endif() 

# install
set(INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install) 
install(TARGETS lidar_sdk_driver lidar_sdk_example
   RUNTIME DESTINATION ${INSTALL_PREFIX}/bin
   LIBRARY DESTINATION ${INSTALL_PREFIX}/lib
   ARCHIVE DESTINATION ${INSTALL_PREFIX}/lib
)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
   DESTINATION ${INSTALL_PREFIX}/include
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
