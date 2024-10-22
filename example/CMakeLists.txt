cmake_minimum_required(VERSION 3.5)
project(lidar_sdk_example)

# set base info
set(CMAKE_CXX_STANDARD 11)  
set(CMAKE_CXX_STANDARD_REQUIRED ON)  
if(SUPPORTS_FDIAGNOSTICS_COLOR)  
    add_compile_options(-fdiagnostics-color=always)  
endif()


set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
add_executable(lidar_sdk_example example.cpp)

# # add the required libraries for linking:
target_link_libraries(lidar_sdk_example lidar_sdk_driver)