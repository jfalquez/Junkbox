cmake_minimum_required( VERSION 2.8 )

project( TestCam )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package( OpenCV REQUIRED )
find_package( HAL REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )

message( STATUS "HAL Includes: " ${HAL_INCLUDE_DIRS} )

add_executable( testcam main.cpp )
target_link_libraries( testcam
                        ${OpenCV_LIBS}
                        ${HAL_LIBRARIES}
                        )
