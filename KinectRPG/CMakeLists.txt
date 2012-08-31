cmake_minimum_required( VERSION 2.8 )

project( Kinect )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )

include_directories( ${OpenCV2_INCLUDE_DIRS} )
include_directories( ${RPG_INCLUDE_DIRS} )

add_executable( kinect main.cpp )

target_link_libraries( kinect
                        ${OpenCV2_LIBRARIES}
                        ${RPG_LIBRARIES}
                        )
