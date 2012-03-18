cmake_minimum_required( VERSION 2.8 )

project( Kinect )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )
find_package( MVL REQUIRED )
find_package( PCL REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${PCL_INCLUDE_DIRS} )

add_executable( kinect main.cpp )
target_link_libraries( kinect
                        ${OpenCV_LIBS}
                        ${RPG_LIBRARIES}
                        ${MVL_LIBRARIES}
                        ${PCL_LIBRARIES}
                        )