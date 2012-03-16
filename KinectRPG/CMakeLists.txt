cmake_minimum_required( VERSION 2.8 )

project( Kinect )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )
find_package( MVL REQUIRED )

include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( kinect main.cpp )
target_link_libraries( kinect
       	${OpenCV_LIBS} 
	${MVL_LIBRARIES}
	${RPG_LIBRARIES}
	)
