cmake_minimum_required( VERSION 2.8 )

project( GwSim )

include( Common.cmake )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )

include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( gwsim main.cpp )
target_link_libraries( gwsim
       	${OpenCV_LIBS} 
	${RPG_LIBRARIES}
	)
