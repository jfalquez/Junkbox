cmake_minimum_required( VERSION 2.8 )

project( GwSim )

include( Common.cmake )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )
find_package( Protobuf REQUIRED )

include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( gwsim main.cpp )
target_link_libraries( gwsim
       	${OpenCV_LIBS}
        ${RPG_LIBRARIES}
	)

add_executable( fileWriter fileWriter.cpp )
target_link_libraries( fileWriter
       	${OpenCV_LIBS}
        ${RPG_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
	)
