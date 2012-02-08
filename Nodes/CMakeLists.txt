cmake_minimum_required( VERSION 2.8 )

project( Nodes )

include( Common.cmake )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( RPG REQUIRED )
find_package( CVARS REQUIRED )

include_directories( ${CVARS_INCLUDE_DIR} )
include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

add_executable( Nodes main.cpp )

target_link_libraries( Nodes 
	${RPG_LIBRARIES}
	${CVARS_LIBRARIES}
        ${ZeroMQ_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
	)
