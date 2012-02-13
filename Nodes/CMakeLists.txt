cmake_minimum_required( VERSION 2.8 )

project( Nodes )

include( Common.cmake )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )

include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

add_executable( node1 node1.cpp Testmsgs.pb.cc )
add_executable( node2 node2.cpp Testmsgs.pb.cc )

add_executable( n1 n1.cpp Testmsgs.pb.cc )
add_executable( n2 n2.cpp Testmsgs.pb.cc )

target_link_libraries( node1 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} )

target_link_libraries( node2 
        ${ZeroMQ_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
	)

target_link_libraries( n1 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} )
target_link_libraries( n2 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} )
