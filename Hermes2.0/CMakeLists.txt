cmake_minimum_required( VERSION 2.8 )

project( Nodes )

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Boost COMPONENTS thread REQUIRED )

include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Messages.proto )


add_executable( node1 node1.cpp Messages.pb.cc )
add_executable( node2 node2.cpp Messages.pb.cc )

target_link_libraries( node1 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES} )
target_link_libraries( node2 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES} )