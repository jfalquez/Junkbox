cmake_minimum_required( VERSION 2.8 )

project( Nodes )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Boost COMPONENTS thread date_time system timer REQUIRED )

include_directories( ${BOOST_INCLUDE_DIR} )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Messages.proto )


add_executable( node1 node1.cpp Messages.pb.cc )
add_executable( node2 node2.cpp Messages.pb.cc )

target_link_libraries( node1 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_TIMER_LIBRARY} )
target_link_libraries( node2 ${ZeroMQ_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_TIMER_LIBRARY} )
