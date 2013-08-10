cmake_minimum_required( VERSION 2.8 )

project( NodeRelay )

# set C++11 flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")

# add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )


find_package( ZeroMQ REQUIRED )
find_package( HAL 0.1 REQUIRED )
find_package( PbMsgs 2.0 REQUIRED )

include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${PbMsgs_INCLUDE_DIRS} )

find_package( Protobuf REQUIRED )
include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Command.proto )


set( HDRS ${PROTO_HDRS} FtdiDriver.h Node.h )
set( SRCS main.cpp ${PROTO_SRCS} )


add_executable( NodeRelay ${HDRS} ${SRCS} )
target_link_libraries( NodeRelay ${ZeroMQ_LIBRARIES} ${HAL_LIBRARIES} ${PbMsgs_LIBRARIES} ${PROTOBUF_LIBRARIES} )
