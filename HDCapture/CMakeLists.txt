cmake_minimum_required( VERSION 2.8 )

project( HDCam )

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH} )

find_package( ZeroMQ REQUIRED )
find_package( Protobuf REQUIRED )
find_package( Boost COMPONENTS thread REQUIRED )

include_directories( Blackmagic )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )
include_directories( ${PROTOBUF_INCLUDE_DIRS} )

include_directories( ${CMAKE_BINARY_DIR} )
#PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Messages.proto )

set( HDCAPTURE_SOURCES HDMICapture.cpp Blackmagic/DeckLinkAPIDispatch.cpp )
set( HDCAPTURE_HEADERS HDMICapture.h Node.h )

add_executable( hdcap HDCapture.cpp Blackmagic/DeckLinkAPIDispatch.cpp )
