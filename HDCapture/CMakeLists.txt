cmake_minimum_required( VERSION 2.8 )

project( HDCap )

find_package( ZeroMQ REQUIRED )
find_package( Boost COMPONENTS thread REQUIRED )

include_directories( Blackmagic )
include_directories( ${ZeroMQ_INCLUDE_DIRS} )

set( HDCAPTURE_HEADERS CaptureDelegate.h )
set( HDCAPTURE_SOURCES HDCapture.cpp CaptureDelegate.cpp Blackmagic/DeckLinkAPIDispatch.cpp )

add_executable( hdcap ${HDCAPTURE_HEADERS} ${HDCAPTURE_SOURCES} )

target_link_libraries( hdcap
                        dl
                        pthread
                        ${ZeroMQ_LIBRARIES}
                        )