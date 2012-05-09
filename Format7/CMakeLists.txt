cmake_minimum_required( VERSION 2.8 )

project( Format7 )

find_package( OpenCV REQUIRED )
find_package( DC1394_2 REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${DC1394_2_INCLUDE_DIR} )

add_executable( format7 main.cpp )
target_link_libraries( format7
                        ${OpenCV_LIBS}
                        ${DC1394_2_LIBRARIES}
                        )
