cmake_minimum_required( VERSION 2.8 )

project( TestCam )

find_package( OpenCV REQUIRED )
find_package( RPG REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${RPG_INCLUDE_DIRS} )

add_executable( testcam main.cpp )
target_link_libraries( testcam
                        ${OpenCV_LIBS}
                        ${RPG_LIBRARIES}
                        )