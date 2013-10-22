cmake_minimum_required( VERSION 2.8 )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")

project( CamSync )

find_package( HAL REQUIRED )

include_directories( ${HAL_INCLUDE_DIRS} )

add_executable( CamSync CamSync.cpp )
target_link_libraries( CamSync
                        ${HAL_LIBRARIES}
                        )
