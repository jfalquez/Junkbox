cmake_minimum_required( VERSION 2.8 )

project( Viconize )

find_package( HAL REQUIRED )
find_package( MVl REQUIRED )

include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )

add_executable( viconize main.cpp )
target_link_libraries( viconize
                        ${HAL_LIBRARIES}
                        ${MVL_LIBRARIES}
                        )
