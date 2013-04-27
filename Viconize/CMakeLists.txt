cmake_minimum_required( VERSION 2.8 )

project( Viconize )

find_package( RPG REQUIRED )

include_directories( ${RPG_INCLUDE_DIRS} )

add_executable( viconize main.cpp )
target_link_libraries( viconize
                        ${RPG_LIBRARIES}
                        )
