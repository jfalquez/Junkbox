cmake_minimum_required( VERSION 2.8 )

project( DenseESM )

find_package( SimpleGui REQUIRED )
find_package( MVL REQUIRED )
find_package( CVARS REQUIRED )

include_directories( ${SimpleGui_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${CVARS_INCLUDE_DIR} )

add_executable( DESM DESM.cpp )

target_link_libraries( DESM
                        ${SimpleGui_LIBRARIES}
                        ${MVL_LIBRARIES}
                        ${CVARS_LIBRARIES}
                     )
