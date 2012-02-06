cmake_minimum_required( VERSION 2.8 )

project( AirGround )

include( Common.cmake )

find_package( MVL REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${MVL_INCLUDE_DIRS} )

add_executable( AirGround main.cpp )

target_link_libraries( AirGround
	${MVL_LIBRARIES}
	)
