cmake_minimum_required( VERSION 2.8 )

project( Matcher )

include( Common.cmake )

find_package( OpenCV REQUIRED )
find_package( FLTK REQUIRED )
find_package( OpenGL REQUIRED )
find_package( CVARS REQUIRED )
find_package( RPG REQUIRED )
find_package( MVL REQUIRED )

include_directories( ${RPG_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( matcher main.cpp )
target_link_libraries( matcher
       	${OpenCV_LIBS} 
	${FLTK_LIBRARIES}
       	${OPENGL_LIBRARIES} 
	${CVARS_LIBRARIES}
	${MVL_LIBRARIES}
	${RPG_LIBRARIES}
	)
