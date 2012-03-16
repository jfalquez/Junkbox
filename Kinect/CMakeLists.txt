cmake_minimum_required( VERSION 2.8 )

project( Kinect )

# Add to module path, so we can find our cmake modules
set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules ${CMAKE_MODULE_PATH} )

find_package( OpenNI REQUIRED )
find_package( OpenGL REQUIRED )
find_package( GLUT REQUIRED )
#find_package( FLTK REQUIRED )

include_directories( ${OPENNI_INCLUDE_DIRS} )
#include_directories( ${FLTK_INCLUDE_DIR} )

add_executable ( kinect kinect.cpp)

target_link_libraries ( kinect ${OPENNI_LIBRARY} )
#target_link_libraries ( kinect ${OPENGL_LIBRARIES} )
target_link_libraries ( kinect ${GLUT_LIBRARIES} )
target_link_libraries( MochaGui ${FLTK_LIBRARIES} )
