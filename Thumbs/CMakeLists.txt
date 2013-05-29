cmake_minimum_required( VERSION 2.8 )

project( Thumbs )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( HAL REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${HAL_INCLUDE_DIRS} )


IF(CMAKE_COMPILER_IS_GNUCXX)
 SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
ENDIF()


add_executable( Thumbs main.cpp Thumbnails.h Thumbnails.cpp )

target_link_libraries( Thumbs
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${HAL_LIBRARIES}
                     )
