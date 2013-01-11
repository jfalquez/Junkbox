cmake_minimum_required( VERSION 2.8 )

project( Calibrate )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( RPG REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${RPG_INCLUDE_DIRS} )

link_directories( ${Pangolin_LIBRARY_DIRS} )

IF(CMAKE_COMPILER_IS_GNUCXX)
 SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
ENDIF()

list(APPEND HDRS
)

list(APPEND SRCS
    Calibrate.cpp
)


add_executable( Calibrate ${HDRS} ${SRCS} )

target_link_libraries( Calibrate
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${RPG_LIBRARIES}
                     )
