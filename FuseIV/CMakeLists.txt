cmake_minimum_required( VERSION 2.8 )

project( FuseIV )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( MVL REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )

link_directories( ${Pangolin_LIBRARY_DIRS} )

IF(CMAKE_COMPILER_IS_GNUCXX)
 SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
ENDIF()

list(APPEND HDRS
    GetPot
    SensorFusion.h
    CarPlannerCommon.h
#    GLPath.h
)

list(APPEND SRCS
    FuseIV.cpp
    SensorFusion.cpp
)


add_executable( FuseIV ${HDRS} ${SRCS} )

target_link_libraries( FuseIV
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${MVL_LIBRARIES}
                     )
