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


# Calibrate
add_executable( Calibrate Calibrate.cpp )

target_link_libraries( Calibrate
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${RPG_LIBRARIES}
                     )

# Calibrate Circles
add_executable( CalibrateCircles CalibrateCircles.cpp )

target_link_libraries( CalibrateCircles
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${RPG_LIBRARIES}
                     )

# Stereo Calibrate
add_executable( StereoCalibrate StereoCalibrate.cpp )

target_link_libraries( StereoCalibrate
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${RPG_LIBRARIES}
                     )
