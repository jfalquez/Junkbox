cmake_minimum_required( VERSION 2.8 )

project( MeshLogger )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( MVL REQUIRED )
find_package( CVARS REQUIRED )
find_package( OpenCV REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${CVARS_INCLUDE_DIR} )
include_directories( ${OpenCV2_INCLUDE_DIRS} )

link_directories( ${Pangolin_LIBRARY_DIRS} )

add_executable( MeshLogger MeshLogger.cpp )

target_link_libraries( MeshLogger
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${MVL_LIBRARIES}
                        ${CVARS_LIBRARIES}
                        ${OpenCV2_LIBRARIES}
                     )
