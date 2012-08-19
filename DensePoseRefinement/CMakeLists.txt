cmake_minimum_required( VERSION 2.8 )

project( DensePR )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( Sophus REQUIRED )
find_package( MVL REQUIRED )
find_package( CVARS REQUIRED )
find_package( OpenCV REQUIRED )

include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${Sophus_INCLUDE_DIR} )
include_directories( ${MVL_INCLUDE_DIRS} )
include_directories( ${OpenCV2_INCLUDE_DIRS} )

link_directories( ${Pangolin_LIBRARY_DIRS} )
link_directories( ${Sophus_LIBRARY_DIRS} )

add_executable( DensePR DensePR.cpp LinearSystem.cpp )

target_link_libraries( DensePR
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${Sophus_LIBRARIES}
                        ${MVL_LIBRARIES}
                        ${OpenCV2_LIBRARIES}
                     )


#add_executable( DensePRwRU DensePRwRU.cpp LinearSystem.cpp )

#target_link_libraries( DensePRwRU
#                        ${Pangolin_LIBRARIES}
#                        ${SceneGraph_LIBRARIES}
#                        ${Sophus_LIBRARIES}
#                        ${MVL_LIBRARIES}
#                        ${OpenCV2_LIBRARIES}
#                     )
