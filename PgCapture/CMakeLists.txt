project( PgCapture )
cmake_minimum_required(VERSION 2.8)

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( Boost COMPONENTS thread system REQUIRED)

include_directories(/usr/include/flycapture
                    ${Pangolin_INCLUDE_DIRS}
                    ${SceneGraph_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR} )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -O3 -std=c++0x" )

link_directories( ${Pangolin_LIBRARY_DIRS} )

link_libraries( flycapture
                ${Pangolin_LIBRARIES}
                ${SceneGraph_LIBRARIES}
                ${Boost_SYSTEM_LIBRARY}
                ${Boost_THREAD_LIBRARY} )

add_executable( PgCapture PgCapture.cpp )

