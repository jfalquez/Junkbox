project( DiskLogger )
cmake_minimum_required(VERSION 2.8)

include( cmake_modules/Common.cmake )

find_package( Pangolin REQUIRED )
find_package( SceneGraph REQUIRED )
find_package( RPG REQUIRED )
find_package( Boost COMPONENTS thread system REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${Pangolin_INCLUDE_DIRS}
                    ${SceneGraph_INCLUDE_DIRS}
                    ${RPG_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${OpenCV2_INCLUDE_DIRS} )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Wall -Wextra -O3 -std=c++0x" )

link_directories( ${Pangolin_LIBRARY_DIRS} )

link_libraries( ${Pangolin_LIBRARIES}
                ${SceneGraph_LIBRARIES}
                ${RPG_LIBRARIES}
                ${Boost_SYSTEM_LIBRARY}
                ${Boost_THREAD_LIBRARY}
                ${OpenCV2_LIBRARIES} )

add_executable( Logger Logger.cpp DiskLogger.cpp )

