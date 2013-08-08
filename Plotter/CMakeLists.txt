project(Plotter)
cmake_minimum_required(VERSION 2.8)

set( CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules ${CMAKE_MODULE_PATH} )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( HAL 0.1 REQUIRED )
find_package( Pangolin 0.1 REQUIRED )
find_package( SceneGraph 0.1 REQUIRED )
find_package( NODE REQUIRED )

find_package( Protobuf REQUIRED )
include_directories( ${CMAKE_BINARY_DIR} )
PROTOBUF_GENERATE_CPP( PROTO_SRCS PROTO_HDRS Command.proto )

include_directories( ${HAL_INCLUDE_DIRS} )
include_directories( ${Pangolin_INCLUDE_DIRS} )
include_directories( ${SceneGraph_INCLUDE_DIRS} )
include_directories( ${NODE_INCLUDE_DIRS} )

add_subdirectory( Gamepad )

set( HDRS JoystickHandler.h ${PROTO_HDRS} )
set( SRCS main.cpp JoystickHandler.cpp ${PROTO_SRCS} )
add_executable( ${PROJECT_NAME} ${HDRS} ${SRCS} )

link_directories( ${HAL_LINK_DIRS} )
link_directories( ${Pangolin_LIBRARY_DIRS} )

target_link_libraries( ${PROJECT_NAME} ${HAL_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} ${Pangolin_LIBRARIES})
target_link_libraries( ${PROJECT_NAME} ${SceneGraph_LIBRARIES} )
target_link_libraries( ${PROJECT_NAME} Gamepad )
target_link_libraries( ${PROJECT_NAME} ${NODE_LIBRARIES} )
