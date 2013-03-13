CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( KinectLoop )

FIND_PACKAGE( Eigen3 REQUIRED )
FIND_PACKAGE( RPG REQUIRED )
FIND_PACKAGE( Pangolin REQUIRED )
FIND_PACKAGE( SceneGraph REQUIRED )
FIND_PACKAGE( CVARS REQUIRED )
FIND_PACKAGE( Kangaroo REQUIRED )
FIND_PACKAGE( Sophus REQUIRED )
#FIND_PACKAGE( CeresSolver REQUIRED )
#FIND_PACKAGE( SensorFusion REQUIRED )

INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR} )
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${RPG_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Pangolin_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${SceneGraph_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${CVARS_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${Kangaroo_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES( ${Sophus_INCLUDE_DIR} )
#INCLUDE_DIRECTORIES( ${CeresSolver_INCLUDE_DIRS} )
#INCLUDE_DIRECTORIES( ${SENSORFUSION_INCLUDE_DIRS} )

LINK_DIRECTORIES( ${Pangolin_LIBRARY_DIRS} )
LINK_DIRECTORIES( ${Sophus_LIBRARY_DIRS} )
#LINK_DIRECTORIES( ${CeresSolver_LIBRARY_DIRS} )

IF( CMAKE_COMPILER_IS_GNUCXX )
 SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
ENDIF()

LIST( APPEND HDRS
        DTrackApp.h
        Gui.h
        GuiConfig.h
    )

LIST( APPEND SRCS
        main.cpp
    )


ADD_EXECUTABLE( KinectLoop ${HDRS} ${SRCS} )


TARGET_LINK_LIBRARIES( KinectLoop
                        DenseMap
                        DenseFrontEnd
                        ${RPG_LIBRARIES}
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${CVARS_LIBRARIES}
                        ${Kangaroo_LIBRARIES}
                        ${Sophus_LIBRARIES}
#                        ${CeresSolver_LIBRARIES}
#                        ${SENSORFUSION_LIBRARIES}
                     )

