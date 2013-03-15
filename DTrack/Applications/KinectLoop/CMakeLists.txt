CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( KinectLoop )


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

