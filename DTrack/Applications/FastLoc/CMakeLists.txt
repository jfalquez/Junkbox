CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( FastLoc )

LIST( APPEND HDRS
        DTrackApp.h
        Gui.h
        GuiConfig.h
    )

LIST( APPEND SRCS
        main.cpp
    )


ADD_EXECUTABLE( FastLoc ${HDRS} ${SRCS} )


TARGET_LINK_LIBRARIES( FastLoc
                        DenseMap
                        DenseFrontEnd
                        DenseBackEnd
                        ${RPG_LIBRARIES}
                        ${Pangolin_LIBRARIES}
                        ${SceneGraph_LIBRARIES}
                        ${CVARS_LIBRARIES}
                        ${Kangaroo_LIBRARIES}
                        ${Sophus_LIBRARIES}
                        ${CeresSolver_LIBRARIES}
                        ${CHOLMOD_LIBRARIES}
#                        ${SENSORFUSION_LIBRARIES}
                     )

