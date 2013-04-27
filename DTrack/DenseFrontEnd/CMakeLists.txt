SET( DENSE_FRONT_END_HDRS
        Common.h
        DenseFrontEnd.h
        DenseFrontEndConfig.h
        DTrackVicon.h
        DTrackViconConfig.h
        FastLocalizer.h
        FastLocalizerConfig.h
        GpuHelpers.h
        Timer.h
        Vicon.h
   )

SET( DENSE_FRONT_END_SRCS
        DenseFrontEnd.cpp
        DTrackVicon.cpp
        FastLocalizer.cpp
        GpuHelpers.cpp
        Vicon.cpp
   )

ADD_LIBRARY( DenseFrontEnd ${DENSE_FRONT_END_HDRS} ${DENSE_FRONT_END_SRCS} )
