SET( DENSE_FRONT_END_HDRS 
        DTrackConfig.h
   )

SET( DENSE_FRONT_END_SRCS
        GpuHelpers.cpp
        Keyframes.cpp
   )

ADD_LIBRARY( DenseFrontEnd ${DENSE_FRONT_END_HDRS} ${DENSE_FRONT_END_SRCS} )
