SET( DENSE_FRONT_END_HDRS
        DenseFrontEnd.h
        DenseFrontEndConfig.h
        GpuHelpers.h
   )

SET( DENSE_FRONT_END_SRCS
        DenseFrontEnd.cpp
        GpuHelpers.cpp
   )

ADD_LIBRARY( DenseFrontEnd ${DENSE_FRONT_END_HDRS} ${DENSE_FRONT_END_SRCS} )
