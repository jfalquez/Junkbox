SET( DENSE_FRONT_END_HDRS
        DenseFrontEnd.h
        DenseFrontEndConfig.h
        FastLocalizer.h
        FastLocalizerConfig.h
        GpuHelpers.h
        ImageHelpers.h
        Timer.h
   )

SET( DENSE_FRONT_END_SRCS
        DenseFrontEnd.cpp
        FastLocalizer.cpp
        GpuHelpers.cpp
   )

ADD_LIBRARY( DenseFrontEnd ${DENSE_FRONT_END_HDRS} ${DENSE_FRONT_END_SRCS} )
