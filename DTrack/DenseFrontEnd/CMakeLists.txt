SET( DENSE_FRONT_END_HDRS
        DenseFrontEnd.h
        DenseFrontEndConfig.h

   )

SET( DENSE_FRONT_END_SRCS
        DenseFrontEnd.cpp
   )

ADD_LIBRARY( DenseFrontEnd ${DENSE_FRONT_END_HDRS} ${DENSE_FRONT_END_SRCS} )
