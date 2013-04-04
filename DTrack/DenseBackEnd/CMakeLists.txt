SET( DENSE_BACK_END_HDRS
        DenseBackEnd.h
        DenseBackEndConfig.h
   )

SET( DENSE_BACK_END_SRCS
        DenseBackEnd.cpp
   )

ADD_LIBRARY( DenseBackEnd ${DENSE_BACK_END_HDRS} ${DENSE_BACK_END_SRCS} )
