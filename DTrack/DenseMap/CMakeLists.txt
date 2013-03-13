SET( DENSE_MAP_HDRS 
        TransformEdge.h
        ReferenceFrame.h
        DenseMap.h
   )

SET( DENSE_MAP_SRCS       
        ReferenceFrame.cpp        
        TransformEdge.cpp
        DenseMap.cpp
   )

ADD_LIBRARY( DenseMap ${DENSE_MAP_SRCS} ${DENSE_MAP_HDRS} )
