CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

PROJECT( DTrack )

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules/" )

# Overide with cmake -DCMAKE_BUILD_TYPE=Debug {dir}
IF( NOT CMAKE_BUILD_TYPE )
   MESSAGE("Build type not set (defaults to release)")
   MESSAGE("-DCMAKE_BUILD_TYPE=Debug for debug")
   SET( CMAKE_BUILD_TYPE Release )
ENDIF()

ADD_SUBDIRECTORY( DenseMap )
ADD_SUBDIRECTORY( DenseFrontEnd )
#ADD_SUBDIRECTORY( DenseBackEnd )
ADD_SUBDIRECTORY( Applications )
