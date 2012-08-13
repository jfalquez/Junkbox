################################################################
################################################################
# Some simple and standard build settings.  The following are defined:
# 
# To use this file, add "include( Common.cmake )" to the top of your
# CMakeLists.txt
#
################################################################
################################################################

# Add to module path, so we can find our cmake modules
SET( CMAKE_MODULE_PATH 
        ${CMAKE_SOURCE_DIR}/cmake_modules
        ${CMAKE_MODULE_PATH}
   )

################### SET BUILD TYPE OPTIONS ######################
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
    # Verbose compile when debugging, and lots of optimization otherwise
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -g -pg")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g -pg")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Debug" )
 IF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -Wall -Wextra -O3 -funroll-loops -finline-functions -mmmx -msse2 ")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Wextra -O3 -funroll-loops -finline-functions -mmmx -msse2 ")
ENDIF( "${CMAKE_BUILD_TYPE}" STREQUAL "Release" )

############### Load CPATH and LIBRARY_PATH ##################
string( REPLACE ":" ";" COMMON_INCLUDE_DIRS "$ENV{CPATH}" ) 
string( REPLACE ":" ";" COMMON_LINK_DIRS "$ENV{LIBRARY_PATH}" ) 
list( REMOVE_DUPLICATES COMMON_INCLUDE_DIRS )
list( REMOVE_DUPLICATES COMMON_LINK_DIRS )

# Hack to get eclipse working
if( APPLE )
    SET( CMAKE_ECLIPSE_EXECUTABLE /Applications/eclipse/eclipse CACHE STRING "" FORCE )
endif()

