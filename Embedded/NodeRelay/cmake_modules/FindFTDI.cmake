# FindFTDI.cmake
#   FTDI_FOUND
#   FTDI_INCLUDE_DIRS
#   FTDI_LIBRARIES

SET(FTDI_POSSIBLE_ROOT_DIRS
        /usr/local                                      # Linux: default dir by CMake
        /usr                                            # Linux
        /opt/local                                      # OS X: default MacPorts location
        ${CMAKE_INSTALL_PREFIX}                         # Android toolchain
)

FIND_PATH(FTDI_INCLUDE_DIR
          NAMES ftd2xx/ftd2xx.h
          PATHS ${FTDI_POSSIBLE_ROOT_DIRS}
          )

FIND_LIBRARY(FTDI_LIBRARY
             NAMES ftd2xx
             PATHS ${FTDI_POSSIBLE_ROOT_DIRS}
             PATH_SUFFIXES lib
            )

SET(FTDI_INCLUDE_DIRS
    ${FTDI_INCLUDE_DIR}
    )

SET(FTDI_LIBRARIES
    ${FTDI_LIBRARY}
    )

MARK_AS_ADVANCED( FTDI_INCLUDE_DIR FTDI_INCLUDE_DIRS FTDI_LIBRARY FTDI_LIBRARIES )
