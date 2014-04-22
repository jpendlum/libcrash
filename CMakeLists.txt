PROJECT(libcrash)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
INCLUDE_DIRECTORIES(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR})
SET(LIB_SRCS libcrash.c)
SET(LIB_HDRS libcrash.h)

# create shared library named "myhello-cmakelib"
ADD_LIBRARY(crash SHARED ${LIB_SRCS})

# Link with udev
FIND_LIBRARY(UDEV_LIBS udev)
TARGET_LINK_LIBRARIES(crash ${UDEV_LIBS})

# mark library as installable 
INSTALL(TARGETS crash LIBRARY DESTINATION lib)
INSTALL(FILES ${LIB_HDRS} DESTINATION include)

# uninstall target
CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
