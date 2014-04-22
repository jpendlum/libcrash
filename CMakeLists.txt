PROJECT(libcrash)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(libsrcs libcrash.c)
SET(libhdrs libcrash.h)

# create shared library named "myhello-cmakelib"
ADD_LIBRARY(libcrash-cmake SHARED ${libsrcs})

# mark library as installable 
INSTALL(TARGETS libcrash-cmake LIBRARY DESTINATION lib)
install(FILES libhdrs DESTINATION include)
