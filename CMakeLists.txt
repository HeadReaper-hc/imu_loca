

CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

add_definitions(-std=c++11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

ADD_SUBDIRECTORY( test )

