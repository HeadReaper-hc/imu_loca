INCLUDE_DIRECTORIES( "../include" )


set( eigen3_dir "/usr/include/eigen3" )
INCLUDE_DIRECTORIES( ${eigen3_dir} )

#set( fast_include_dir ${CMAKE_SOURCE_DIR}/3rdparty/fast/include/ )
#INCLUDE_DIRECTORIES( ${fast_include_dir} )

#set( sophus_include_dir ${CMAKE_SOURCE_DIR}/3rdparty/sophus/include )
#INCLUDE_DIRECTORIES( ${sophus_include_dir} )


#LINK_DIRECTORIES( "${CMAKE_SOURCE_DIR}/lib/" )

file(GLOB all_source ../src/*.cpp )

ADD_EXECUTABLE( navgation_test navgation_test.cpp ${all_source} )


