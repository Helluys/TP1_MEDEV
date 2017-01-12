################################
# Project configuration
################################
cmake_minimum_required( VERSION 2.8 )
project( TPAvion )
set( CMAKE_VERBOSE_MAKEFILE ON )

################################
# OpenSceneGraph
################################
find_package( OpenSceneGraph 3.2.1 REQUIRED osg osgDB osgGA osgUtil osgViewer )
include_directories( ${OPENSCENEGRAPH_INCLUDE_DIR} )

################################
# GTest
################################
find_package( GTest )
enable_testing( )
include_directories( ${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR} )

################################
# Main App
################################
set( CMAKE_BUILD_TYPE "Release" )

file( GLOB
source_files
src/*.cpp
 )

add_executable( tpavion ${source_files} )
set_target_properties( tpavion PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin" )
target_link_libraries( tpavion ${OPENSCENEGRAPH_LIBRARIES} )

################################
# Unit Tests
################################

file( GLOB
test_source_files
Testing/src/*.cpp
 )

add_executable( runUnitTests ${test_source_files} )
set_target_properties( runUnitTests PROPERTIES 
                            RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing/bin"
                            COMPILE_FLAGS --coverage )
include_directories( ${CMAKE_SOURCE_DIR}/include ) # need to include the main app headers
target_link_libraries( runUnitTests gtest gtest_main gcov )
add_test( runUnitTests Testing/bin/runUnitTests )

################################
# gcov - lcov
################################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules)
if(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    setup_target_for_coverage(tpavion_coverage runUnitTests coverage)
endif()
