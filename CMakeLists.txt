################################
# Project configuration
################################
cmake_minimum_required( VERSION 2.8 )
project( TPAvion )
set(CMAKE_VERBOSE_MAKEFILE ON)

################################
# OpenSceneGraph
################################
find_package(OpenSceneGraph 3.2.1 REQUIRED osg osgDB osgGA osgUtil osgViewer)
INCLUDE_DIRECTORIES( ${OPENSCENEGRAPH_INCLUDE_DIR} )
MESSAGE( "${OPENSCENEGRAPH_INCLUDE_DIR}" )

################################
# GTest
################################
find_package(GTest)
enable_testing()
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

################################
# Main App
################################
set(CMAKE_BUILD_TYPE "Release")

set(SRCEXT "cpp")
file( GLOB
source_files
*.${SRCEXT}
)

add_executable( tpavion ${source_files} )
set_target_properties( tpavion PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
TARGET_LINK_LIBRARIES( tpavion ${OPENSCENEGRAPH_LIBRARIES} )

################################
# Unit Tests
################################

file( GLOB
test_source_files
Testing/Sources/*.${SRCEXT}
)

add_executable( runUnitTests ${test_source_files} )
set_target_properties( runUnitTests PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/Testing/bin")
target_link_libraries(runUnitTests gtest gtest_main)
add_test( runUnitTests runUnitTests )
