cmake_minimum_required( VERSION 2.8 )
project( TPAvion )
set(CMAKE_VERBOSE_MAKEFILE ON)

set(SRCEXT "cpp")
file( GLOB
source_files
*.${SRCEXT}
)

find_package(OpenSceneGraph 3.2.1 REQUIRED osg osgDB osgGA osgUtil osgViewer)
INCLUDE_DIRECTORIES( ${OPENSCENEGRAPH_INCLUDE_DIR} )
MESSAGE( "${OPENSCENEGRAPH_INCLUDE_DIR}" )

set(CMAKE_BUILD_TYPE "Release")

add_executable( tpavion ${source_files} )

TARGET_LINK_LIBRARIES( tpavion ${OPENSCENEGRAPH_LIBRARIES} )
MESSAGE( "${OPENSCENEGRAPH_LIBRARIES}" )

