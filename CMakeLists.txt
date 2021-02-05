cmake_minimum_required( VERSION 3.10 FATAL_ERROR )
set( CMAKE_VERBOSE_MAKEFILE ON )

project( GazeTracking )

#set(CINDER_PATH "/home/martin/devel/Cinder")
get_filename_component( APP_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE )

include( "${CINDER_PATH}/proj/cmake/modules/cinderMakeApp.cmake" )

ci_make_app(
	SOURCES     ${APP_PATH}/main.cpp
	CINDER_PATH ${CINDER_PATH}
        LIBRARIES realsense2 zmq
        INCLUDES ./third-party
        ASSETS_PATH ${APP_PATH}/data/
)
