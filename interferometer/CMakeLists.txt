if(" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
  message(FATAL_ERROR "
FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.
       Try 'cmake ..' in build directory.
")
endif()

cmake_minimum_required( VERSION 2.8 )
project( interferometer )
set( OpenCV_STATIC ON )
set( CMAKE_EXE_LINKER_FLAGS -static )
find_package( OpenCV REQUIRED )
find_package( raspicam )
target_compile_definitions(interferometer PRIVATE RASPICAM=${raspicam_VERSION} OPENCV=${OpenCV_VERSION})
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( interferometer interferometer.cpp )
target_link_libraries( interferometer ${OpenCV_LIBS} )
