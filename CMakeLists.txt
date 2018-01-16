SET(PROJECT_NAME DSO_MWE)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
#set(CMAKE_VERBOSE_MAKEFILE ON)


set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
#set(BUILD_TYPE RelWithDebInfo)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(DSO_DIR "${CMAKE_SOURCE_DIR}/../dso")
set(CMAKE_MODULE_PATH ${DSO_DIR}/cmake)

# required libraries
find_package(SuiteParse REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 

# optional libraries
find_package(LibZip QUIET)
find_package(Pangolin 0.2 QUIET)
find_package(OpenCV QUIET)

# decide if we have ziplib.
if (LIBZIP_LIBRARY)
        message("--- found ziplib (${LIBZIP_VERSION}), compiling with zip capability.")
        add_definitions(-DHAS_ZIPLIB=1)
        include_directories( ${LIBZIP_INCLUDE_DIR_ZIP} ${LIBZIP_INCLUDE_DIR_ZIPCONF} )
else()
        message("--- not found ziplib (${LIBZIP_LIBRARY}), compiling without zip capability.")
        set(LIBZIP_LIBRARY "")
endif()

# flags
add_definitions("-DENABLE_SSE")
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS
       "${SSE_FLAGS} -O0 -g -std=c++0x -march=native"
    )
else()
    set(CMAKE_CXX_FLAGS
       "${SSE_FLAGS} -O3 -g -std=c++0x -march=native"
    )
endif()



if (MSVC)
     set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif (MSVC)

include_directories(
  ${DSO_DIR}/src
  ${DSO_DIR}/thirdparty/Sophus
  ${DSO_DIR}/thirdparty/sse2neon
  ${EIGEN3_INCLUDE_DIR}
) 

# compile main library.
include_directories( ${CSPARSE_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR} ${Pangolin_INCLUDE_DIR} ${OpenCV_INCLUDE_DIR} ${LIBZIP_INCLUDE_DIR})
#add_library(dso ${dso_SOURCE_FILES} ${dso_opencv_SOURCE_FILES} ${dso_pangolin_SOURCE_FILES})
LINK_DIRECTORIES(${DSO_DIR}/build/lib)

#set_property( TARGET dso APPEND_STRING PROPERTY COMPILE_FLAGS -Wall )


if (${CMAKE_SYSTEM_NAME} MATCHES "Darwin") # OSX
    set(BOOST_THREAD_LIBRARY boost_thread-mt)
else()
    set(BOOST_THREAD_LIBRARY boost_thread)
endif()

message("--- compiling dso_mwe")
add_executable(dso_mwe ${PROJECT_SOURCE_DIR}/src/main.cpp )
target_link_libraries(dso_mwe libdso.a boost_system cxsparse ${BOOST_THREAD_LIBRARY} ${LIBZIP_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})

