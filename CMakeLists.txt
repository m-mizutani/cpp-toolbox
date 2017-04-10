CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(cpptb)


IF(NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, default to Release")
    SET(CMAKE_BUILD_TYPE "Release")
ENDIF()

IF(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
ENDIF()
IF(POLICY CMP0015)
    cmake_policy(SET CMP0015 NEW)
ENDIF()



SET(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -std=c++0x")
SET(CMAKE_CXX_FLAGS_DEBUG   "-Wall -O0 -std=c++0x")
SET(CMAKE_SHARED_LINKER_FLAGS "-lpcap -dynamiclib")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)


INCLUDE_DIRECTORIES(${INC_DIR} ./src)
LINK_DIRECTORIES(${LIB_DIR})
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    INCLUDE_DIRECTORIES(/usr/local/include)
    LINK_DIRECTORIES(/usr/local/lib)
ENDIF()

# Build library

FILE(GLOB BASESRCS
	"src/buffer.cc"	 "src/buffer.hpp"
	"src/hash.cc"	   "src/hash.hpp"
  "src/cpptb.hpp"
  "src/exception.hpp"
  "src/cache.hpp"
)
FILE(GLOB TESTSRCS "test/*.cc"
	"./test/gtest/gtest-all.cc"
	"./test/gtest/main.cc"
	)


# Library
ADD_LIBRARY(cpptb SHARED ${BASESRCS})
IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    TARGET_LINK_LIBRARIES(cpptb pcap pthread rt)
ELSE(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    TARGET_LINK_LIBRARIES(cpptb pcap pthread)
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

# Test code
ADD_EXECUTABLE(cpptb-test ${TESTSRCS})
TARGET_LINK_LIBRARIES(cpptb-test cpptb)

