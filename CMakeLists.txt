cmake_minimum_required(VERSION 3.27)

project(drone-swarm-sim)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif ()
# https://github.com/cpp-best-practices/cppbestpractices/blob/master/02-Use_the_Tools_Available.md#gcc--clang
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -pedantic -Wshadow -Wnon-virtual-dtor \
										-Wold-style-cast -Wcast-align -Wunused -Woverloaded-virtual -Wconversion -Wsign-conversion \
										-Wmisleading-indentation -Wduplicated-cond -Wduplicated-branches -Wlogical-op -Wnull-dereference \
										-Wuseless-cast -Wdouble-promotion -Wformat=2 -Wpadded")

# install deps via Conan
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
	message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
	file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.18.1/conan.cmake"
								"${CMAKE_BINARY_DIR}/conan.cmake"
								TLS_VERIFY ON)
endif()
include(${CMAKE_BINARY_DIR}/conan.cmake)
configure_file(${CMAKE_SOURCE_DIR}/conanfile.txt ${CMAKE_BINARY_DIR}/conanfile.txt COPYONLY)
set(CONAN_HOST_PROFILE "default" CACHE STRING "Conan profile of target machine") # change this externally for cross compilation
set(CONAN_BUILD_PROFILE "default" CACHE STRING "Conan profile of target machine")
conan_cmake_install(PATH_OR_REFERENCE . BUILD missing PROFILE_HOST ${CONAN_HOST_PROFILE} PROFILE_BUILD ${CONAN_BUILD_PROFILE})

# set up deps vis conan. CMAKE_BINARY_DIR=the build folder
include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
conan_basic_setup(TARGETS)

# setup some vars
set(SRC_DIR src)
set(LIB_DIR lib)
set(TEST_DIR test)

# libraries
# https://cmake.org/cmake/help/latest/command/add_library.html
add_library(drone STATIC ${LIB_DIR}/drone.h ${LIB_DIR}/drone.cc)
target_include_directories(drone PUBLIC ${LIB_DIR})

# executables

add_executable(drone-swarm-sim ${SRC_DIR}/drone-swarm-sim.cc)
target_link_libraries(drone-swarm-sim drone CONAN_PKG::cxxopts CONAN_PKG::nlohmann_json)