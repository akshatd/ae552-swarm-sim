cmake_minimum_required(VERSION 3.27)

project(drone-swarm-sim)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Debug")
endif ()
# https://github.com/cpp-best-practices/cppbestpractices/blob/master/02-Use_the_Tools_Available.md#gcc--clang
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

# set up deps for matplot++
include(FetchContent)
FetchContent_Declare(
				matplotplusplus
        GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
        GIT_TAG origin/master)
FetchContent_GetProperties(matplotplusplus)
if(NOT matplotplusplus_POPULATED)
		FetchContent_Populate(matplotplusplus)
		add_subdirectory(${matplotplusplus_SOURCE_DIR} ${matplotplusplus_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

# setup some vars
set(SRC_DIR src)
set(LIB_DIR lib)
set(TEST_DIR test)

# libraries
# https://cmake.org/cmake/help/latest/command/add_library.html

add_library(drone STATIC ${LIB_DIR}/drone.h ${LIB_DIR}/drone.cc)
target_include_directories(drone PUBLIC ${LIB_DIR})

add_library(ui STATIC ${LIB_DIR}/ui.h ${LIB_DIR}/ui.cc)
target_include_directories(ui PUBLIC ${LIB_DIR})
target_link_libraries(ui matplot)

add_library(world STATIC ${LIB_DIR}/world.h ${LIB_DIR}/world.cc)
target_include_directories(world PUBLIC ${LIB_DIR})

# executables

add_executable(drone-swarm-sim ${SRC_DIR}/drone-swarm-sim.cc)
target_link_libraries(drone-swarm-sim drone ui world CONAN_PKG::cxxopts CONAN_PKG::nlohmann_json)
