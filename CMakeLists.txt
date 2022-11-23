cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(geofinder)

if(NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(JSONCPP_WITH_TESTS OFF)
set(JSONCPP_WITH_POST_BUILD_UNITTEST OFF)
set(JSONCPP_WITH_PKGCONFIG_SUPPORT OFF)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_STATIC_LIBS ON)
set(BUILD_OBJECT_LIBS OFF)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/jsoncpp-1.9.5")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party/jsoncpp-1.9.5/include")

find_package(Boost REQUIRED OPTIONAL_COMPONENTS geometry)
include_directories("${Boost_INCLUDE_DIRS}")

if(POLICY CMP0068)
    cmake_policy(SET CMP0068 NEW)
endif()

file(GLOB geofinder_main "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_executable(geofinder
    ${geofinder_main}
)
add_dependencies(geofinder jsoncpp_static)
set_property(TARGET geofinder PROPERTY CXX_STANDARD 14)
target_link_libraries(geofinder PRIVATE jsoncpp_static ${Boost_GEOMETRY_LIBRARY})

if(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /wd4996 /wd4267 /wd4068")
else()
    if (CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wno-deprecated-declarations")
endif()
