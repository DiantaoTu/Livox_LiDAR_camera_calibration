cmake_minimum_required(VERSION 3.4)
project(livox_calib)

set(CMAKE_CXX_STANDARD 11)


IF (CMAKE_BUILD_TYPE MATCHES "Debug")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Debug")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
ELSEIF (CMAKE_BUILD_TYPE MATCHES "Release")
        MESSAGE(STATUS "CMAKE_BUILD_TYPE = Release")
ELSE()
        set(CMAKE_BUILD_TYPE "Release")
        MESSAGE(STATUS "Set defalult CMAKE_BUILD_TYPE = Release")
ENDIF()


find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES("/usr/local/include/eigen3/")

find_package(Boost REQUIRED COMPONENTS system filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})
link_directories( ${Boost_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})



aux_source_directory(. src)
add_executable(${PROJECT_NAME} ${src})

target_link_libraries(${PROJECT_NAME} 
                        ${PCL_LIBRARIES}  
                        ${Boost_LIBRARIES} 
                        ${OpenCV_LIBRARIES}
                        ${CERES_LIBRARIES})
