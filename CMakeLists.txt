# Set the minimum required version of cmake for a project.
cmake_minimum_required(VERSION 3.1)

# Set Variables
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PROJECT_NAME "setup_toolkit")

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

# Project
project(${PROJECT_NAME} CXX)

# Find library package
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(k4a QUIET)

# openMP
FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


# Include directories
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

# Link libraries
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

# File
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/include/*)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

# The main program
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
add_executable(${PROJECT_NAME} main.cpp ${SRC_FILES} ${HEADER_FILES})

# Link
if (k4a_FOUND)
    message("k4a found")
    target_compile_definitions(${PROJECT_NAME} PRIVATE K4A_FOUND)
    target_link_libraries( ${PROJECT_NAME} 
        ${OpenCV_LIBS} 
        k4a::k4a
        Eigen3::Eigen
        ${ZED_LIBS}
    )
else()
    message("k4a not found")
    target_link_libraries( ${PROJECT_NAME} 
        ${OpenCV_LIBS} 
        Eigen3::Eigen
        ${ZED_LIBS}
    )
endif()