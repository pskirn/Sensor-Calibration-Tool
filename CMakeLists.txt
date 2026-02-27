cmake_minimum_required(VERSION 3.16)
project(camera_lidar_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation sample_consensus)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(yaml-cpp REQUIRED)

# # Debug prints – very important right now
# message(STATUS "----------------------------------------")
# message(STATUS "OpenCV_VERSION:     ${OpenCV_VERSION}")
# message(STATUS "OpenCV_INCLUDE_DIRS:${OpenCV_INCLUDE_DIRS}")
# message(STATUS "OpenCV_LIBS:        ${OpenCV_LIBS}")
# message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")   # common var name
# message(STATUS "EIGEN3_INCLUDE_DIRS:${EIGEN3_INCLUDE_DIRS}") # sometimes this one
# message(STATUS "----------------------------------------")


add_definitions(${PCL_DEFINITIONS})

set(SOURCES src/main.cpp)


add_executable(${PROJECT_NAME} src/main.cpp src/config.cpp src/camera_detector.cpp)


target_include_directories(
    ${PROJECT_NAME}
    PRIVATE
    include/ 
    ${OpenCV_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # ${EIGEN3_INCLUDE_DIRS}

)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    Ceres::ceres
    yaml-cpp
)

target_compile_options(
    ${PROJECT_NAME}
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)