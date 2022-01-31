cmake_minimum_required(VERSION 2.8)
project(visual_odometry)

SET(CMAKE_BUILD_TYPE Release)
#  -pthread -fopenmp --> threading flags
set(CMAKE_CXX_FLAGS "-Wall -O3 -march=native -pthread -fopenmp -std=c++17 -DENABLE_INTERNAL_FMT=ON")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# pangolin
find_package(Pangolin REQUIRED)

# Sophus
find_package(Sophus REQUIRED)

# G2O
find_package(G2O REQUIRED)

# glog
# find_package(Glog REQUIRED)

# # gtest
# find_package(GTest REQUIRED)

# # gflags
# find_package(GFlags REQUIRED)

# csparse
find_package(CSparse REQUIRED)

# pybind11
find_package(pybind11 REQUIRED)

# fmt
find_package(fmt REQUIRED)

set(INCLUDE_DIRS
    "/usr/include/eigen3"
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    # ${GLOG_INCLUDE_DIRS}
    # ${GTEST_INCLUDE_DIRS}
    # ${GFLAGS_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

set (G2O_LIBRARIES g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension)
 
set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES} GL GLU GLEW glut
    ${G2O_LIBRARIES}
    # ${GTEST_BOTH_LIBRARIES}
    # ${GLOG_LIBRARIES}
    # ${GFLAGS_LIBRARIES}
    pthread
    ${CSPARSE_LIBRARY}
    fmt
)

enable_testing()

include_directories(${INCLUDE_DIRS})

add_executable(main
    src/main.cc
    src/frame.cc
)

target_link_libraries(main 
${THIRD_PARTY_LIBS})


