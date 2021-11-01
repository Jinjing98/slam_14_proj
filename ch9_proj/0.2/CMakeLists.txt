cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )




set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/cmake/) #look for FOOconfig.cmake
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)# this is for module mode: look for FindFOO.cmake
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package(OpenCV  REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)
find_package(Sophus  REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/opencv_contrib/modules/viz/include)

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba opencv_viz # in order to use opencv_viz module, we have to explicitly put it here.
)
############### source and test ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
