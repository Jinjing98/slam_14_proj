cmake_minimum_required(VERSION 3.16)
Project(USECERES)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/cmake/) #look for FOOconfig.cmake
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)# this is for module mode: look for FindFOO.cmake
find_package(OpenCV  REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(g2o REQUIRED)
find_package(Sophus  REQUIRED)
 


include_directories(${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS} ${Sophus_INCLUDE_DIR})

add_executable(useceres useCERES.cpp)
target_link_libraries(useceres ${OpenCV_LIBS} ${CERES_LIBRARIES} ${Sophus_LIBRARY} )


add_executable(useg2o useG2O.cpp)
target_link_libraries(useg2o  ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${CERES_LIBRARIES} ${Sophus_LIBRARY})
