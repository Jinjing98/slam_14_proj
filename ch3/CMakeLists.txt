Project(useEIGEN)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR} /usr/include/laka_error)
include_directories(${EIGEN3_INCLUDE_DIR}  )
#include_directories( "/usr/include/eigen3" )





add_executable(useeigen useEigen.cpp)
add_executable(useeigenGeo useEigenGeometry.cpp)

#target_include_directories(useeigen PUBLIC  /usr/include/eigen3)
#include_directories( "/usr/include/eigen3" )
 

