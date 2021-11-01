cmake_minimum_required(VERSION 3.16)
Project(useopencv)
 


set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/cmake/)
find_package(Sophus  REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(PCL REQUIRED COMPONENT common io filters)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
 

include_directories(${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} )
add_executable(useopencv useOpencv.cpp)
target_link_libraries(useopencv ${OpenCV_LIBS})


add_executable(joinmap joinMap.cpp)
target_link_libraries(joinmap ${OpenCV_LIBS} ${PCL_LIBRARY_DIRS} ${PCL_LIBRARIES} )

 
