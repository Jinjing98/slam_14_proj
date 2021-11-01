cmake_minimum_required(VERSION 2.8)
project(ch8)

 




set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/cmake/) #look for FOOconfig.cmake
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)# this is for module mode: look for FindFOO.cmake

find_package(OpenCV  REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)
find_package(Sophus  REQUIRED)
find_package(Ceres REQUIRED)
 
#find_package(Pangolin REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})


add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS}  ${Sophus_LIBRARY} )#${Pangolin_LIBRARIES})



add_executable(useLK useLK.cpp)
target_link_libraries(useLK ${OpenCV_LIBS}  ${Sophus_LIBRARY} )#${Pangolin_LIBRARIES})




#add_executable( direct_sparse direct_sparse.cpp )
#target_link_libraries( direct_sparse ${OpenCV_LIBS}   ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} g2o_types_sba  )

#add_executable( direct_semidense direct_semidense.cpp )
#target_link_libraries( direct_semidense ${OpenCV_LIBS}        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} g2o_types_sba)
