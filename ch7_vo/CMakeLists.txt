Project(feature_matching)

set(CMAKE_PREFIX_PATH ${CMAKE_SOURCE_DIR}/cmake/) #look for FOOconfig.cmake
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/)# this is for module mode: look for FindFOO.cmake

find_package(OpenCV  REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(g2o REQUIRED)
find_package(Sophus  REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${Sophus_INCLUDE_DIR} ${CERES_INCLUDE_DIRS} )
add_executable(feature_matching feature_matching.cpp)
target_link_libraries(feature_matching ${OpenCV_LIBS})


add_executable(2d2d 2D2D.cpp)
target_link_libraries(2d2d ${OpenCV_LIBS})
# 
# 
# 
# add_executable(2d2d_ori 2d2d_ori.cpp)
# target_link_libraries(2d2d_ori ${OpenCV_LIBS})
add_executable( 3d2d pose_estimation_3d2d.cpp)
target_link_libraries( 3d2d
        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
        ${OpenCV_LIBS} ${Sophus_LIBRARY}  ${CERES_LIBRARIES})

add_executable( 3d3d pose_estimation_3d3d.cpp)
target_link_libraries( 3d3d
        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY}
        ${OpenCV_LIBS} ${Sophus_LIBRARY}  ${CERES_LIBRARIES})
