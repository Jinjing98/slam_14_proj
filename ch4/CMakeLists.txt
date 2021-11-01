Project(USESOPHUS)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Sophus  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}  ${Sophus_INCLUDE_DIR})

#CMAKE_MODULE_PATH()
#CMAKE_PREFIX_PATH



add_executable(usesophus useSophus.cpp)
target_link_libraries(usesophus ${Sophus_LIBRARY})
