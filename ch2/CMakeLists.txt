Project(SLAMPROJ)



#include_directories(mylib PUBLIC ./)
add_library(mylib SHARED mylibfunc.cpp)
target_include_directories(mylib PUBLIC ./)



add_executable(helloslam helloSLAM.cpp)
target_include_directories(helloslam PUBLIC ./)
target_link_libraries(helloslam mylib)


