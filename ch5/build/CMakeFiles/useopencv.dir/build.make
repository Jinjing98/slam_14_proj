# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jinjing/Projects/SLAM/slam_jinjing/ch5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/Projects/SLAM/slam_jinjing/ch5/build

# Include any dependencies generated for this target.
include CMakeFiles/useopencv.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/useopencv.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useopencv.dir/flags.make

CMakeFiles/useopencv.dir/useOpencv.cpp.o: CMakeFiles/useopencv.dir/flags.make
CMakeFiles/useopencv.dir/useOpencv.cpp.o: ../useOpencv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/Projects/SLAM/slam_jinjing/ch5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useopencv.dir/useOpencv.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/useopencv.dir/useOpencv.cpp.o -c /home/jinjing/Projects/SLAM/slam_jinjing/ch5/useOpencv.cpp

CMakeFiles/useopencv.dir/useOpencv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useopencv.dir/useOpencv.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/Projects/SLAM/slam_jinjing/ch5/useOpencv.cpp > CMakeFiles/useopencv.dir/useOpencv.cpp.i

CMakeFiles/useopencv.dir/useOpencv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useopencv.dir/useOpencv.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/Projects/SLAM/slam_jinjing/ch5/useOpencv.cpp -o CMakeFiles/useopencv.dir/useOpencv.cpp.s

# Object files for target useopencv
useopencv_OBJECTS = \
"CMakeFiles/useopencv.dir/useOpencv.cpp.o"

# External object files for target useopencv
useopencv_EXTERNAL_OBJECTS =

useopencv: CMakeFiles/useopencv.dir/useOpencv.cpp.o
useopencv: CMakeFiles/useopencv.dir/build.make
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_gapi.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_highgui.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_ml.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_objdetect.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_photo.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_stitching.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_video.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_videoio.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_imgcodecs.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_dnn.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_calib3d.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_features2d.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_flann.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_imgproc.so.4.5.4
useopencv: /home/jinjing/Projects/SLAM/slam_jinjing/3rd_pkg/opencv/build/lib/libopencv_core.so.4.5.4
useopencv: CMakeFiles/useopencv.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/Projects/SLAM/slam_jinjing/ch5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useopencv"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useopencv.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useopencv.dir/build: useopencv

.PHONY : CMakeFiles/useopencv.dir/build

CMakeFiles/useopencv.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useopencv.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useopencv.dir/clean

CMakeFiles/useopencv.dir/depend:
	cd /home/jinjing/Projects/SLAM/slam_jinjing/ch5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/Projects/SLAM/slam_jinjing/ch5 /home/jinjing/Projects/SLAM/slam_jinjing/ch5 /home/jinjing/Projects/SLAM/slam_jinjing/ch5/build /home/jinjing/Projects/SLAM/slam_jinjing/ch5/build /home/jinjing/Projects/SLAM/slam_jinjing/ch5/build/CMakeFiles/useopencv.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useopencv.dir/depend

