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
CMAKE_SOURCE_DIR = /home/jinjing/Projects/SLAM/slam_jinjing/ch3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/Projects/SLAM/slam_jinjing/ch3/build

# Include any dependencies generated for this target.
include CMakeFiles/useeigen.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/useeigen.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/useeigen.dir/flags.make

CMakeFiles/useeigen.dir/useEigen.o: CMakeFiles/useeigen.dir/flags.make
CMakeFiles/useeigen.dir/useEigen.o: ../useEigen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/Projects/SLAM/slam_jinjing/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/useeigen.dir/useEigen.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/useeigen.dir/useEigen.o -c /home/jinjing/Projects/SLAM/slam_jinjing/ch3/useEigen.cpp

CMakeFiles/useeigen.dir/useEigen.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/useeigen.dir/useEigen.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/Projects/SLAM/slam_jinjing/ch3/useEigen.cpp > CMakeFiles/useeigen.dir/useEigen.i

CMakeFiles/useeigen.dir/useEigen.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/useeigen.dir/useEigen.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/Projects/SLAM/slam_jinjing/ch3/useEigen.cpp -o CMakeFiles/useeigen.dir/useEigen.s

# Object files for target useeigen
useeigen_OBJECTS = \
"CMakeFiles/useeigen.dir/useEigen.o"

# External object files for target useeigen
useeigen_EXTERNAL_OBJECTS =

useeigen: CMakeFiles/useeigen.dir/useEigen.o
useeigen: CMakeFiles/useeigen.dir/build.make
useeigen: CMakeFiles/useeigen.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/Projects/SLAM/slam_jinjing/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable useeigen"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/useeigen.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/useeigen.dir/build: useeigen

.PHONY : CMakeFiles/useeigen.dir/build

CMakeFiles/useeigen.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/useeigen.dir/cmake_clean.cmake
.PHONY : CMakeFiles/useeigen.dir/clean

CMakeFiles/useeigen.dir/depend:
	cd /home/jinjing/Projects/SLAM/slam_jinjing/ch3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/Projects/SLAM/slam_jinjing/ch3 /home/jinjing/Projects/SLAM/slam_jinjing/ch3 /home/jinjing/Projects/SLAM/slam_jinjing/ch3/build /home/jinjing/Projects/SLAM/slam_jinjing/ch3/build /home/jinjing/Projects/SLAM/slam_jinjing/ch3/build/CMakeFiles/useeigen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/useeigen.dir/depend

