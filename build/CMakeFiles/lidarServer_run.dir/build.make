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
CMAKE_SOURCE_DIR = /home/team0/Documents/LidarServer_Design3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/team0/Documents/LidarServer_Design3/build

# Include any dependencies generated for this target.
include CMakeFiles/lidarServer_run.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lidarServer_run.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lidarServer_run.dir/flags.make

CMakeFiles/lidarServer_run.dir/main.cpp.o: CMakeFiles/lidarServer_run.dir/flags.make
CMakeFiles/lidarServer_run.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/team0/Documents/LidarServer_Design3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lidarServer_run.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidarServer_run.dir/main.cpp.o -c /home/team0/Documents/LidarServer_Design3/main.cpp

CMakeFiles/lidarServer_run.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidarServer_run.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/team0/Documents/LidarServer_Design3/main.cpp > CMakeFiles/lidarServer_run.dir/main.cpp.i

CMakeFiles/lidarServer_run.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidarServer_run.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/team0/Documents/LidarServer_Design3/main.cpp -o CMakeFiles/lidarServer_run.dir/main.cpp.s

# Object files for target lidarServer_run
lidarServer_run_OBJECTS = \
"CMakeFiles/lidarServer_run.dir/main.cpp.o"

# External object files for target lidarServer_run
lidarServer_run_EXTERNAL_OBJECTS =

lidarServer_run: CMakeFiles/lidarServer_run.dir/main.cpp.o
lidarServer_run: CMakeFiles/lidarServer_run.dir/build.make
lidarServer_run: lidarServer_lib/liblidarServer_lib.a
lidarServer_run: CMakeFiles/lidarServer_run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/team0/Documents/LidarServer_Design3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lidarServer_run"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidarServer_run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lidarServer_run.dir/build: lidarServer_run

.PHONY : CMakeFiles/lidarServer_run.dir/build

CMakeFiles/lidarServer_run.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lidarServer_run.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lidarServer_run.dir/clean

CMakeFiles/lidarServer_run.dir/depend:
	cd /home/team0/Documents/LidarServer_Design3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/team0/Documents/LidarServer_Design3 /home/team0/Documents/LidarServer_Design3 /home/team0/Documents/LidarServer_Design3/build /home/team0/Documents/LidarServer_Design3/build /home/team0/Documents/LidarServer_Design3/build/CMakeFiles/lidarServer_run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lidarServer_run.dir/depend

