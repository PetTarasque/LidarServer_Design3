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
include Google_tests/CMakeFiles/Google_Tests_run.dir/depend.make

# Include the progress variables for this target.
include Google_tests/CMakeFiles/Google_Tests_run.dir/progress.make

# Include the compile flags for this target's objects.
include Google_tests/CMakeFiles/Google_Tests_run.dir/flags.make

Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o: Google_tests/CMakeFiles/Google_Tests_run.dir/flags.make
Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o: ../Google_tests/LidarServerTest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/team0/Documents/LidarServer_Design3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o"
	cd /home/team0/Documents/LidarServer_Design3/build/Google_tests && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o -c /home/team0/Documents/LidarServer_Design3/Google_tests/LidarServerTest.cpp

Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.i"
	cd /home/team0/Documents/LidarServer_Design3/build/Google_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/team0/Documents/LidarServer_Design3/Google_tests/LidarServerTest.cpp > CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.i

Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.s"
	cd /home/team0/Documents/LidarServer_Design3/build/Google_tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/team0/Documents/LidarServer_Design3/Google_tests/LidarServerTest.cpp -o CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.s

# Object files for target Google_Tests_run
Google_Tests_run_OBJECTS = \
"CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o"

# External object files for target Google_Tests_run
Google_Tests_run_EXTERNAL_OBJECTS =

Google_tests/Google_Tests_run: Google_tests/CMakeFiles/Google_Tests_run.dir/LidarServerTest.cpp.o
Google_tests/Google_Tests_run: Google_tests/CMakeFiles/Google_Tests_run.dir/build.make
Google_tests/Google_Tests_run: lidarServer_lib/liblidarServer_lib.a
Google_tests/Google_Tests_run: lib/libgtest.a
Google_tests/Google_Tests_run: lib/libgtest_main.a
Google_tests/Google_Tests_run: lib/libgtest.a
Google_tests/Google_Tests_run: Google_tests/CMakeFiles/Google_Tests_run.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/team0/Documents/LidarServer_Design3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Google_Tests_run"
	cd /home/team0/Documents/LidarServer_Design3/build/Google_tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Google_Tests_run.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Google_tests/CMakeFiles/Google_Tests_run.dir/build: Google_tests/Google_Tests_run

.PHONY : Google_tests/CMakeFiles/Google_Tests_run.dir/build

Google_tests/CMakeFiles/Google_Tests_run.dir/clean:
	cd /home/team0/Documents/LidarServer_Design3/build/Google_tests && $(CMAKE_COMMAND) -P CMakeFiles/Google_Tests_run.dir/cmake_clean.cmake
.PHONY : Google_tests/CMakeFiles/Google_Tests_run.dir/clean

Google_tests/CMakeFiles/Google_Tests_run.dir/depend:
	cd /home/team0/Documents/LidarServer_Design3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/team0/Documents/LidarServer_Design3 /home/team0/Documents/LidarServer_Design3/Google_tests /home/team0/Documents/LidarServer_Design3/build /home/team0/Documents/LidarServer_Design3/build/Google_tests /home/team0/Documents/LidarServer_Design3/build/Google_tests/CMakeFiles/Google_Tests_run.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Google_tests/CMakeFiles/Google_Tests_run.dir/depend
