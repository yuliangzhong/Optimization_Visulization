# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/CMM/a1-yuliangzhong

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/CMM/a1-yuliangzhong/build

# Include any dependencies generated for this target.
include src/test-a1/CMakeFiles/test-a1.dir/depend.make

# Include the progress variables for this target.
include src/test-a1/CMakeFiles/test-a1.dir/progress.make

# Include the compile flags for this target's objects.
include src/test-a1/CMakeFiles/test-a1.dir/flags.make

src/test-a1/CMakeFiles/test-a1.dir/test.cpp.o: src/test-a1/CMakeFiles/test-a1.dir/flags.make
src/test-a1/CMakeFiles/test-a1.dir/test.cpp.o: ../src/test-a1/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/test-a1/CMakeFiles/test-a1.dir/test.cpp.o"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test-a1.dir/test.cpp.o -c /home/ubuntu/CMM/a1-yuliangzhong/src/test-a1/test.cpp

src/test-a1/CMakeFiles/test-a1.dir/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test-a1.dir/test.cpp.i"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/CMM/a1-yuliangzhong/src/test-a1/test.cpp > CMakeFiles/test-a1.dir/test.cpp.i

src/test-a1/CMakeFiles/test-a1.dir/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test-a1.dir/test.cpp.s"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/CMM/a1-yuliangzhong/src/test-a1/test.cpp -o CMakeFiles/test-a1.dir/test.cpp.s

# Object files for target test-a1
test__a1_OBJECTS = \
"CMakeFiles/test-a1.dir/test.cpp.o"

# External object files for target test-a1
test__a1_EXTERNAL_OBJECTS =

src/test-a1/test-a1: src/test-a1/CMakeFiles/test-a1.dir/test.cpp.o
src/test-a1/test-a1: src/test-a1/CMakeFiles/test-a1.dir/build.make
src/test-a1/test-a1: src/kinematics/libkinematics.a
src/test-a1/test-a1: src/optimization/liboptimization.a
src/test-a1/test-a1: src/test-a1/CMakeFiles/test-a1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test-a1"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test-a1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/test-a1/CMakeFiles/test-a1.dir/build: src/test-a1/test-a1

.PHONY : src/test-a1/CMakeFiles/test-a1.dir/build

src/test-a1/CMakeFiles/test-a1.dir/clean:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 && $(CMAKE_COMMAND) -P CMakeFiles/test-a1.dir/cmake_clean.cmake
.PHONY : src/test-a1/CMakeFiles/test-a1.dir/clean

src/test-a1/CMakeFiles/test-a1.dir/depend:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/CMM/a1-yuliangzhong /home/ubuntu/CMM/a1-yuliangzhong/src/test-a1 /home/ubuntu/CMM/a1-yuliangzhong/build /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1 /home/ubuntu/CMM/a1-yuliangzhong/build/src/test-a1/CMakeFiles/test-a1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/test-a1/CMakeFiles/test-a1.dir/depend

