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
include src/kinematics/CMakeFiles/kinematics.dir/depend.make

# Include the progress variables for this target.
include src/kinematics/CMakeFiles/kinematics.dir/progress.make

# Include the compile flags for this target's objects.
include src/kinematics/CMakeFiles/kinematics.dir/flags.make

src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.o: src/kinematics/CMakeFiles/kinematics.dir/flags.make
src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.o: ../src/kinematics/dummy.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.o"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/kinematics.dir/dummy.cpp.o -c /home/ubuntu/CMM/a1-yuliangzhong/src/kinematics/dummy.cpp

src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/kinematics.dir/dummy.cpp.i"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/CMM/a1-yuliangzhong/src/kinematics/dummy.cpp > CMakeFiles/kinematics.dir/dummy.cpp.i

src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/kinematics.dir/dummy.cpp.s"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/CMM/a1-yuliangzhong/src/kinematics/dummy.cpp -o CMakeFiles/kinematics.dir/dummy.cpp.s

# Object files for target kinematics
kinematics_OBJECTS = \
"CMakeFiles/kinematics.dir/dummy.cpp.o"

# External object files for target kinematics
kinematics_EXTERNAL_OBJECTS =

src/kinematics/libkinematics.a: src/kinematics/CMakeFiles/kinematics.dir/dummy.cpp.o
src/kinematics/libkinematics.a: src/kinematics/CMakeFiles/kinematics.dir/build.make
src/kinematics/libkinematics.a: src/kinematics/CMakeFiles/kinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libkinematics.a"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && $(CMAKE_COMMAND) -P CMakeFiles/kinematics.dir/cmake_clean_target.cmake
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/kinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/kinematics/CMakeFiles/kinematics.dir/build: src/kinematics/libkinematics.a

.PHONY : src/kinematics/CMakeFiles/kinematics.dir/build

src/kinematics/CMakeFiles/kinematics.dir/clean:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics && $(CMAKE_COMMAND) -P CMakeFiles/kinematics.dir/cmake_clean.cmake
.PHONY : src/kinematics/CMakeFiles/kinematics.dir/clean

src/kinematics/CMakeFiles/kinematics.dir/depend:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/CMM/a1-yuliangzhong /home/ubuntu/CMM/a1-yuliangzhong/src/kinematics /home/ubuntu/CMM/a1-yuliangzhong/build /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics /home/ubuntu/CMM/a1-yuliangzhong/build/src/kinematics/CMakeFiles/kinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/kinematics/CMakeFiles/kinematics.dir/depend

