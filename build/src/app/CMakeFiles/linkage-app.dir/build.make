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
include src/app/CMakeFiles/linkage-app.dir/depend.make

# Include the progress variables for this target.
include src/app/CMakeFiles/linkage-app.dir/progress.make

# Include the compile flags for this target's objects.
include src/app/CMakeFiles/linkage-app.dir/flags.make

src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.o: src/app/CMakeFiles/linkage-app.dir/flags.make
src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.o: ../src/app/linkage-app.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.o"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/linkage-app.dir/linkage-app.cpp.o -c /home/ubuntu/CMM/a1-yuliangzhong/src/app/linkage-app.cpp

src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/linkage-app.dir/linkage-app.cpp.i"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/CMM/a1-yuliangzhong/src/app/linkage-app.cpp > CMakeFiles/linkage-app.dir/linkage-app.cpp.i

src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/linkage-app.dir/linkage-app.cpp.s"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/app && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/CMM/a1-yuliangzhong/src/app/linkage-app.cpp -o CMakeFiles/linkage-app.dir/linkage-app.cpp.s

# Object files for target linkage-app
linkage__app_OBJECTS = \
"CMakeFiles/linkage-app.dir/linkage-app.cpp.o"

# External object files for target linkage-app
linkage__app_EXTERNAL_OBJECTS =

src/app/linkage-app: src/app/CMakeFiles/linkage-app.dir/linkage-app.cpp.o
src/app/linkage-app: src/app/CMakeFiles/linkage-app.dir/build.make
src/app/linkage-app: src/kinematics/libkinematics.a
src/app/linkage-app: src/gui/libgui.a
src/app/linkage-app: src/optimization/liboptimization.a
src/app/linkage-app: ext/glad/libglad.a
src/app/linkage-app: /usr/lib/x86_64-linux-gnu/libGL.so
src/app/linkage-app: /usr/lib/x86_64-linux-gnu/libGLU.so
src/app/linkage-app: ext/ext/glfw/src/libglfw3.a
src/app/linkage-app: /usr/lib/x86_64-linux-gnu/librt.so
src/app/linkage-app: /usr/lib/x86_64-linux-gnu/libm.so
src/app/linkage-app: /usr/lib/x86_64-linux-gnu/libX11.so
src/app/linkage-app: ext/libnanovg.a
src/app/linkage-app: ext/libimgui.a
src/app/linkage-app: src/app/CMakeFiles/linkage-app.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/CMM/a1-yuliangzhong/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable linkage-app"
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/linkage-app.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/app/CMakeFiles/linkage-app.dir/build: src/app/linkage-app

.PHONY : src/app/CMakeFiles/linkage-app.dir/build

src/app/CMakeFiles/linkage-app.dir/clean:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build/src/app && $(CMAKE_COMMAND) -P CMakeFiles/linkage-app.dir/cmake_clean.cmake
.PHONY : src/app/CMakeFiles/linkage-app.dir/clean

src/app/CMakeFiles/linkage-app.dir/depend:
	cd /home/ubuntu/CMM/a1-yuliangzhong/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/CMM/a1-yuliangzhong /home/ubuntu/CMM/a1-yuliangzhong/src/app /home/ubuntu/CMM/a1-yuliangzhong/build /home/ubuntu/CMM/a1-yuliangzhong/build/src/app /home/ubuntu/CMM/a1-yuliangzhong/build/src/app/CMakeFiles/linkage-app.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/app/CMakeFiles/linkage-app.dir/depend
