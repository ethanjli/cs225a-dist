# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.8.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.8.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/Morris-Downing/documents/225project/cs225a-dist

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/Morris-Downing/documents/225project/cs225a-dist/src

# Include any dependencies generated for this target.
include src/demo_project/CMakeFiles/demo_project.dir/depend.make

# Include the progress variables for this target.
include src/demo_project/CMakeFiles/demo_project.dir/progress.make

# Include the compile flags for this target's objects.
include src/demo_project/CMakeFiles/demo_project.dir/flags.make

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o: src/demo_project/CMakeFiles/demo_project.dir/flags.make
src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o: redis/RedisClient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp > CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.i

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp -o CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.s

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.requires:

.PHONY : src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.requires

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.provides: src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.requires
	$(MAKE) -f src/demo_project/CMakeFiles/demo_project.dir/build.make src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.provides.build
.PHONY : src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.provides

src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.provides.build: src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o


src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o: src/demo_project/CMakeFiles/demo_project.dir/flags.make
src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o: timer/LoopTimer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp

src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp > CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.i

src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp -o CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.s

src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.requires:

.PHONY : src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.requires

src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.provides: src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.requires
	$(MAKE) -f src/demo_project/CMakeFiles/demo_project.dir/build.make src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.provides.build
.PHONY : src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.provides

src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.provides.build: src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o


src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o: src/demo_project/CMakeFiles/demo_project.dir/flags.make
src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o: demo_project/DemoProject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo_project.dir/DemoProject.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/demo_project/DemoProject.cpp

src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo_project.dir/DemoProject.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/demo_project/DemoProject.cpp > CMakeFiles/demo_project.dir/DemoProject.cpp.i

src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo_project.dir/DemoProject.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/demo_project/DemoProject.cpp -o CMakeFiles/demo_project.dir/DemoProject.cpp.s

src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.requires:

.PHONY : src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.requires

src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.provides: src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.requires
	$(MAKE) -f src/demo_project/CMakeFiles/demo_project.dir/build.make src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.provides.build
.PHONY : src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.provides

src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.provides.build: src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o


# Object files for target demo_project
demo_project_OBJECTS = \
"CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o" \
"CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o" \
"CMakeFiles/demo_project.dir/DemoProject.cpp.o"

# External object files for target demo_project
demo_project_EXTERNAL_OBJECTS =

../bin/demo_project: src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o
../bin/demo_project: src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o
../bin/demo_project: src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o
../bin/demo_project: src/demo_project/CMakeFiles/demo_project.dir/build.make
../bin/demo_project: /Users/Morris-Downing/documents/cs327a/sai2-common/build/libsai2-common.a
../bin/demo_project: /usr/local/lib/libtinyxml2.dylib
../bin/demo_project: /Users/Morris-Downing/Documents/cs327a/chai3d/build/libchai3d.a
../bin/demo_project: /Users/Morris-Downing/documents/cs327a/sai2-simulation/build/libsai2-simulation.a
../bin/demo_project: /usr/local/lib/librbdl.dylib
../bin/demo_project: /usr/local/lib/librbdl_urdfreader.dylib
../bin/demo_project: /usr/local/lib/libhiredis.dylib
../bin/demo_project: /usr/local/lib/libglfw.dylib
../bin/demo_project: /usr/local/lib/libjsoncpp.dylib
../bin/demo_project: src/demo_project/CMakeFiles/demo_project.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../../bin/demo_project"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo_project.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/demo_project/CMakeFiles/demo_project.dir/build: ../bin/demo_project

.PHONY : src/demo_project/CMakeFiles/demo_project.dir/build

src/demo_project/CMakeFiles/demo_project.dir/requires: src/demo_project/CMakeFiles/demo_project.dir/__/redis/RedisClient.cpp.o.requires
src/demo_project/CMakeFiles/demo_project.dir/requires: src/demo_project/CMakeFiles/demo_project.dir/__/timer/LoopTimer.cpp.o.requires
src/demo_project/CMakeFiles/demo_project.dir/requires: src/demo_project/CMakeFiles/demo_project.dir/DemoProject.cpp.o.requires

.PHONY : src/demo_project/CMakeFiles/demo_project.dir/requires

src/demo_project/CMakeFiles/demo_project.dir/clean:
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project && $(CMAKE_COMMAND) -P CMakeFiles/demo_project.dir/cmake_clean.cmake
.PHONY : src/demo_project/CMakeFiles/demo_project.dir/clean

src/demo_project/CMakeFiles/demo_project.dir/depend:
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Morris-Downing/documents/225project/cs225a-dist /Users/Morris-Downing/documents/225project/cs225a-dist/src/demo_project /Users/Morris-Downing/documents/225project/cs225a-dist/src /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/demo_project/CMakeFiles/demo_project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/demo_project/CMakeFiles/demo_project.dir/depend

