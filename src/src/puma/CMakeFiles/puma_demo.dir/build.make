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
include src/puma/CMakeFiles/puma_demo.dir/depend.make

# Include the progress variables for this target.
include src/puma/CMakeFiles/puma_demo.dir/progress.make

# Include the compile flags for this target's objects.
include src/puma/CMakeFiles/puma_demo.dir/flags.make

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o: src/puma/CMakeFiles/puma_demo.dir/flags.make
src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o: redis/RedisClient.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp > CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.i

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/redis/RedisClient.cpp -o CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.s

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.requires:

.PHONY : src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.requires

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.provides: src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.requires
	$(MAKE) -f src/puma/CMakeFiles/puma_demo.dir/build.make src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.provides.build
.PHONY : src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.provides

src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.provides.build: src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o


src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o: src/puma/CMakeFiles/puma_demo.dir/flags.make
src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o: timer/LoopTimer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp

src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp > CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.i

src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/timer/LoopTimer.cpp -o CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.s

src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.requires:

.PHONY : src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.requires

src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.provides: src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.requires
	$(MAKE) -f src/puma/CMakeFiles/puma_demo.dir/build.make src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.provides.build
.PHONY : src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.provides

src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.provides.build: src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o


src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o: src/puma/CMakeFiles/puma_demo.dir/flags.make
src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o: puma/DemoController.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/puma_demo.dir/DemoController.cpp.o -c /Users/Morris-Downing/documents/225project/cs225a-dist/src/puma/DemoController.cpp

src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/puma_demo.dir/DemoController.cpp.i"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/Morris-Downing/documents/225project/cs225a-dist/src/puma/DemoController.cpp > CMakeFiles/puma_demo.dir/DemoController.cpp.i

src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/puma_demo.dir/DemoController.cpp.s"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && /Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/Morris-Downing/documents/225project/cs225a-dist/src/puma/DemoController.cpp -o CMakeFiles/puma_demo.dir/DemoController.cpp.s

src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.requires:

.PHONY : src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.requires

src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.provides: src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.requires
	$(MAKE) -f src/puma/CMakeFiles/puma_demo.dir/build.make src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.provides.build
.PHONY : src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.provides

src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.provides.build: src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o


# Object files for target puma_demo
puma_demo_OBJECTS = \
"CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o" \
"CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o" \
"CMakeFiles/puma_demo.dir/DemoController.cpp.o"

# External object files for target puma_demo
puma_demo_EXTERNAL_OBJECTS =

../bin/puma_demo: src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o
../bin/puma_demo: src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o
../bin/puma_demo: src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o
../bin/puma_demo: src/puma/CMakeFiles/puma_demo.dir/build.make
../bin/puma_demo: /Users/Morris-Downing/documents/cs327a/sai2-common/build/libsai2-common.a
../bin/puma_demo: /usr/local/lib/libtinyxml2.dylib
../bin/puma_demo: /Users/Morris-Downing/Documents/cs327a/chai3d/build/libchai3d.a
../bin/puma_demo: /Users/Morris-Downing/documents/cs327a/sai2-simulation/build/libsai2-simulation.a
../bin/puma_demo: /usr/local/lib/librbdl.dylib
../bin/puma_demo: /usr/local/lib/librbdl_urdfreader.dylib
../bin/puma_demo: /usr/local/lib/libhiredis.dylib
../bin/puma_demo: /usr/local/lib/libglfw.dylib
../bin/puma_demo: /usr/local/lib/libjsoncpp.dylib
../bin/puma_demo: src/puma/CMakeFiles/puma_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/Morris-Downing/documents/225project/cs225a-dist/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable ../../../bin/puma_demo"
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/puma_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/puma/CMakeFiles/puma_demo.dir/build: ../bin/puma_demo

.PHONY : src/puma/CMakeFiles/puma_demo.dir/build

src/puma/CMakeFiles/puma_demo.dir/requires: src/puma/CMakeFiles/puma_demo.dir/__/redis/RedisClient.cpp.o.requires
src/puma/CMakeFiles/puma_demo.dir/requires: src/puma/CMakeFiles/puma_demo.dir/__/timer/LoopTimer.cpp.o.requires
src/puma/CMakeFiles/puma_demo.dir/requires: src/puma/CMakeFiles/puma_demo.dir/DemoController.cpp.o.requires

.PHONY : src/puma/CMakeFiles/puma_demo.dir/requires

src/puma/CMakeFiles/puma_demo.dir/clean:
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma && $(CMAKE_COMMAND) -P CMakeFiles/puma_demo.dir/cmake_clean.cmake
.PHONY : src/puma/CMakeFiles/puma_demo.dir/clean

src/puma/CMakeFiles/puma_demo.dir/depend:
	cd /Users/Morris-Downing/documents/225project/cs225a-dist/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/Morris-Downing/documents/225project/cs225a-dist /Users/Morris-Downing/documents/225project/cs225a-dist/src/puma /Users/Morris-Downing/documents/225project/cs225a-dist/src /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma /Users/Morris-Downing/documents/225project/cs225a-dist/src/src/puma/CMakeFiles/puma_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/puma/CMakeFiles/puma_demo.dir/depend

