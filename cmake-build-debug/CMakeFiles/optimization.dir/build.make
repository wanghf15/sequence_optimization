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
CMAKE_COMMAND = /home/wanghf/installs/clion-2017.2.3/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/wanghf/installs/clion-2017.2.3/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/wanghf/dev/momenta/sequence_optimization

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/optimization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/optimization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/optimization.dir/flags.make

CMakeFiles/optimization.dir/main.cpp.o: CMakeFiles/optimization.dir/flags.make
CMakeFiles/optimization.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/optimization.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/optimization.dir/main.cpp.o -c /home/wanghf/dev/momenta/sequence_optimization/main.cpp

CMakeFiles/optimization.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/optimization.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/wanghf/dev/momenta/sequence_optimization/main.cpp > CMakeFiles/optimization.dir/main.cpp.i

CMakeFiles/optimization.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/optimization.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/wanghf/dev/momenta/sequence_optimization/main.cpp -o CMakeFiles/optimization.dir/main.cpp.s

CMakeFiles/optimization.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/optimization.dir/main.cpp.o.requires

CMakeFiles/optimization.dir/main.cpp.o.provides: CMakeFiles/optimization.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/optimization.dir/build.make CMakeFiles/optimization.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/optimization.dir/main.cpp.o.provides

CMakeFiles/optimization.dir/main.cpp.o.provides.build: CMakeFiles/optimization.dir/main.cpp.o


# Object files for target optimization
optimization_OBJECTS = \
"CMakeFiles/optimization.dir/main.cpp.o"

# External object files for target optimization
optimization_EXTERNAL_OBJECTS =

optimization: CMakeFiles/optimization.dir/main.cpp.o
optimization: CMakeFiles/optimization.dir/build.make
optimization: /usr/local/lib/libceres.a
optimization: /usr/lib/x86_64-linux-gnu/libglog.so
optimization: /usr/lib/x86_64-linux-gnu/libgflags.so
optimization: /usr/lib/x86_64-linux-gnu/libspqr.so
optimization: /usr/lib/x86_64-linux-gnu/libcholmod.so
optimization: /usr/lib/x86_64-linux-gnu/libccolamd.so
optimization: /usr/lib/x86_64-linux-gnu/libcamd.so
optimization: /usr/lib/x86_64-linux-gnu/libcolamd.so
optimization: /usr/lib/x86_64-linux-gnu/libamd.so
optimization: /usr/lib/liblapack.so
optimization: /usr/lib/libblas.so
optimization: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
optimization: /usr/lib/x86_64-linux-gnu/librt.so
optimization: /usr/lib/x86_64-linux-gnu/libcxsparse.so
optimization: /usr/lib/x86_64-linux-gnu/libspqr.so
optimization: /usr/lib/x86_64-linux-gnu/libcholmod.so
optimization: /usr/lib/x86_64-linux-gnu/libccolamd.so
optimization: /usr/lib/x86_64-linux-gnu/libcamd.so
optimization: /usr/lib/x86_64-linux-gnu/libcolamd.so
optimization: /usr/lib/x86_64-linux-gnu/libamd.so
optimization: /usr/lib/liblapack.so
optimization: /usr/lib/libblas.so
optimization: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
optimization: /usr/lib/x86_64-linux-gnu/librt.so
optimization: /usr/lib/x86_64-linux-gnu/libcxsparse.so
optimization: CMakeFiles/optimization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable optimization"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/optimization.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/optimization.dir/build: optimization

.PHONY : CMakeFiles/optimization.dir/build

CMakeFiles/optimization.dir/requires: CMakeFiles/optimization.dir/main.cpp.o.requires

.PHONY : CMakeFiles/optimization.dir/requires

CMakeFiles/optimization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/optimization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/optimization.dir/clean

CMakeFiles/optimization.dir/depend:
	cd /home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wanghf/dev/momenta/sequence_optimization /home/wanghf/dev/momenta/sequence_optimization /home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug /home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug /home/wanghf/dev/momenta/sequence_optimization/cmake-build-debug/CMakeFiles/optimization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/optimization.dir/depend

