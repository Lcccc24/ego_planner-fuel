# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/uav1/fuel_ws/build/quadrotor_msgs

# Include any dependencies generated for this target.
include CMakeFiles/encode_msgs.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/encode_msgs.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/encode_msgs.dir/flags.make

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o: CMakeFiles/encode_msgs.dir/flags.make
CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o: /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs/src/encode_msgs.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/uav1/fuel_ws/build/quadrotor_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o -c /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs/src/encode_msgs.cpp

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs/src/encode_msgs.cpp > CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.i

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs/src/encode_msgs.cpp -o CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.s

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.requires:

.PHONY : CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.requires

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.provides: CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.requires
	$(MAKE) -f CMakeFiles/encode_msgs.dir/build.make CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.provides.build
.PHONY : CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.provides

CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.provides.build: CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o


# Object files for target encode_msgs
encode_msgs_OBJECTS = \
"CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o"

# External object files for target encode_msgs
encode_msgs_EXTERNAL_OBJECTS =

/home/uav1/fuel_ws/devel/.private/quadrotor_msgs/lib/libencode_msgs.so: CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o
/home/uav1/fuel_ws/devel/.private/quadrotor_msgs/lib/libencode_msgs.so: CMakeFiles/encode_msgs.dir/build.make
/home/uav1/fuel_ws/devel/.private/quadrotor_msgs/lib/libencode_msgs.so: CMakeFiles/encode_msgs.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/uav1/fuel_ws/build/quadrotor_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/uav1/fuel_ws/devel/.private/quadrotor_msgs/lib/libencode_msgs.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/encode_msgs.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/encode_msgs.dir/build: /home/uav1/fuel_ws/devel/.private/quadrotor_msgs/lib/libencode_msgs.so

.PHONY : CMakeFiles/encode_msgs.dir/build

CMakeFiles/encode_msgs.dir/requires: CMakeFiles/encode_msgs.dir/src/encode_msgs.cpp.o.requires

.PHONY : CMakeFiles/encode_msgs.dir/requires

CMakeFiles/encode_msgs.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/encode_msgs.dir/cmake_clean.cmake
.PHONY : CMakeFiles/encode_msgs.dir/clean

CMakeFiles/encode_msgs.dir/depend:
	cd /home/uav1/fuel_ws/build/quadrotor_msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs /home/uav1/fuel_ws/src/exploration/FUEL/uav_simulator/Utils/quadrotor_msgs /home/uav1/fuel_ws/build/quadrotor_msgs /home/uav1/fuel_ws/build/quadrotor_msgs /home/uav1/fuel_ws/build/quadrotor_msgs/CMakeFiles/encode_msgs.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/encode_msgs.dir/depend

