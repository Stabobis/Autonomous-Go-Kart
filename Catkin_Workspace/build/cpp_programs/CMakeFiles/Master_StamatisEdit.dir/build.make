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
CMAKE_SOURCE_DIR = /home/stamatis/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/stamatis/catkin_ws/build

# Include any dependencies generated for this target.
include cpp_programs/CMakeFiles/Master_StamatisEdit.dir/depend.make

# Include the progress variables for this target.
include cpp_programs/CMakeFiles/Master_StamatisEdit.dir/progress.make

# Include the compile flags for this target's objects.
include cpp_programs/CMakeFiles/Master_StamatisEdit.dir/flags.make

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/flags.make
cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o: /home/stamatis/catkin_ws/src/cpp_programs/src/ScratchCode/Master_StamatisEdit.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/stamatis/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o"
	cd /home/stamatis/catkin_ws/build/cpp_programs && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o -c /home/stamatis/catkin_ws/src/cpp_programs/src/ScratchCode/Master_StamatisEdit.cpp

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.i"
	cd /home/stamatis/catkin_ws/build/cpp_programs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/stamatis/catkin_ws/src/cpp_programs/src/ScratchCode/Master_StamatisEdit.cpp > CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.i

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.s"
	cd /home/stamatis/catkin_ws/build/cpp_programs && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/stamatis/catkin_ws/src/cpp_programs/src/ScratchCode/Master_StamatisEdit.cpp -o CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.s

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.requires:

.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.requires

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.provides: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.requires
	$(MAKE) -f cpp_programs/CMakeFiles/Master_StamatisEdit.dir/build.make cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.provides.build
.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.provides

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.provides.build: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o


# Object files for target Master_StamatisEdit
Master_StamatisEdit_OBJECTS = \
"CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o"

# External object files for target Master_StamatisEdit
Master_StamatisEdit_EXTERNAL_OBJECTS =

/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/build.make
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/libroscpp.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/librosconsole.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/librostime.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /opt/ros/melodic/lib/libcpp_common.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/stamatis/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit"
	cd /home/stamatis/catkin_ws/build/cpp_programs && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Master_StamatisEdit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cpp_programs/CMakeFiles/Master_StamatisEdit.dir/build: /home/stamatis/catkin_ws/devel/lib/cpp_programs/Master_StamatisEdit

.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/build

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/requires: cpp_programs/CMakeFiles/Master_StamatisEdit.dir/src/ScratchCode/Master_StamatisEdit.cpp.o.requires

.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/requires

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/clean:
	cd /home/stamatis/catkin_ws/build/cpp_programs && $(CMAKE_COMMAND) -P CMakeFiles/Master_StamatisEdit.dir/cmake_clean.cmake
.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/clean

cpp_programs/CMakeFiles/Master_StamatisEdit.dir/depend:
	cd /home/stamatis/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/stamatis/catkin_ws/src /home/stamatis/catkin_ws/src/cpp_programs /home/stamatis/catkin_ws/build /home/stamatis/catkin_ws/build/cpp_programs /home/stamatis/catkin_ws/build/cpp_programs/CMakeFiles/Master_StamatisEdit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cpp_programs/CMakeFiles/Master_StamatisEdit.dir/depend

