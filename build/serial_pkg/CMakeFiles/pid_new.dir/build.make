# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/win/cmake-3.13.0/bin/cmake

# The command to remove a file.
RM = /home/win/cmake-3.13.0/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/win/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/win/catkin_ws/build

# Include any dependencies generated for this target.
include serial_pkg/CMakeFiles/pid_new.dir/depend.make

# Include the progress variables for this target.
include serial_pkg/CMakeFiles/pid_new.dir/progress.make

# Include the compile flags for this target's objects.
include serial_pkg/CMakeFiles/pid_new.dir/flags.make

serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.o: serial_pkg/CMakeFiles/pid_new.dir/flags.make
serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.o: /home/win/catkin_ws/src/serial_pkg/src/pid_new.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/win/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.o"
	cd /home/win/catkin_ws/build/serial_pkg && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pid_new.dir/src/pid_new.cpp.o -c /home/win/catkin_ws/src/serial_pkg/src/pid_new.cpp

serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pid_new.dir/src/pid_new.cpp.i"
	cd /home/win/catkin_ws/build/serial_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/win/catkin_ws/src/serial_pkg/src/pid_new.cpp > CMakeFiles/pid_new.dir/src/pid_new.cpp.i

serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pid_new.dir/src/pid_new.cpp.s"
	cd /home/win/catkin_ws/build/serial_pkg && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/win/catkin_ws/src/serial_pkg/src/pid_new.cpp -o CMakeFiles/pid_new.dir/src/pid_new.cpp.s

# Object files for target pid_new
pid_new_OBJECTS = \
"CMakeFiles/pid_new.dir/src/pid_new.cpp.o"

# External object files for target pid_new
pid_new_EXTERNAL_OBJECTS =

/home/win/catkin_ws/devel/lib/libpid_new.so: serial_pkg/CMakeFiles/pid_new.dir/src/pid_new.cpp.o
/home/win/catkin_ws/devel/lib/libpid_new.so: serial_pkg/CMakeFiles/pid_new.dir/build.make
/home/win/catkin_ws/devel/lib/libpid_new.so: serial_pkg/CMakeFiles/pid_new.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/win/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/win/catkin_ws/devel/lib/libpid_new.so"
	cd /home/win/catkin_ws/build/serial_pkg && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pid_new.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
serial_pkg/CMakeFiles/pid_new.dir/build: /home/win/catkin_ws/devel/lib/libpid_new.so

.PHONY : serial_pkg/CMakeFiles/pid_new.dir/build

serial_pkg/CMakeFiles/pid_new.dir/clean:
	cd /home/win/catkin_ws/build/serial_pkg && $(CMAKE_COMMAND) -P CMakeFiles/pid_new.dir/cmake_clean.cmake
.PHONY : serial_pkg/CMakeFiles/pid_new.dir/clean

serial_pkg/CMakeFiles/pid_new.dir/depend:
	cd /home/win/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/win/catkin_ws/src /home/win/catkin_ws/src/serial_pkg /home/win/catkin_ws/build /home/win/catkin_ws/build/serial_pkg /home/win/catkin_ws/build/serial_pkg/CMakeFiles/pid_new.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_pkg/CMakeFiles/pid_new.dir/depend

