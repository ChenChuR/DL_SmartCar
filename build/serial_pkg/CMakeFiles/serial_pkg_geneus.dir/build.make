# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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
CMAKE_COMMAND = /home/win/.local/lib/python3.6/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/win/.local/lib/python3.6/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/win/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/win/catkin_ws/build

# Utility rule file for serial_pkg_geneus.

# Include any custom commands dependencies for this target.
include serial_pkg/CMakeFiles/serial_pkg_geneus.dir/compiler_depend.make

# Include the progress variables for this target.
include serial_pkg/CMakeFiles/serial_pkg_geneus.dir/progress.make

serial_pkg_geneus: serial_pkg/CMakeFiles/serial_pkg_geneus.dir/build.make
.PHONY : serial_pkg_geneus

# Rule to build all files generated by this target.
serial_pkg/CMakeFiles/serial_pkg_geneus.dir/build: serial_pkg_geneus
.PHONY : serial_pkg/CMakeFiles/serial_pkg_geneus.dir/build

serial_pkg/CMakeFiles/serial_pkg_geneus.dir/clean:
	cd /home/win/catkin_ws/build/serial_pkg && $(CMAKE_COMMAND) -P CMakeFiles/serial_pkg_geneus.dir/cmake_clean.cmake
.PHONY : serial_pkg/CMakeFiles/serial_pkg_geneus.dir/clean

serial_pkg/CMakeFiles/serial_pkg_geneus.dir/depend:
	cd /home/win/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/win/catkin_ws/src /home/win/catkin_ws/src/serial_pkg /home/win/catkin_ws/build /home/win/catkin_ws/build/serial_pkg /home/win/catkin_ws/build/serial_pkg/CMakeFiles/serial_pkg_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : serial_pkg/CMakeFiles/serial_pkg_geneus.dir/depend

