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
CMAKE_SOURCE_DIR = /home/win/catkin_ws/src/serial_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/win/catkin_ws/build_isolated/serial_pkg

# Include any dependencies generated for this target.
include CMakeFiles/yolo_receive.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/yolo_receive.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/yolo_receive.dir/flags.make

CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o: CMakeFiles/yolo_receive.dir/flags.make
CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o: /home/win/catkin_ws/src/serial_pkg/src/yolo_receive.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/win/catkin_ws/build_isolated/serial_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o -c /home/win/catkin_ws/src/serial_pkg/src/yolo_receive.cpp

CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/win/catkin_ws/src/serial_pkg/src/yolo_receive.cpp > CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.i

CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/win/catkin_ws/src/serial_pkg/src/yolo_receive.cpp -o CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.s

# Object files for target yolo_receive
yolo_receive_OBJECTS = \
"CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o"

# External object files for target yolo_receive
yolo_receive_EXTERNAL_OBJECTS =

/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: CMakeFiles/yolo_receive.dir/src/yolo_receive.cpp.o
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: CMakeFiles/yolo_receive.dir/build.make
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libcv_bridge.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libimage_transport.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libmessage_filters.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libclass_loader.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/libPocoFoundation.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libdl.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libroslib.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/librospack.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libroscpp.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/librosconsole.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/librostime.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libcpp_common.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: /opt/ros/melodic/lib/libserial.so
/home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive: CMakeFiles/yolo_receive.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/win/catkin_ws/build_isolated/serial_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yolo_receive.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/yolo_receive.dir/build: /home/win/catkin_ws/devel_isolated/serial_pkg/lib/serial_pkg/yolo_receive

.PHONY : CMakeFiles/yolo_receive.dir/build

CMakeFiles/yolo_receive.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/yolo_receive.dir/cmake_clean.cmake
.PHONY : CMakeFiles/yolo_receive.dir/clean

CMakeFiles/yolo_receive.dir/depend:
	cd /home/win/catkin_ws/build_isolated/serial_pkg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/win/catkin_ws/src/serial_pkg /home/win/catkin_ws/src/serial_pkg /home/win/catkin_ws/build_isolated/serial_pkg /home/win/catkin_ws/build_isolated/serial_pkg /home/win/catkin_ws/build_isolated/serial_pkg/CMakeFiles/yolo_receive.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/yolo_receive.dir/depend

