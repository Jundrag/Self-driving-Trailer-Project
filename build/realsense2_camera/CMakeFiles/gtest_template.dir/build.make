# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/jun/ros2_ws/src/realsense-ros/realsense2_camera

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jun/ros2_ws/build/realsense2_camera

# Include any dependencies generated for this target.
include CMakeFiles/gtest_template.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gtest_template.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gtest_template.dir/flags.make

CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o: CMakeFiles/gtest_template.dir/flags.make
CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o: /home/jun/ros2_ws/src/realsense-ros/realsense2_camera/test/gtest_template.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jun/ros2_ws/build/realsense2_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o -c /home/jun/ros2_ws/src/realsense-ros/realsense2_camera/test/gtest_template.cpp

CMakeFiles/gtest_template.dir/test/gtest_template.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest_template.dir/test/gtest_template.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jun/ros2_ws/src/realsense-ros/realsense2_camera/test/gtest_template.cpp > CMakeFiles/gtest_template.dir/test/gtest_template.cpp.i

CMakeFiles/gtest_template.dir/test/gtest_template.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest_template.dir/test/gtest_template.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jun/ros2_ws/src/realsense-ros/realsense2_camera/test/gtest_template.cpp -o CMakeFiles/gtest_template.dir/test/gtest_template.cpp.s

# Object files for target gtest_template
gtest_template_OBJECTS = \
"CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o"

# External object files for target gtest_template
gtest_template_EXTERNAL_OBJECTS =

gtest_template: CMakeFiles/gtest_template.dir/test/gtest_template.cpp.o
gtest_template: CMakeFiles/gtest_template.dir/build.make
gtest_template: gtest/libgtest_main.a
gtest_template: gtest/libgtest.a
gtest_template: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
gtest_template: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
gtest_template: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
gtest_template: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
gtest_template: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
gtest_template: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
gtest_template: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
gtest_template: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
gtest_template: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
gtest_template: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
gtest_template: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
gtest_template: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
gtest_template: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
gtest_template: /opt/ros/foxy/lib/librosidl_typesupport_c.so
gtest_template: /opt/ros/foxy/lib/librosidl_runtime_c.so
gtest_template: /opt/ros/foxy/lib/librcpputils.so
gtest_template: /opt/ros/foxy/lib/librcutils.so
gtest_template: CMakeFiles/gtest_template.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jun/ros2_ws/build/realsense2_camera/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gtest_template"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest_template.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gtest_template.dir/build: gtest_template

.PHONY : CMakeFiles/gtest_template.dir/build

CMakeFiles/gtest_template.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gtest_template.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gtest_template.dir/clean

CMakeFiles/gtest_template.dir/depend:
	cd /home/jun/ros2_ws/build/realsense2_camera && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jun/ros2_ws/src/realsense-ros/realsense2_camera /home/jun/ros2_ws/src/realsense-ros/realsense2_camera /home/jun/ros2_ws/build/realsense2_camera /home/jun/ros2_ws/build/realsense2_camera /home/jun/ros2_ws/build/realsense2_camera/CMakeFiles/gtest_template.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gtest_template.dir/depend

