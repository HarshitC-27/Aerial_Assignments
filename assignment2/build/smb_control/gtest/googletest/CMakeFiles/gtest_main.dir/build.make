# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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
CMAKE_SOURCE_DIR = /home/harshit/Aerial_Assignments/assignment2/src/smb_control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/harshit/Aerial_Assignments/assignment2/build/smb_control

# Include any dependencies generated for this target.
include gtest/googletest/CMakeFiles/gtest_main.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include gtest/googletest/CMakeFiles/gtest_main.dir/compiler_depend.make

# Include the progress variables for this target.
include gtest/googletest/CMakeFiles/gtest_main.dir/progress.make

# Include the compile flags for this target's objects.
include gtest/googletest/CMakeFiles/gtest_main.dir/flags.make

gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o: gtest/googletest/CMakeFiles/gtest_main.dir/flags.make
gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o: /usr/src/googletest/googletest/src/gtest_main.cc
gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o: gtest/googletest/CMakeFiles/gtest_main.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/harshit/Aerial_Assignments/assignment2/build/smb_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o"
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o -MF CMakeFiles/gtest_main.dir/src/gtest_main.cc.o.d -o CMakeFiles/gtest_main.dir/src/gtest_main.cc.o -c /usr/src/googletest/googletest/src/gtest_main.cc

gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gtest_main.dir/src/gtest_main.cc.i"
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /usr/src/googletest/googletest/src/gtest_main.cc > CMakeFiles/gtest_main.dir/src/gtest_main.cc.i

gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gtest_main.dir/src/gtest_main.cc.s"
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /usr/src/googletest/googletest/src/gtest_main.cc -o CMakeFiles/gtest_main.dir/src/gtest_main.cc.s

# Object files for target gtest_main
gtest_main_OBJECTS = \
"CMakeFiles/gtest_main.dir/src/gtest_main.cc.o"

# External object files for target gtest_main
gtest_main_EXTERNAL_OBJECTS =

gtest/lib/libgtest_main.so: gtest/googletest/CMakeFiles/gtest_main.dir/src/gtest_main.cc.o
gtest/lib/libgtest_main.so: gtest/googletest/CMakeFiles/gtest_main.dir/build.make
gtest/lib/libgtest_main.so: gtest/lib/libgtest.so
gtest/lib/libgtest_main.so: gtest/googletest/CMakeFiles/gtest_main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/harshit/Aerial_Assignments/assignment2/build/smb_control/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../lib/libgtest_main.so"
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gtest_main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
gtest/googletest/CMakeFiles/gtest_main.dir/build: gtest/lib/libgtest_main.so
.PHONY : gtest/googletest/CMakeFiles/gtest_main.dir/build

gtest/googletest/CMakeFiles/gtest_main.dir/clean:
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest && $(CMAKE_COMMAND) -P CMakeFiles/gtest_main.dir/cmake_clean.cmake
.PHONY : gtest/googletest/CMakeFiles/gtest_main.dir/clean

gtest/googletest/CMakeFiles/gtest_main.dir/depend:
	cd /home/harshit/Aerial_Assignments/assignment2/build/smb_control && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/harshit/Aerial_Assignments/assignment2/src/smb_control /usr/src/googletest/googletest /home/harshit/Aerial_Assignments/assignment2/build/smb_control /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest /home/harshit/Aerial_Assignments/assignment2/build/smb_control/gtest/googletest/CMakeFiles/gtest_main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : gtest/googletest/CMakeFiles/gtest_main.dir/depend

