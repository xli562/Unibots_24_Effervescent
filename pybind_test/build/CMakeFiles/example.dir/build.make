# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.28.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.28.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/yqs/Desktop/pybind_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/yqs/Desktop/pybind_test/build

# Include any dependencies generated for this target.
include CMakeFiles/example.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/example.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/example.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example.dir/flags.make

CMakeFiles/example.dir/test1.cpp.o: CMakeFiles/example.dir/flags.make
CMakeFiles/example.dir/test1.cpp.o: /Users/yqs/Desktop/pybind_test/test1.cpp
CMakeFiles/example.dir/test1.cpp.o: CMakeFiles/example.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/yqs/Desktop/pybind_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example.dir/test1.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/example.dir/test1.cpp.o -MF CMakeFiles/example.dir/test1.cpp.o.d -o CMakeFiles/example.dir/test1.cpp.o -c /Users/yqs/Desktop/pybind_test/test1.cpp

CMakeFiles/example.dir/test1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/example.dir/test1.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/yqs/Desktop/pybind_test/test1.cpp > CMakeFiles/example.dir/test1.cpp.i

CMakeFiles/example.dir/test1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/example.dir/test1.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/yqs/Desktop/pybind_test/test1.cpp -o CMakeFiles/example.dir/test1.cpp.s

# Object files for target example
example_OBJECTS = \
"CMakeFiles/example.dir/test1.cpp.o"

# External object files for target example
example_EXTERNAL_OBJECTS =

example.cpython-39-darwin.so: CMakeFiles/example.dir/test1.cpp.o
example.cpython-39-darwin.so: CMakeFiles/example.dir/build.make
example.cpython-39-darwin.so: CMakeFiles/example.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/yqs/Desktop/pybind_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module example.cpython-39-darwin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example.dir/build: example.cpython-39-darwin.so
.PHONY : CMakeFiles/example.dir/build

CMakeFiles/example.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example.dir/clean

CMakeFiles/example.dir/depend:
	cd /Users/yqs/Desktop/pybind_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/yqs/Desktop/pybind_test /Users/yqs/Desktop/pybind_test /Users/yqs/Desktop/pybind_test/build /Users/yqs/Desktop/pybind_test/build /Users/yqs/Desktop/pybind_test/build/CMakeFiles/example.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/example.dir/depend
